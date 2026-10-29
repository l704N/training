import rclpy
import math
import numpy as np
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from .climb_traj import VelocityRampFilter 
from .xbox_controller import PygameController 

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller
from .roll_leg_control import RollLegController
from .ground_detection import detect_leg_ground_status

WHEEL_RADIUS = 0.06   # 轮子半径 (m)


def cal_euler(q):
    # 四元数 -> 欧拉角，返回 roll, pitch, yaw
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class PIDController:
    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.feedforward = feedforward
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        self.integral += error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        derivative = error - self.prev_error
        self.prev_error = error
        output = (self.kp * error + self.ki * self.integral
                  + self.kd * derivative + self.feedforward)
        return np.clip(output, -self.max_out, self.max_out)


def control_left_leg(info, five_links, lqr, yaw_pid, L0_pid, pitch, gyro_pitch, gyro_yaw,
                     target_w, target_length, common_motor_pos, common_motor_speed, wheel_radius,
                     is_airborne_last_step, ground_detection_enabled, leg_tp_comp=0.0):
    """左腿 LQR + VMC 核心控制计算与条件离地保护机制"""
    motor1_angle = info['pos']['left1']
    motor2_angle = info['pos']['left2']

    # 1. 运动学与状态更新
    five_links.forward_kinematics_cal(1, motor1_angle, motor2_angle, pitch, gyro_pitch)
    lqr.states_update(1, five_links, pitch, gyro_pitch, common_motor_pos, common_motor_speed, wheel_radius)
    lqr.calc_k_matrix_from_poly(five_links.L_0)

    # 2. VMC 虚拟力基本计算
    F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
    F_0 = (4 / math.cos(five_links.theta) if math.cos(five_links.theta) != 0 else 0) + F_0_ctrl
    F_0 = np.clip(F_0, -100.0, 100.0)

    # 预计算常规状况下的 Tp
    T_p = (lqr.K[1] * (lqr.theta - 0.0)
           + lqr.K[3] * (lqr.d_theta - 0.0)
           + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
           + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
           + lqr.K[9] * (lqr.phi - 0.0)
           + lqr.K[11] * (lqr.d_phi - 0.0))

    # 3. 核心条件判定：离地保护
    if ground_detection_enabled and (detect_leg_ground_status(five_links, F_0, T_p) == 1 or is_airborne_last_step):
        left_wheel_T = 0.0
        T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
        T_p += leg_tp_comp
        flight_flag = 1
    else:
        yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
        wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                   + lqr.K[2] * (lqr.d_theta - 0.0)
                   + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                   + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                   + lqr.K[8] * (lqr.phi - 0.0)
                   + lqr.K[10] * (lqr.d_phi - 0.0))

        left_wheel_T = -wheel_T - yaw_ctrl
        left_wheel_T = np.clip(left_wheel_T, -1.5, 1.5)
        flight_flag = 0

    # 4. 最终 VMC 力矩映射
    torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)

    return -torque_set_0, -torque_set_1, left_wheel_T, flight_flag


def control_right_leg(info, five_links, lqr, yaw_pid, L0_pid, pitch, gyro_pitch, gyro_yaw,
                      target_w, target_length, common_motor_pos, common_motor_speed, wheel_radius,
                      is_airborne_last_step, ground_detection_enabled, leg_tp_comp=0.0):
    """右腿 LQR + VMC 核心控制计算与条件离地保护机制"""
    motor1_angle = info['pos']['right1']
    motor2_angle = info['pos']['right2']

    # 1. 运动学与状态更新
    five_links.forward_kinematics_cal(0, motor1_angle, motor2_angle, pitch, gyro_pitch)
    lqr.states_update(0, five_links, pitch, gyro_pitch, -common_motor_pos, -common_motor_speed, wheel_radius)
    lqr.calc_k_matrix_from_poly(five_links.L_0)

    # 2. VMC 虚拟力基本计算
    F_0_ctrl = L0_pid.compute(five_links.L_0, target_length)
    F_0 = (4 / math.cos(five_links.theta) if math.cos(five_links.theta) != 0 else 0) + F_0_ctrl
    F_0 = np.clip(F_0, -100.0, 100.0)

    # 预计算常规状况下的 Tp
    T_p = (lqr.K[1] * (lqr.theta - 0.0)
           + lqr.K[3] * (lqr.d_theta - 0.0)
           + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
           + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
           + lqr.K[9] * (lqr.phi - 0.0)
           + lqr.K[11] * (lqr.d_phi - 0.0))

    # 3. 核心条件判定
    if ground_detection_enabled and (detect_leg_ground_status(five_links, F_0, T_p) == 1 or is_airborne_last_step):
        right_wheel_T = 0.0
        T_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
        T_p += leg_tp_comp
        flight_flag = 1
    else:
        yaw_ctrl = yaw_pid.compute(gyro_yaw, target_w)
        wheel_T = (lqr.K[0] * (lqr.theta - 0.0)
                   + lqr.K[2] * (lqr.d_theta - 0.0)
                   + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                   + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                   + lqr.K[8] * (lqr.phi - 0.0)
                   + lqr.K[10] * (lqr.d_phi - 0.0))

        right_wheel_T = wheel_T - yaw_ctrl
        right_wheel_T = np.clip(right_wheel_T, -1.5, 1.5)
        flight_flag = 0

    torque_set_0, torque_set_1 = five_links.VMC_torque_cal(F_0, T_p)

    return torque_set_0, torque_set_1, right_wheel_T, flight_flag


class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')
        # 订阅
        self.create_subscription(Imu, 'imu', self.imu_cb, 10)
        self.create_subscription(JointState, 'joint_states', self.joints_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        # 发布
        self.actuator_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        # 定时器dt = 0.002s
        self.timer = self.create_timer(0.002, self.step)

        # 状态
        self.pitch = 0.0
        self.roll = 0.0
        self.gyro_pitch = 0.0
        self.gyro_yaw = 0.0
        self.right_wheel_vel = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_pos = 0.0
        self.v_set = 0.0
        self.current_v = 0.0
        self.info = {'pos': {}, 'vel': {}}

        

        # 算法对象
        self.five_links_left = five_links_param(dt=0.002)
        self.five_links_right = five_links_param(dt=0.002)
        self.lqr_left = lqr_controller(dt=0.002)
        self.lqr_right = lqr_controller(dt=0.002)
        self.left_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)
        self.right_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)
        self.left_L0_pid = PIDController(500.0, 0.0, 5.0, 150.0, 0.0)
        self.right_L0_pid = PIDController(500.0, 0.0, 5.0, 150.0, 0.0)

        # 目标与配置
        self.target_length = RollLegController()
        # 左右腿目标长度的初始值，step() 中每周期用 compute_leg_lengths 更新
        self.target_length_left = 0.1
        self.target_length_right = 0.1
        
        self.target_w = 0.0
        self.robot_is_airborne = False
        #self.ground_detection_enabled = False#滞空检测
        self.declare_parameter('ground_detection_enabled', False)
        self.ground_detection_enabled = self.get_parameter('ground_detection_enabled').value

        # 手柄与速度爬坡滤波器：只创建一次，step 中仅调用 update()
        self.teleop = PygameController()
        self.v_filter = VelocityRampFilter(dt=0.002, accel_rate=0.8, decel_rate=1.4)

    def imu_cb(self, msg):
        roll_raw, pitch_raw, yaw_raw = cal_euler(msg.orientation)
        self.pitch = -pitch_raw
        self.roll = -roll_raw
        self.gyro_pitch = -msg.angular_velocity.y
        self.gyro_yaw = msg.angular_velocity.x
        self.get_logger().info(f'pitch={self.pitch:.3f} rad', throttle_duration_sec=1.0)

    def joints_cb(self, msg):
        # name 顺序：right1,right2,left1,left2,rightwheel,leftwheel
        self.info = {
            'pos': {
                'right1': msg.position[0],
                'right2': msg.position[1],
                'left1': msg.position[2],
                'left2': msg.position[3],
                'rightwheel': msg.position[4],
                'leftwheel': msg.position[5],
            },
            'vel': {
                'right1': msg.velocity[0],
                'right2': msg.velocity[1],
                'left1': msg.velocity[2],
                'left2': msg.velocity[3],
                'rightwheel': msg.velocity[4],
                'leftwheel': msg.velocity[5],
            },
        }
        self.right_wheel_vel = msg.velocity[4]
        self.left_wheel_vel = msg.velocity[5]
        self.right_wheel_pos = msg.position[4]
        self.left_wheel_pos = msg.position[5]

    def cmd_vel_cb(self, msg):
        self.v_set = msg.linear.x

    def step(self):
        # 读取手柄输入并刷新控制面板（teleop/v_filter 在 __init__ 中创建，避免每次 step 重建窗口）
        self.teleop.update(
            current_v=self.right_wheel_vel,
            target_length_left=self.target_length_left,
            target_length_right=self.target_length_right,
            roll=self.roll,
        )
        self.v_set = self.teleop.target_v
        self.target_w = self.teleop.target_w
        self.target_length_left,self.target_length_right=self.target_length.compute_leg_lengths(
                        current_roll=self.roll,
                        target_roll=0.0,
                        base_length=self.teleop.target_length
                    )
        if not self.info['pos']:
            return

        #离地保护
        self.ground_detection_enabled = self.get_parameter('ground_detection_enabled').value

        # 共模（left - right）/ 2
        common_motor_speed = (self.left_wheel_vel - self.right_wheel_vel) / 2.0
        common_motor_pos = (self.left_wheel_pos - self.right_wheel_pos) / 2.0

        # LQR 目标速度：爬坡滤波器的 current 输入必须与 v_set 同单位(m/s)，
        # 用内部状态 current_v 反馈，而不是右轮实测角速度(rad/s)
        self.current_v = self.v_filter.update(self.current_v, self.v_set)
        self.lqr_left.v_set = self.current_v
        self.lqr_right.v_set = self.current_v

        # x_b_set 只在运动时更新（站桩时冻结，否则位置环失效）
        if not (self.current_v == 0.0 and self.v_set == 0):
            self.lqr_left.x_b_set = common_motor_pos * WHEEL_RADIUS
            self.lqr_right.x_b_set = common_motor_pos * WHEEL_RADIUS

        # 左腿
        ctrl_left_0, ctrl_left_1, left_wheel_T, left_flight = control_left_leg(
            self.info, self.five_links_left, self.lqr_left,
            self.left_yaw_pid, self.left_L0_pid,
            self.pitch, self.gyro_pitch, self.gyro_yaw,
            self.target_w, self.target_length_left,
            common_motor_pos, common_motor_speed, WHEEL_RADIUS,
            self.robot_is_airborne, self.ground_detection_enabled,
        )
        # 右腿
        ctrl_right_0, ctrl_right_1, right_wheel_T, right_flight = control_right_leg(
            self.info, self.five_links_right, self.lqr_right,
            self.right_yaw_pid, self.right_L0_pid,
            self.pitch, self.gyro_pitch, self.gyro_yaw,
            self.target_w, self.target_length_right,
            common_motor_pos, common_motor_speed, WHEEL_RADIUS,
            self.robot_is_airborne, self.ground_detection_enabled,
        )

        if self.ground_detection_enabled:
            self.robot_is_airborne = (left_flight == 1) or (right_flight == 1)
        else:
            self.robot_is_airborne = False

        cmd = WheelLegCmd()
        cmd.right_phi1 = ctrl_right_0
        cmd.right_phi4 = ctrl_right_1
        cmd.left_phi1 = ctrl_left_0
        cmd.left_phi4 = ctrl_left_1
        cmd.right_wheel = right_wheel_T
        cmd.left_wheel = left_wheel_T
        self.actuator_pub.publish(cmd)
        #日志
        self.get_logger().info(
        f'L0_left={self.five_links_left.L_0:.4f}, L0_right={self.five_links_right.L_0:.4f}',
        throttle_duration_sec=1.0)


def main(args=None):
    

    rclpy.init(args=args)
    node = WheelLegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
