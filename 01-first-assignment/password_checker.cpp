#include <iostream>
#include <string>
/*
第5行lower<=ch<=upper；
第16行改为>=10；
第19行0，9改为'0','9'；
第29行删掉password后面大写S；
输入"Cai Xukun Ikun114514"
返回VALID
*/
bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        if (lower <= ch && ch <= upper) {
            return true;//=
        }
    }
    return false;
}

bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    bool length_ok = password.size() >= 10;//>=10
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');//'0'
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;
    int checked = 0;
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::cout << "输入：名 姓 密码:";
    std::string first_name, last_name, password;
    if (!(std::cin >> first_name >> last_name >> password)) {//delet S
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
