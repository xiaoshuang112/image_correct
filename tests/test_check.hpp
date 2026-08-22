// 微型断言工具（头文件内联，避免额外框架依赖）
#ifndef TEST_CHECK_HPP
#define TEST_CHECK_HPP

#include <iostream>
#include <string>

namespace testutil {

inline int& failureCount() {
    static int n = 0;
    return n;
}

inline void report(bool ok, const std::string& name) {
    if (ok) {
        std::cout << "PASS  " << name << "\n";
    } else {
        std::cout << "FAIL  " << name << "\n";
        ++failureCount();
    }
}

inline int takeFailures() {
    const int n = failureCount();
    failureCount() = 0;
    return n;
}

}  // namespace testutil

#define TEST_CHECK(cond, name) ::testutil::report(static_cast<bool>(cond), name)

#endif  // TEST_CHECK_HPP
