// 无硬件单元测试入口：MVS 选机策略、错误码诊断、内参门禁、配置解析。
// 全部用例不依赖相机、机器人、MVS SDK 运行库或 Qt。
#include <iostream>
#include <string>

namespace suite {
int test_mvs_device_selector();
int test_mvs_error();
int test_intrinsic_identity();
int test_config_resolver();
int test_candidate_planner();
int test_pose_generation_config();
int test_solver_consensus();
}  // namespace suite

int main() {
    int failures = 0;
    failures += suite::test_mvs_device_selector();
    failures += suite::test_mvs_error();
    failures += suite::test_intrinsic_identity();
    failures += suite::test_config_resolver();
    failures += suite::test_candidate_planner();
    failures += suite::test_pose_generation_config();
    failures += suite::test_solver_consensus();
    if (failures == 0) {
        std::cout << "\n[caitu_tests] ALL PASS\n";
        return 0;
    }
    std::cout << "\n[caitu_tests] " << failures << " FAILURE(S)\n";
    return 1;
}
