#ifndef CANDIDATE_PLANNER_HPP
#define CANDIDATE_PLANNER_HPP

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

// 机器人候选位姿。source_index 是生成网格中的稳定编号，执行排序后不改变。
struct PoseCandidate {
    double axis_angle[6]{};   // 法兰轴角位姿 (mm, deg)
    double flange_euler[6]{}; // 法兰欧拉位姿 (mm, deg)
    double joint[6]{};        // 关节角 (deg)
    double step_from_previous[6]{}; // 相对上一规划节点的逐轴跳变 (deg)
    int source_index = -1;
    int previous_source_index = -1;
    // 过渡点只用于把两个采集点安全连接起来，不触发相机采图。
    bool capture = true;
};

// 使用 reference_joint 作为 IK 参考解重新计算候选关节角。这样最终路径上的
// 每一步都选择最接近上一姿态的连续逆解，而不是始终沿用示教起点的逆解分支。
using CandidateJointResolver = std::function<bool(
    const PoseCandidate& candidate,
    const std::array<double, 6>& reference_joint,
    std::array<double, 6>& resolved_joint,
    int& error_code)>;

struct CandidatePlanningOptions {
    // 0 表示不压缩；半自动标定使用约 90 个代表性候选。
    std::size_t maximum_candidates = 0;
    // 任一关节相邻跳变超过该值时拒绝执行；<=0 表示关闭门禁。
    double maximum_joint_step_degrees = 0.0;
    CandidateJointResolver joint_resolver;
};

struct CandidatePlanningReport {
    std::size_t input_candidates = 0;
    std::size_t selected_candidates = 0;
    std::size_t ordered_candidates = 0;
    std::size_t transition_candidates = 0;
    std::size_t connected_candidates = 0;
    std::size_t disconnected_candidates = 0;
    double estimated_path_degrees = 0.0;
    double maximum_joint_step_degrees = 0.0;
    double maximum_j6_step_degrees = 0.0;
    std::array<double, 6> maximum_axis_step_degrees{};
    int maximum_step_axis = -1;
    int maximum_step_from_source_index = -1;
    int maximum_step_to_source_index = -1;
};

// 对可达候选建立安全关节图，从示教起点的连通分量中选取覆盖目标，并在
// 无法直接安全到达时插入 capture=false 的过渡点。最终路径逐步重新求 IK。
// 失败时 candidates 保持不变。
bool planPoseCandidates(std::vector<PoseCandidate>& candidates,
                        const double initial_joint[6],
                        const CandidatePlanningOptions& options,
                        CandidatePlanningReport& report,
                        std::string& error);

double candidateMaximumJointStep(const double from[6], const double to[6]);
double candidateJointPathDistance(const double from[6], const double to[6]);

#endif // CANDIDATE_PLANNER_HPP
