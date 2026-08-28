#ifndef CANDIDATE_PLANNER_HPP
#define CANDIDATE_PLANNER_HPP

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// 机器人候选位姿。source_index 是生成网格中的稳定编号，执行排序后不改变。
struct PoseCandidate {
    double axis_angle[6]{};   // 法兰轴角位姿 (mm, deg)
    double flange_euler[6]{}; // 法兰欧拉位姿 (mm, deg)
    double joint[6]{};        // 关节角 (deg)
    int source_index = -1;
};

struct CandidatePlanningOptions {
    // 0 表示不压缩；半自动标定使用约 90 个代表性候选。
    std::size_t maximum_candidates = 0;
    // 任一关节相邻跳变超过该值时拒绝执行；<=0 表示关闭门禁。
    double maximum_joint_step_degrees = 0.0;
};

struct CandidatePlanningReport {
    std::size_t input_candidates = 0;
    std::size_t selected_candidates = 0;
    std::size_t ordered_candidates = 0;
    double estimated_path_degrees = 0.0;
    double maximum_joint_step_degrees = 0.0;
    double maximum_j6_step_degrees = 0.0;
    int maximum_step_from_source_index = -1;
    int maximum_step_to_source_index = -1;
};

// 对可达候选做确定性的覆盖压缩、J6 连续化和最近关节角排序。
// 失败时 candidates 保持不变。
bool planPoseCandidates(std::vector<PoseCandidate>& candidates,
                        const double initial_joint[6],
                        const CandidatePlanningOptions& options,
                        CandidatePlanningReport& report,
                        std::string& error);

double candidateMaximumJointStep(const double from[6], const double to[6]);
double candidateJointPathDistance(const double from[6], const double to[6]);

#endif // CANDIDATE_PLANNER_HPP
