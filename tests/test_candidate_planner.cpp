#include "test_check.hpp"

#include "calibration/candidate_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace suite {
namespace {

PoseCandidate candidate(int source, double joint0, double joint5,
                        double feature0, double feature3 = 0.0)
{
    PoseCandidate value{};
    value.source_index = source;
    value.joint[0] = joint0;
    value.joint[5] = joint5;
    value.flange_euler[0] = feature0;
    value.flange_euler[3] = feature3;
    return value;
}

} // namespace

int test_candidate_planner()
{
    {
        std::vector<PoseCandidate> candidates{
            candidate(30, 30.0, 0.0, 30.0),
            candidate(10, 10.0, 0.0, 10.0),
            candidate(20, 20.0, 0.0, 20.0)};
        const double initial[6]{};
        CandidatePlanningOptions options;
        options.maximum_joint_step_degrees = 15.0;
        CandidatePlanningReport report;
        std::string error;
        const bool ok = planPoseCandidates(candidates, initial, options,
                                           report, error);
        TEST_CHECK(ok, "candidate planner orders reachable short steps");
        TEST_CHECK(candidates.size() == 3 &&
                   candidates[0].source_index == 10 &&
                   candidates[1].source_index == 20 &&
                   candidates[2].source_index == 30,
                   "candidate planner uses nearest joint order");
        TEST_CHECK(std::abs(report.maximum_joint_step_degrees - 10.0) < 1.0e-9,
                   "candidate planner reports maximum step");
    }

    {
        std::vector<PoseCandidate> candidates{
            candidate(1, 1.0, 350.0, 1.0)};
        const double initial[6]{0.0, 0.0, 0.0, 0.0, 0.0, -5.0};
        CandidatePlanningOptions options;
        options.maximum_joint_step_degrees = 10.0;
        CandidatePlanningReport report;
        std::string error;
        const bool ok = planPoseCandidates(candidates, initial, options,
                                           report, error);
        TEST_CHECK(ok, "candidate planner accepts equivalent J6 angle");
        TEST_CHECK(std::abs(candidates[0].joint[5] + 10.0) < 1.0e-9,
                   "candidate planner keeps J6 continuous");
    }

    {
        std::vector<PoseCandidate> candidates;
        for (int index = 0; index < 120; ++index)
            candidates.push_back(candidate(index, index * 0.1, 0.0,
                                           static_cast<double>(index),
                                           static_cast<double>(index % 7)));
        const double initial[6]{};
        CandidatePlanningOptions options;
        options.maximum_candidates = 90;
        CandidatePlanningReport report;
        std::string error;
        const bool ok = planPoseCandidates(candidates, initial, options,
                                           report, error);
        TEST_CHECK(ok && candidates.size() == 90,
                   "candidate planner compresses to configured limit");
        TEST_CHECK(report.input_candidates == 120 &&
                   report.selected_candidates == 90,
                   "candidate planner reports compression counts");
    }

    {
        std::vector<PoseCandidate> candidates{
            candidate(1, 80.0, 0.0, 80.0)};
        const double initial[6]{};
        CandidatePlanningOptions options;
        options.maximum_joint_step_degrees = 45.0;
        CandidatePlanningReport report;
        std::string error;
        TEST_CHECK(!planPoseCandidates(candidates, initial, options,
                                       report, error) && !error.empty(),
                   "candidate planner rejects unsafe isolated step");
    }

    return testutil::takeFailures();
}

} // namespace suite
