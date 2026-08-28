#include "candidate_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace {

constexpr int kAxisCount = 6;
constexpr double kTieTolerance = 1.0e-12;

double continuousDelta(double from, double to, int axis)
{
    const double delta = to - from;
    return axis == 5 ? std::remainder(delta, 360.0) : delta;
}

void makeJ6Continuous(PoseCandidate& candidate, const double reference[6])
{
    candidate.joint[5] +=
        std::round((reference[5] - candidate.joint[5]) / 360.0) * 360.0;
}

std::vector<PoseCandidate> selectDiverseCandidates(
    const std::vector<PoseCandidate>& input,
    std::size_t maximum,
    const double initial_joint[6])
{
    if (maximum == 0 || input.size() <= maximum)
        return input;

    std::array<double, kAxisCount> minimum{};
    std::array<double, kAxisCount> maximumValue{};
    minimum.fill(std::numeric_limits<double>::infinity());
    maximumValue.fill(-std::numeric_limits<double>::infinity());
    for (const PoseCandidate& candidate : input) {
        for (int axis = 0; axis < kAxisCount; ++axis) {
            minimum[axis] = std::min(minimum[axis], candidate.flange_euler[axis]);
            maximumValue[axis] = std::max(maximumValue[axis], candidate.flange_euler[axis]);
        }
    }

    auto featureDistanceSquared = [&](std::size_t left, std::size_t right) {
        double sum = 0.0;
        for (int axis = 0; axis < kAxisCount; ++axis) {
            const double range = maximumValue[axis] - minimum[axis];
            if (range <= kTieTolerance)
                continue;
            double delta = input[left].flange_euler[axis] -
                           input[right].flange_euler[axis];
            if (axis >= 3)
                delta = std::remainder(delta, 360.0);
            const double normalized = delta / range;
            sum += normalized * normalized;
        }
        return sum;
    };

    // 从离当前机器人最近的候选开始，随后使用最远点采样保持六维覆盖。
    std::size_t first = 0;
    double firstDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < input.size(); ++index) {
        const double distance = candidateJointPathDistance(initial_joint,
                                                           input[index].joint);
        if (distance < firstDistance - kTieTolerance) {
            firstDistance = distance;
            first = index;
        }
    }

    std::vector<bool> selected(input.size(), false);
    std::vector<double> nearestDistance(input.size(),
                                        std::numeric_limits<double>::infinity());
    std::vector<std::size_t> selection;
    selection.reserve(maximum);
    selection.push_back(first);
    selected[first] = true;

    while (selection.size() < maximum) {
        const std::size_t newest = selection.back();
        std::size_t best = input.size();
        double bestDistance = -1.0;
        for (std::size_t index = 0; index < input.size(); ++index) {
            if (selected[index])
                continue;
            nearestDistance[index] = std::min(
                nearestDistance[index], featureDistanceSquared(index, newest));
            if (nearestDistance[index] > bestDistance + kTieTolerance ||
                (std::abs(nearestDistance[index] - bestDistance) <= kTieTolerance &&
                 (best == input.size() ||
                  input[index].source_index < input[best].source_index))) {
                bestDistance = nearestDistance[index];
                best = index;
            }
        }
        if (best == input.size())
            break;
        selected[best] = true;
        selection.push_back(best);
    }

    std::vector<PoseCandidate> output;
    output.reserve(selection.size());
    for (const std::size_t index : selection)
        output.push_back(input[index]);
    return output;
}

} // namespace

double candidateMaximumJointStep(const double from[6], const double to[6])
{
    double maximum = 0.0;
    for (int axis = 0; axis < kAxisCount; ++axis)
        maximum = std::max(maximum, std::abs(continuousDelta(from[axis], to[axis], axis)));
    return maximum;
}

double candidateJointPathDistance(const double from[6], const double to[6])
{
    double sum = 0.0;
    for (int axis = 0; axis < kAxisCount; ++axis) {
        const double delta = continuousDelta(from[axis], to[axis], axis);
        sum += delta * delta;
    }
    return std::sqrt(sum);
}

bool planPoseCandidates(std::vector<PoseCandidate>& candidates,
                        const double initial_joint[6],
                        const CandidatePlanningOptions& options,
                        CandidatePlanningReport& report,
                        std::string& error)
{
    report = CandidatePlanningReport{};
    report.input_candidates = candidates.size();
    error.clear();
    if (!initial_joint) {
        error = "initial joint is null";
        return false;
    }
    if (candidates.empty()) {
        error = "no reachable candidates";
        return false;
    }
    if (!std::isfinite(options.maximum_joint_step_degrees)) {
        error = "maximum joint step is not finite";
        return false;
    }

    std::vector<PoseCandidate> selected = selectDiverseCandidates(
        candidates, options.maximum_candidates, initial_joint);
    report.selected_candidates = selected.size();

    std::vector<PoseCandidate> ordered;
    ordered.reserve(selected.size());
    std::vector<bool> used(selected.size(), false);
    double current[kAxisCount];
    std::copy(initial_joint, initial_joint + kAxisCount, current);
    int previousSource = -1;

    for (std::size_t count = 0; count < selected.size(); ++count) {
        std::size_t best = selected.size();
        double bestDistance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < selected.size(); ++index) {
            if (used[index])
                continue;
            PoseCandidate continuous = selected[index];
            makeJ6Continuous(continuous, current);
            const double distance = candidateJointPathDistance(current,
                                                               continuous.joint);
            if (distance < bestDistance - kTieTolerance ||
                (std::abs(distance - bestDistance) <= kTieTolerance &&
                 (best == selected.size() ||
                  continuous.source_index < selected[best].source_index))) {
                bestDistance = distance;
                best = index;
            }
        }
        if (best == selected.size()) {
            error = "candidate ordering stopped unexpectedly";
            return false;
        }

        PoseCandidate candidate = selected[best];
        makeJ6Continuous(candidate, current);
        const double maximumStep = candidateMaximumJointStep(current,
                                                             candidate.joint);
        const double j6Step = std::abs(candidate.joint[5] - current[5]);
        if (maximumStep > report.maximum_joint_step_degrees) {
            report.maximum_joint_step_degrees = maximumStep;
            report.maximum_step_from_source_index = previousSource;
            report.maximum_step_to_source_index = candidate.source_index;
        }
        report.maximum_j6_step_degrees = std::max(report.maximum_j6_step_degrees,
                                                  j6Step);
        report.estimated_path_degrees += bestDistance;

        if (options.maximum_joint_step_degrees > 0.0 &&
            maximumStep > options.maximum_joint_step_degrees) {
            error = "joint step gate exceeded: source " +
                    std::to_string(previousSource) + " -> " +
                    std::to_string(candidate.source_index) + ", step=" +
                    std::to_string(maximumStep) + " deg, limit=" +
                    std::to_string(options.maximum_joint_step_degrees) + " deg";
            return false;
        }

        used[best] = true;
        std::copy(candidate.joint, candidate.joint + kAxisCount, current);
        previousSource = candidate.source_index;
        ordered.push_back(candidate);
    }

    report.ordered_candidates = ordered.size();
    candidates.swap(ordered);
    return true;
}
