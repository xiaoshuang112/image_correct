#include "candidate_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>

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

std::array<double, kAxisCount> jointSteps(const double from[6],
                                          const double to[6])
{
    std::array<double, kAxisCount> steps{};
    for (int axis = 0; axis < kAxisCount; ++axis)
        steps[axis] = std::abs(continuousDelta(from[axis], to[axis], axis));
    return steps;
}

std::string formatJoints(const double values[6])
{
    std::ostringstream stream;
    stream << "[";
    for (int axis = 0; axis < kAxisCount; ++axis) {
        if (axis > 0)
            stream << ",";
        stream << values[axis];
    }
    stream << "]";
    return stream.str();
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

using Graph = std::vector<std::vector<std::pair<std::size_t, double>>>;

void addUndirectedEdge(Graph& graph, std::size_t left, std::size_t right,
                       double cost)
{
    graph[left].push_back({right, cost});
    graph[right].push_back({left, cost});
}

bool stepAllowed(const double from[6], const double to[6], double limit)
{
    return limit <= 0.0 || candidateMaximumJointStep(from, to) <= limit;
}

Graph buildSafeGraph(const std::vector<PoseCandidate>& candidates,
                     const double initial_joint[6], double limit)
{
    const std::size_t start = candidates.size();
    Graph graph(candidates.size() + 1);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (stepAllowed(initial_joint, candidates[index].joint, limit)) {
            addUndirectedEdge(graph, start, index,
                candidateJointPathDistance(initial_joint,
                                           candidates[index].joint));
        }
        for (std::size_t other = 0; other < index; ++other) {
            if (!stepAllowed(candidates[other].joint,
                             candidates[index].joint, limit)) {
                continue;
            }
            addUndirectedEdge(graph, other, index,
                candidateJointPathDistance(candidates[other].joint,
                                           candidates[index].joint));
        }
    }
    return graph;
}

std::vector<bool> connectedFromStart(const Graph& graph, std::size_t start)
{
    std::vector<bool> connected(graph.size(), false);
    std::queue<std::size_t> pending;
    connected[start] = true;
    pending.push(start);
    while (!pending.empty()) {
        const std::size_t node = pending.front();
        pending.pop();
        for (const auto& edge : graph[node]) {
            if (connected[edge.first])
                continue;
            connected[edge.first] = true;
            pending.push(edge.first);
        }
    }
    return connected;
}

bool shortestPathToAnyTarget(const Graph& graph, std::size_t start,
                             const std::vector<bool>& target,
                             const std::vector<PoseCandidate>& candidates,
                             std::vector<std::size_t>& path)
{
    const std::size_t count = graph.size();
    std::vector<double> distance(count, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(count, count);
    using QueueEntry = std::pair<double, std::size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        std::greater<QueueEntry>> pending;
    distance[start] = 0.0;
    pending.push({0.0, start});

    while (!pending.empty()) {
        const QueueEntry entry = pending.top();
        pending.pop();
        if (entry.first > distance[entry.second] + kTieTolerance)
            continue;
        for (const auto& edge : graph[entry.second]) {
            const double nextDistance = entry.first + edge.second;
            if (nextDistance < distance[edge.first] - kTieTolerance) {
                distance[edge.first] = nextDistance;
                previous[edge.first] = entry.second;
                pending.push({nextDistance, edge.first});
            }
        }
    }

    std::size_t best = count;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (!target[index] || !std::isfinite(distance[index]))
            continue;
        if (best == count || distance[index] < distance[best] - kTieTolerance ||
            (std::abs(distance[index] - distance[best]) <= kTieTolerance &&
             candidates[index].source_index < candidates[best].source_index)) {
            best = index;
        }
    }
    if (best == count)
        return false;

    path.clear();
    for (std::size_t node = best; node != start; node = previous[node]) {
        if (node == count || previous[node] == count)
            return false;
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    return true;
}

bool resolveForStep(PoseCandidate& candidate,
                    const std::array<double, kAxisCount>& reference,
                    const CandidatePlanningOptions& options,
                    int& resolver_error)
{
    resolver_error = 0;
    if (options.joint_resolver) {
        std::array<double, kAxisCount> resolved{};
        if (!options.joint_resolver(candidate, reference, resolved,
                                    resolver_error)) {
            return false;
        }
        std::copy(resolved.begin(), resolved.end(), candidate.joint);
    }
    makeJ6Continuous(candidate, reference.data());
    return true;
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

    const std::size_t start = candidates.size();
    const Graph graph = buildSafeGraph(candidates, initial_joint,
                                       options.maximum_joint_step_degrees);
    const std::vector<bool> connected = connectedFromStart(graph, start);
    std::vector<PoseCandidate> connectedCandidates;
    connectedCandidates.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (connected[index])
            connectedCandidates.push_back(candidates[index]);
    }
    report.connected_candidates = connectedCandidates.size();
    report.disconnected_candidates =
        candidates.size() - connectedCandidates.size();
    if (connectedCandidates.empty()) {
        error = "no candidate is safely connected to the initial joint, limit=" +
                std::to_string(options.maximum_joint_step_degrees) + " deg";
        return false;
    }

    std::vector<PoseCandidate> selected = selectDiverseCandidates(
        connectedCandidates, options.maximum_candidates, initial_joint);
    report.selected_candidates = selected.size();

    std::set<int> targetSources;
    for (const PoseCandidate& candidate : selected)
        targetSources.insert(candidate.source_index);
    std::vector<bool> remainingTarget(candidates.size(), false);
    for (std::size_t index = 0; index < candidates.size(); ++index)
        remainingTarget[index] = targetSources.count(candidates[index].source_index) != 0;

    std::vector<PoseCandidate> ordered;
    ordered.reserve(selected.size());
    std::array<double, kAxisCount> current{};
    std::copy(initial_joint, initial_joint + kAxisCount, current.begin());
    std::size_t currentNode = start;
    int previousSource = -1;
    std::size_t targetsRemaining = selected.size();

    while (targetsRemaining > 0) {
        std::vector<std::size_t> path;
        if (!shortestPathToAnyTarget(graph, currentNode, remainingTarget,
                                     candidates, path)) {
            error = "safe graph cannot reach remaining capture candidates";
            return false;
        }
        for (const std::size_t node : path) {
            PoseCandidate candidate{};
            int resolverError = 0;
            if (node == start) {
                std::copy(initial_joint, initial_joint + kAxisCount,
                          candidate.joint);
                candidate.source_index = -1;
                candidate.capture = false;
                makeJ6Continuous(candidate, current.data());
            } else {
                candidate = candidates[node];
            }
            if (node != start &&
                !resolveForStep(candidate, current, options, resolverError)) {
                error = "continuous IK failed: source " +
                        std::to_string(previousSource) + " -> " +
                        std::to_string(candidate.source_index) +
                        ", sdk_error=" + std::to_string(resolverError);
                return false;
            }

            const std::array<double, kAxisCount> steps = jointSteps(
                current.data(), candidate.joint);
            const auto maximumIt = std::max_element(steps.begin(), steps.end());
            const int maximumAxis = static_cast<int>(
                std::distance(steps.begin(), maximumIt));
            const double maximumStep = *maximumIt;
            const double pathDistance = candidateJointPathDistance(
                current.data(), candidate.joint);
            const double j6Step = steps[5];
            for (int axis = 0; axis < kAxisCount; ++axis) {
                report.maximum_axis_step_degrees[axis] = std::max(
                    report.maximum_axis_step_degrees[axis], steps[axis]);
            }
            if (maximumStep > report.maximum_joint_step_degrees) {
                report.maximum_joint_step_degrees = maximumStep;
                report.maximum_step_axis = maximumAxis;
                report.maximum_step_from_source_index = previousSource;
                report.maximum_step_to_source_index = candidate.source_index;
            }
            report.maximum_j6_step_degrees = std::max(
                report.maximum_j6_step_degrees, j6Step);
            report.estimated_path_degrees += pathDistance;

            if (options.maximum_joint_step_degrees > 0.0 &&
                maximumStep > options.maximum_joint_step_degrees) {
                error = "joint step gate exceeded: source " +
                        std::to_string(previousSource) + " -> " +
                        std::to_string(candidate.source_index) + ", step=" +
                        std::to_string(maximumStep) + " deg, limit=" +
                        std::to_string(options.maximum_joint_step_degrees) +
                        " deg, max_axis=J" + std::to_string(maximumAxis + 1) +
                        ", axis_steps=" + formatJoints(steps.data()) +
                        ", from_joint=" + formatJoints(current.data()) +
                        ", to_joint=" + formatJoints(candidate.joint);
                return false;
            }

            candidate.capture = node != start && remainingTarget[node];
            std::copy(steps.begin(), steps.end(),
                      candidate.step_from_previous);
            candidate.previous_source_index = previousSource;
            if (candidate.capture) {
                remainingTarget[node] = false;
                --targetsRemaining;
            } else {
                ++report.transition_candidates;
            }
            std::copy(candidate.joint, candidate.joint + kAxisCount,
                      current.begin());
            previousSource = candidate.source_index;
            currentNode = node;
            ordered.push_back(candidate);
        }
    }

    report.ordered_candidates = ordered.size();
    candidates.swap(ordered);
    return true;
}
