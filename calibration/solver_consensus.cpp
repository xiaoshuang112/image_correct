#include "solver_consensus.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace solver_consensus {
namespace {

double normalized(const PairDistance& distance,
                  double translation_limit,
                  double rotation_limit) {
    return distance.translation / translation_limit +
           distance.rotation / rotation_limit;
}

double percentile95(std::vector<double> values) {
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(0.95 * static_cast<double>(values.size()))) - 1;
    return values[std::min(index, values.size() - 1)];
}

bool validMatrix(const std::vector<std::vector<PairDistance>>& distances) {
    const std::size_t size = distances.size();
    if (size == 0 || size >= sizeof(unsigned long long) * 8)
        return false;
    for (const auto& row : distances) {
        if (row.size() != size)
            return false;
        for (const PairDistance& distance : row) {
            if (!std::isfinite(distance.translation) ||
                !std::isfinite(distance.rotation) ||
                distance.translation < 0.0 || distance.rotation < 0.0) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

Selection selectCompleteLinkCluster(
    const std::vector<std::vector<PairDistance>>& distances,
    double translation_limit,
    double rotation_limit,
    std::size_t minimum_members) {
    Selection result;
    const std::size_t count = distances.size();
    if (!validMatrix(distances) || !std::isfinite(translation_limit) ||
        !std::isfinite(rotation_limit) || translation_limit <= 0.0 ||
        rotation_limit <= 0.0) {
        result.failure_reason = "invalid_solver_distance";
        return result;
    }

    for (std::size_t left = 0; left < count; ++left) {
        for (std::size_t right = left + 1; right < count; ++right) {
            result.global_max_translation = std::max(
                result.global_max_translation, distances[left][right].translation);
            result.global_max_rotation = std::max(
                result.global_max_rotation, distances[left][right].rotation);
        }
    }

    struct Candidate {
        std::vector<std::size_t> members;
        std::size_t medoid = 0;
        double medoid_sum = 0.0;
        double max_pair = 0.0;
        double pair_sum = 0.0;
    };
    std::vector<Candidate> candidates;
    std::size_t largest_size = 0;
    const unsigned long long end = 1ULL << count;
    for (unsigned long long mask = 1; mask < end; ++mask) {
        const std::size_t size = static_cast<std::size_t>(__builtin_popcountll(mask));
        if (size < minimum_members || size < largest_size)
            continue;
        Candidate candidate;
        for (std::size_t index = 0; index < count; ++index) {
            if ((mask & (1ULL << index)) != 0)
                candidate.members.push_back(index);
        }
        bool complete_link = true;
        std::vector<double> sums(count, 0.0);
        for (std::size_t left_pos = 0; left_pos < candidate.members.size(); ++left_pos) {
            for (std::size_t right_pos = left_pos + 1;
                 right_pos < candidate.members.size(); ++right_pos) {
                const std::size_t left = candidate.members[left_pos];
                const std::size_t right = candidate.members[right_pos];
                const PairDistance& distance = distances[left][right];
                if (distance.translation > translation_limit ||
                    distance.rotation > rotation_limit) {
                    complete_link = false;
                    break;
                }
                const double value = normalized(
                    distance, translation_limit, rotation_limit);
                candidate.max_pair = std::max(candidate.max_pair, value);
                candidate.pair_sum += value;
                sums[left] += value;
                sums[right] += value;
            }
            if (!complete_link)
                break;
        }
        if (!complete_link)
            continue;
        candidate.medoid = candidate.members.front();
        candidate.medoid_sum = sums[candidate.medoid];
        for (const std::size_t index : candidate.members) {
            if (sums[index] < candidate.medoid_sum) {
                candidate.medoid = index;
                candidate.medoid_sum = sums[index];
            }
        }
        if (size > largest_size) {
            candidates.clear();
            largest_size = size;
        }
        candidates.push_back(std::move(candidate));
    }

    if (candidates.empty()) {
        result.failure_reason = count < minimum_members
            ? "insufficient_successful_solvers"
            : "no_majority_solver_cluster";
        return result;
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& left,
                                                       const Candidate& right) {
        if (left.max_pair != right.max_pair)
            return left.max_pair < right.max_pair;
        if (left.pair_sum != right.pair_sum)
            return left.pair_sum < right.pair_sum;
        return left.members < right.members;
    });
    for (const Candidate& candidate : candidates)
        result.largest_clusters.push_back(candidate.members);

    const Candidate& selected = candidates.front();
    result.members = selected.members;
    result.medoid_index = selected.medoid;
    result.max_normalized_pair_distance = selected.max_pair;
    result.normalized_pair_distance_sum = selected.pair_sum;
    for (std::size_t index = 0; index < count; ++index) {
        if (std::find(result.members.begin(), result.members.end(), index) ==
            result.members.end()) {
            result.excluded.push_back(index);
        }
    }

    for (std::size_t index = 1; index < candidates.size(); ++index) {
        const PairDistance& medoid_distance =
            distances[selected.medoid][candidates[index].medoid];
        if (medoid_distance.translation > translation_limit ||
            medoid_distance.rotation > rotation_limit) {
            result.ambiguous = true;
            result.failure_reason = "ambiguous_solver_clusters";
            break;
        }
    }

    std::vector<double> translations;
    std::vector<double> rotations;
    for (std::size_t left_pos = 0; left_pos < result.members.size(); ++left_pos) {
        for (std::size_t right_pos = left_pos + 1;
             right_pos < result.members.size(); ++right_pos) {
            const PairDistance& distance = distances[result.members[left_pos]]
                                                    [result.members[right_pos]];
            translations.push_back(distance.translation);
            rotations.push_back(distance.rotation);
            result.cluster_max_translation = std::max(
                result.cluster_max_translation, distance.translation);
            result.cluster_max_rotation = std::max(
                result.cluster_max_rotation, distance.rotation);
        }
    }
    result.cluster_p95_translation = percentile95(std::move(translations));
    result.cluster_p95_rotation = percentile95(std::move(rotations));
    result.passed = result.failure_reason.empty();
    return result;
}

}  // namespace solver_consensus
