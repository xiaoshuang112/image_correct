#ifndef SOLVER_CONSENSUS_HPP
#define SOLVER_CONSENSUS_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace solver_consensus {

struct PairDistance {
    double translation = 0.0;
    double rotation = 0.0;
};

struct Selection {
    bool passed = false;
    bool ambiguous = false;
    std::string failure_reason;
    std::vector<std::size_t> members;
    std::vector<std::size_t> excluded;
    std::vector<std::vector<std::size_t>> largest_clusters;
    std::size_t medoid_index = 0;
    double medoid_distance_sum = 0.0;
    double cluster_max_translation = 0.0;
    double cluster_p95_translation = 0.0;
    double cluster_max_rotation = 0.0;
    double cluster_p95_rotation = 0.0;
    double global_max_translation = 0.0;
    double global_max_rotation = 0.0;
    double max_normalized_pair_distance = 0.0;
    double normalized_pair_distance_sum = 0.0;
};

// Selects a maximum-cardinality complete-link cluster. Every pair in the
// returned cluster satisfies both limits. Ties are deterministic and a tie
// whose medoids are outside either production limit is rejected as ambiguous.
Selection selectCompleteLinkCluster(
    const std::vector<std::vector<PairDistance>>& distances,
    double translation_limit,
    double rotation_limit,
    std::size_t minimum_members = 3);

}  // namespace solver_consensus

#endif
