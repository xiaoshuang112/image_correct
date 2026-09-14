#include "test_check.hpp"

#include "calibration/solver_consensus.hpp"

#include <vector>

namespace suite {
namespace {

using solver_consensus::PairDistance;
using solver_consensus::Selection;

std::vector<std::vector<PairDistance>> matrix(std::size_t size) {
    return std::vector<std::vector<PairDistance>>(
        size, std::vector<PairDistance>(size));
}

void setPair(std::vector<std::vector<PairDistance>>& values,
             std::size_t left, std::size_t right,
             double translation, double rotation) {
    values[left][right] = {translation, rotation};
    values[right][left] = values[left][right];
}

}  // namespace

int test_solver_consensus() {
    std::cout << "---- solver_consensus ----\n";
    {
        auto values = matrix(5);
        for (std::size_t left = 0; left < 5; ++left)
            for (std::size_t right = left + 1; right < 5; ++right)
                setPair(values, left, right, 0.002, 0.2);
        const Selection selected = solver_consensus::selectCompleteLinkCluster(
            values, 0.010, 1.0);
        TEST_CHECK(selected.passed && selected.members.size() == 5,
                   "consistent data keeps all solvers");
        TEST_CHECK(selected.medoid_index == 0,
                   "consistent tie is deterministic");
    }
    {
        auto values = matrix(5);
        for (std::size_t left = 0; left < 5; ++left)
            for (std::size_t right = left + 1; right < 5; ++right)
                setPair(values, left, right, 0.002, 0.2);
        for (std::size_t index = 0; index < 4; ++index)
            setPair(values, index, 4, 0.020, 0.2);
        const Selection selected = solver_consensus::selectCompleteLinkCluster(
            values, 0.010, 1.0);
        TEST_CHECK(selected.passed && selected.members.size() == 4 &&
                       selected.excluded.size() == 1 &&
                       selected.excluded.front() == 4,
                   "single solver outlier cannot veto majority");
        TEST_CHECK(selected.global_max_translation == 0.020 &&
                       selected.cluster_max_translation == 0.002,
                   "global and in-cluster maxima stay auditable");
    }
    {
        auto values = matrix(5);
        for (std::size_t left = 0; left < 5; ++left)
            for (std::size_t right = left + 1; right < 5; ++right)
                setPair(values, left, right, 0.020, 1.5);
        setPair(values, 0, 1, 0.002, 0.2);
        setPair(values, 0, 2, 0.003, 0.3);
        setPair(values, 1, 2, 0.002, 0.2);
        setPair(values, 3, 4, 0.002, 0.2);
        const Selection selected = solver_consensus::selectCompleteLinkCluster(
            values, 0.010, 1.0);
        TEST_CHECK(selected.passed && selected.members.size() == 3 &&
                       selected.excluded.size() == 2,
                   "two outlying solvers cannot veto three-solver majority");
    }
    {
        // 0--1 and 1--2 pass, but 0--2 fails: a connected-component
        // implementation would incorrectly accept this chain.
        auto values = matrix(3);
        setPair(values, 0, 1, 0.009, 0.2);
        setPair(values, 1, 2, 0.009, 0.2);
        setPair(values, 0, 2, 0.018, 0.2);
        const Selection selected = solver_consensus::selectCompleteLinkCluster(
            values, 0.010, 1.0);
        TEST_CHECK(!selected.passed &&
                       selected.failure_reason == "no_majority_solver_cluster",
                   "complete-link rejects chained cluster");
    }
    {
        // Two maximum triangles share solver 2 but have medoids on opposite
        // sides of a failed edge; publication must be refused as ambiguous.
        auto values = matrix(5);
        for (std::size_t left = 0; left < 5; ++left)
            for (std::size_t right = left + 1; right < 5; ++right)
                setPair(values, left, right, 0.020, 1.5);
        setPair(values, 0, 1, 0.001, 0.1);
        setPair(values, 0, 2, 0.002, 0.1);
        setPair(values, 1, 2, 0.002, 0.1);
        setPair(values, 2, 3, 0.009, 0.9);
        setPair(values, 2, 4, 0.009, 0.9);
        setPair(values, 3, 4, 0.001, 0.1);
        const Selection selected = solver_consensus::selectCompleteLinkCluster(
            values, 0.010, 1.0);
        TEST_CHECK(!selected.passed && selected.ambiguous &&
                       selected.failure_reason == "ambiguous_solver_clusters",
                   "split maximum clusters are rejected");
    }
    {
        auto values = matrix(3);
        setPair(values, 0, 1, 0.002, 0.49);
        setPair(values, 0, 2, 0.002, 0.51);
        setPair(values, 1, 2, 0.002, 0.51);
        const Selection production = solver_consensus::selectCompleteLinkCluster(
            values, 0.010, 1.0);
        const Selection tight = solver_consensus::selectCompleteLinkCluster(
            values, 0.003, 0.5);
        TEST_CHECK(production.passed && !tight.passed,
                   "near-half-degree jitter is diagnostic, not production gate");
    }
    return testutil::takeFailures();
}

}  // namespace suite
