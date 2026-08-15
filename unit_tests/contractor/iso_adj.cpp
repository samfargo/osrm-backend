#include "contractor/iso_adj.hpp"
#include "contractor/phast_runtime.hpp"

#include "../common/temporary_file.hpp"

#include <boost/test/unit_test.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

BOOST_AUTO_TEST_SUITE(iso_adj)

using namespace osrm;
using namespace osrm::contractor;
using namespace osrm::contractor::phast;

namespace
{
IsoAdjMetadata makeMetadata()
{
    IsoAdjMetadata metadata;
    metadata.phast_schema_version = PHAST_SCHEMA_VERSION;
    metadata.metric_kind = MetricKind::Duration;
    metadata.orientation = PHASTOrientation::Reverse;
    metadata.exclude_index = 2;
    metadata.connectivity_checksum = 0xDEADBEEF;
    metadata.node_count = 3;
    metadata.edge_count = 4;
    metadata.candidate_count = 4;
    metadata.metric_name = "duration";
    metadata.resolution = 9;
    metadata.artifact_version = "test-v1";
    metadata.ordinal_count = 4;
    metadata.ordinals_file_size = 32;
    metadata.ordinals_hash_hi = 11;
    metadata.ordinals_hash_lo = 22;
    metadata.build_id = BuildIsoAdjBuildID("2026-08-11T00:00:00Z", metadata);
    return metadata;
}
} // namespace

BOOST_AUTO_TEST_CASE(round_trips_versioned_metadata_and_adjacency)
{
    const auto metadata = makeMetadata();
    const std::vector<std::uint64_t> offsets = {0, 2, 3, 4};
    const std::vector<std::uint32_t> targets = {1, 2, 0, 1};
    const std::vector<std::uint32_t> costs = {10, 20, 30, 40};
    const std::vector<std::uint64_t> cell_offsets = {0, 1, 2, 4};
    const std::vector<std::uint32_t> candidate_ordinals = {0, 1, 2, 3};
    const std::vector<std::int32_t> candidate_costs = {1, 2, 3, 4};
    const std::vector<std::uint32_t> original_to_spatial = {2, 0, 1};
    TemporaryFile output;

    BOOST_REQUIRE(WriteIsoAdj(output.path,
                              metadata,
                              offsets,
                              targets,
                              costs,
                              cell_offsets,
                              candidate_ordinals,
                              candidate_costs,
                              original_to_spatial));
    BOOST_CHECK(!std::filesystem::exists(output.path.string() + ".tmp"));

    IsoAdj loaded;
    BOOST_REQUIRE(LoadIsoAdj(output.path, loaded));
    BOOST_CHECK_EQUAL(loaded.metadata.phast_schema_version, metadata.phast_schema_version);
    BOOST_CHECK(loaded.metadata.metric_kind == metadata.metric_kind);
    BOOST_CHECK(loaded.metadata.orientation == metadata.orientation);
    BOOST_CHECK_EQUAL(loaded.metadata.exclude_index, metadata.exclude_index);
    BOOST_CHECK_EQUAL(loaded.metadata.connectivity_checksum, metadata.connectivity_checksum);
    BOOST_CHECK_EQUAL(loaded.metadata.node_count, metadata.node_count);
    BOOST_CHECK_EQUAL(loaded.metadata.edge_count, metadata.edge_count);
    BOOST_CHECK_EQUAL(loaded.metadata.candidate_count, metadata.candidate_count);
    BOOST_CHECK_EQUAL(loaded.metadata.metric_name, metadata.metric_name);
    BOOST_CHECK(loaded.metadata.build_id == metadata.build_id);
    BOOST_CHECK_EQUAL(loaded.metadata.resolution, metadata.resolution);
    BOOST_CHECK_EQUAL(loaded.metadata.artifact_version, metadata.artifact_version);
    BOOST_CHECK_EQUAL_COLLECTIONS(
        loaded.offsets, loaded.offsets + offsets.size(), offsets.begin(), offsets.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(
        loaded.targets, loaded.targets + targets.size(), targets.begin(), targets.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(
        loaded.costs, loaded.costs + costs.size(), costs.begin(), costs.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(loaded.cell_offsets,
                                  loaded.cell_offsets + cell_offsets.size(),
                                  cell_offsets.begin(),
                                  cell_offsets.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(loaded.candidate_ordinals,
                                  loaded.candidate_ordinals + candidate_ordinals.size(),
                                  candidate_ordinals.begin(),
                                  candidate_ordinals.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(loaded.original_to_spatial,
                                  loaded.original_to_spatial + original_to_spatial.size(),
                                  original_to_spatial.begin(),
                                  original_to_spatial.end());
}

BOOST_AUTO_TEST_CASE(build_identity_binds_dataset_and_compatibility_metadata)
{
    auto metadata = makeMetadata();
    const auto expected = metadata.build_id;
    BOOST_CHECK(BuildIsoAdjBuildID("different-dataset", metadata) != expected);

    ++metadata.exclude_index;
    BOOST_CHECK(BuildIsoAdjBuildID("2026-08-11T00:00:00Z", metadata) != expected);

    metadata = makeMetadata();
    ++metadata.node_order;
    BOOST_CHECK(BuildIsoAdjBuildID("2026-08-11T00:00:00Z", metadata) != expected);
}

BOOST_AUTO_TEST_CASE(rejects_old_or_truncated_artifacts)
{
    const auto metadata = makeMetadata();
    const std::vector<std::uint64_t> offsets = {0, 2, 3, 4};
    const std::vector<std::uint32_t> targets = {1, 2, 0, 1};
    const std::vector<std::uint32_t> costs = {10, 20, 30, 40};
    const std::vector<std::uint64_t> cell_offsets = {0, 1, 2, 4};
    const std::vector<std::uint32_t> candidate_ordinals = {0, 1, 2, 3};
    const std::vector<std::int32_t> candidate_costs = {1, 2, 3, 4};
    const std::vector<std::uint32_t> original_to_spatial = {2, 0, 1};
    TemporaryFile output;
    BOOST_REQUIRE(WriteIsoAdj(output.path,
                              metadata,
                              offsets,
                              targets,
                              costs,
                              cell_offsets,
                              candidate_ordinals,
                              candidate_costs,
                              original_to_spatial));

    {
        std::fstream file(output.path, std::ios::binary | std::ios::in | std::ios::out);
        const std::uint32_t old_version = ISO_ADJ_VERSION - 1;
        file.seekp(static_cast<std::streamoff>(ISO_ADJ_MAGIC.size()));
        file.write(reinterpret_cast<const char *>(&old_version), sizeof(old_version));
    }
    IsoAdj old;
    BOOST_CHECK(!LoadIsoAdj(output.path, old));

    BOOST_REQUIRE(WriteIsoAdj(output.path,
                              metadata,
                              offsets,
                              targets,
                              costs,
                              cell_offsets,
                              candidate_ordinals,
                              candidate_costs,
                              original_to_spatial));
    std::filesystem::resize_file(output.path, std::filesystem::file_size(output.path) - 1);
    IsoAdj truncated;
    BOOST_CHECK(!LoadIsoAdj(output.path, truncated));
}

BOOST_AUTO_TEST_CASE(capped_dijkstra_matches_expected_shortest_paths)
{
    const std::vector<std::uint64_t> offsets = {0, 2, 3, 4};
    const std::vector<std::uint32_t> targets = {1, 2, 0, 1};
    const std::vector<std::uint32_t> costs = {10, 20, 30, 40};
    IsoAdj adjacency;
    adjacency.metadata.node_count = 3;
    adjacency.metadata.edge_count = costs.size();
    adjacency.offsets = offsets.data();
    adjacency.targets = targets.data();
    adjacency.costs = costs.data();
    const std::vector<PHASTSeed> seeds = {
        {0, to_alias<EdgeWeight>(5), SeedClass::Bidirectional},
    };

    std::vector<ReachedState> reached;
    BOOST_REQUIRE(RunCappedDijkstraIsoAdj(adjacency, seeds, std::nullopt, reached));
    BOOST_REQUIRE_EQUAL(reached.size(), std::size_t{3});
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(reached[0].cost), 5);
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(reached[1].cost), 15);
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(reached[2].cost), 25);

    BOOST_REQUIRE(RunCappedDijkstraIsoAdj(
        adjacency, seeds, to_alias<EdgeWeight>(20), reached));
    BOOST_REQUIRE_EQUAL(reached.size(), std::size_t{2});
    BOOST_CHECK_EQUAL(reached[0].node, 0);
    BOOST_CHECK_EQUAL(reached[1].node, 1);
}

BOOST_AUTO_TEST_CASE(lean_search_matches_upward_and_downward_phast_semantics)
{
    const std::vector<std::uint64_t> lean_offsets = {0, 1, 1, 2};
    const std::vector<std::uint32_t> lean_targets = {2, 1};
    const std::vector<std::uint32_t> lean_costs = {10, 5};
    IsoAdj lean;
    lean.metadata.node_count = 3;
    lean.metadata.edge_count = lean_costs.size();
    lean.offsets = lean_offsets.data();
    lean.targets = lean_targets.data();
    lean.costs = lean_costs.data();

    DerivedAdjacency phast;
    phast.up_offsets = {0, 1, 1, 1};
    phast.up_targets = {2};
    phast.up_costs = {to_alias<EdgeWeight>(10)};
    phast.down_offsets = {0, 0, 0, 1};
    phast.down_targets = {1};
    phast.down_costs = {to_alias<EdgeWeight>(5)};
    PhastData ordering;
    ordering.node_count = 3;
    ordering.ordering.order = {0, 1, 2};
    ordering.ordering.rank = {0, 1, 2};
    const std::vector<PHASTSeed> seeds = {
        {0, to_alias<EdgeWeight>(0), SeedClass::Bidirectional},
    };

    for (const auto cap : {std::optional<EdgeWeight>{},
                           std::optional<EdgeWeight>{to_alias<EdgeWeight>(12)}})
    {
        std::vector<ReachedState> reached;
        BOOST_REQUIRE(RunCappedDijkstraIsoAdj(lean, seeds, cap, reached));
        std::vector<EdgeWeight> lean_distances(3, INVALID_EDGE_WEIGHT);
        for (const auto &state : reached)
        {
            lean_distances[state.node] = state.cost;
        }

        std::vector<EdgeWeight> phast_distances;
        std::size_t settled = 0;
        std::size_t updates = 0;
        BOOST_REQUIRE(RunUpwardSearch(phast, seeds, cap, phast_distances, settled));
        BOOST_REQUIRE(RunDownwardSweep(ordering, phast, cap, phast_distances, &updates));
        BOOST_CHECK_EQUAL_COLLECTIONS(lean_distances.begin(),
                                      lean_distances.end(),
                                      phast_distances.begin(),
                                      phast_distances.end());
    }
}

BOOST_AUTO_TEST_CASE(capped_dijkstra_rejects_invalid_touched_adjacency)
{
    const std::vector<std::uint64_t> offsets = {0, 1, 1};
    const std::vector<std::uint32_t> targets = {2};
    const std::vector<std::uint32_t> costs = {10};
    IsoAdj adjacency;
    adjacency.metadata.node_count = 2;
    adjacency.metadata.edge_count = 1;
    adjacency.offsets = offsets.data();
    adjacency.targets = targets.data();
    adjacency.costs = costs.data();
    const std::vector<PHASTSeed> seeds = {
        {0, to_alias<EdgeWeight>(0), SeedClass::Bidirectional},
    };

    std::vector<ReachedState> reached;
    BOOST_CHECK(!RunCappedDijkstraIsoAdj(adjacency, seeds, std::nullopt, reached));
}

BOOST_AUTO_TEST_CASE(sparse_reduction_matches_dense_minimum_rounding_and_cap_semantics)
{
    IsoAdj adjacency;
    adjacency.metadata = makeMetadata();
    adjacency.metadata.candidate_count = 5;
    const std::vector<std::uint64_t> cell_offsets = {0, 2, 4, 5};
    const std::vector<std::uint32_t> ordinals = {0, 1, 0, 2, 3};
    const std::vector<std::int32_t> offsets = {5, 15, 10, 200, -5};
    adjacency.cell_offsets = cell_offsets.data();
    adjacency.candidate_ordinals = ordinals.data();
    adjacency.candidate_costs = offsets.data();
    const std::vector<ReachedState> reached = {
        {0, to_alias<EdgeWeight>(100)},
        {1, to_alias<EdgeWeight>(90)},
        {2, to_alias<EdgeWeight>(150)},
    };

    std::vector<SparseResultRecord> records;
    BOOST_REQUIRE(ReduceReachedCells(
        adjacency, reached, to_alias<EdgeWeight>(180), records));
    BOOST_REQUIRE_EQUAL(records.size(), std::size_t{3});
    BOOST_CHECK_EQUAL(records[0].ordinal, 0);
    BOOST_CHECK_EQUAL(records[0].seconds, 10);
    BOOST_CHECK_EQUAL(records[1].ordinal, 1);
    BOOST_CHECK_EQUAL(records[1].seconds, 12);
    BOOST_CHECK_EQUAL(records[2].ordinal, 3);
    BOOST_CHECK_EQUAL(records[2].seconds, 14);
}

BOOST_AUTO_TEST_CASE(sparse_reduction_rejects_invalid_touched_candidate)
{
    IsoAdj adjacency;
    adjacency.metadata = makeMetadata();
    adjacency.metadata.candidate_count = 1;
    const std::vector<std::uint64_t> cell_offsets = {0, 1, 1, 1};
    const std::vector<std::uint32_t> ordinals = {4};
    const std::vector<std::int32_t> offsets = {0};
    adjacency.cell_offsets = cell_offsets.data();
    adjacency.candidate_ordinals = ordinals.data();
    adjacency.candidate_costs = offsets.data();
    const std::vector<ReachedState> reached = {
        {0, to_alias<EdgeWeight>(0)},
    };

    std::vector<SparseResultRecord> records;
    BOOST_CHECK(!ReduceReachedCells(adjacency, reached, std::nullopt, records));
}

BOOST_AUTO_TEST_CASE(sparse_result_publication_is_atomic_and_versioned)
{
    const auto metadata = makeMetadata();
    const std::vector<SparseResultRecord> records = {{1, 10, 0}, {3, 20, 0}};
    TemporaryFile output;

    BOOST_REQUIRE(WriteSparseResult(output.path, metadata, 60, records));
    BOOST_CHECK(!std::filesystem::exists(output.path.string() + ".tmp"));
    BOOST_CHECK_EQUAL(std::filesystem::file_size(output.path),
                      sizeof(SparseResultFileHeader) + records.size() * sizeof(SparseResultRecord));
    std::ifstream input(output.path, std::ios::binary);
    SparseResultFileHeader header;
    input.read(reinterpret_cast<char *>(&header), sizeof(header));
    BOOST_REQUIRE(input.good());
    BOOST_CHECK(header.magic == SPARSE_RESULT_MAGIC);
    BOOST_CHECK_EQUAL(header.format_version, SPARSE_RESULT_VERSION);
    BOOST_CHECK_EQUAL(header.resolution, 9);
    BOOST_CHECK_EQUAL(header.record_count, records.size());
    BOOST_CHECK_EQUAL(std::string(header.artifact_version.data(), metadata.artifact_version.size()),
                      metadata.artifact_version);
}

BOOST_AUTO_TEST_SUITE_END()
