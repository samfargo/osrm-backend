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
    metadata.metric_name = "duration";
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
    TemporaryFile output;

    BOOST_REQUIRE(WriteIsoAdj(output.path, metadata, offsets, targets, costs));
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
    BOOST_CHECK_EQUAL(loaded.metadata.metric_name, metadata.metric_name);
    BOOST_CHECK(loaded.metadata.build_id == metadata.build_id);
    BOOST_CHECK_EQUAL_COLLECTIONS(
        loaded.offsets, loaded.offsets + offsets.size(), offsets.begin(), offsets.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(
        loaded.targets, loaded.targets + targets.size(), targets.begin(), targets.end());
    BOOST_CHECK_EQUAL_COLLECTIONS(
        loaded.costs, loaded.costs + costs.size(), costs.begin(), costs.end());
}

BOOST_AUTO_TEST_CASE(build_identity_binds_dataset_and_compatibility_metadata)
{
    auto metadata = makeMetadata();
    const auto expected = metadata.build_id;
    BOOST_CHECK(BuildIsoAdjBuildID("different-dataset", metadata) != expected);

    ++metadata.exclude_index;
    BOOST_CHECK(BuildIsoAdjBuildID("2026-08-11T00:00:00Z", metadata) != expected);
}

BOOST_AUTO_TEST_CASE(rejects_old_or_truncated_artifacts)
{
    const auto metadata = makeMetadata();
    const std::vector<std::uint64_t> offsets = {0, 2, 3, 4};
    const std::vector<std::uint32_t> targets = {1, 2, 0, 1};
    const std::vector<std::uint32_t> costs = {10, 20, 30, 40};
    TemporaryFile output;
    BOOST_REQUIRE(WriteIsoAdj(output.path, metadata, offsets, targets, costs));

    {
        std::fstream file(output.path, std::ios::binary | std::ios::in | std::ios::out);
        const std::uint32_t old_version = ISO_ADJ_VERSION - 1;
        file.seekp(static_cast<std::streamoff>(ISO_ADJ_MAGIC.size()));
        file.write(reinterpret_cast<const char *>(&old_version), sizeof(old_version));
    }
    IsoAdj old;
    BOOST_CHECK(!LoadIsoAdj(output.path, old));

    BOOST_REQUIRE(WriteIsoAdj(output.path, metadata, offsets, targets, costs));
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
    adjacency.offsets = offsets.data();
    adjacency.targets = targets.data();
    adjacency.costs = costs.data();
    const std::vector<PHASTSeed> seeds = {
        {0, to_alias<EdgeWeight>(5), SeedClass::Bidirectional},
    };

    std::vector<EdgeWeight> distances;
    RunCappedDijkstraIsoAdj(adjacency, seeds, std::nullopt, distances);
    BOOST_REQUIRE_EQUAL(distances.size(), std::size_t{3});
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(distances[0]), 5);
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(distances[1]), 15);
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(distances[2]), 25);

    RunCappedDijkstraIsoAdj(adjacency, seeds, to_alias<EdgeWeight>(20), distances);
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(distances[0]), 5);
    BOOST_CHECK_EQUAL(from_alias<std::int64_t>(distances[1]), 15);
    BOOST_CHECK(distances[2] == INVALID_EDGE_WEIGHT);
}

BOOST_AUTO_TEST_CASE(lean_search_matches_upward_and_downward_phast_semantics)
{
    const std::vector<std::uint64_t> lean_offsets = {0, 1, 1, 2};
    const std::vector<std::uint32_t> lean_targets = {2, 1};
    const std::vector<std::uint32_t> lean_costs = {10, 5};
    IsoAdj lean;
    lean.metadata.node_count = 3;
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
        std::vector<EdgeWeight> lean_distances;
        RunCappedDijkstraIsoAdj(lean, seeds, cap, lean_distances);

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

BOOST_AUTO_TEST_SUITE_END()
