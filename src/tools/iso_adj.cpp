// osrm-iso-adj
//
// One-time exporter that turns the uncontracted edge-based graph (`.osrm.ebg`)
// into a compact, memory-mappable adjacency in CSR form. States are ordered by
// a Morton key derived from road geometry, and the file includes the original
// edge-based node-id to spatial-id map used for snapped seeds. The lean
// custom-isochrone path in `osrm-phast --iso-adj` mmaps this file and runs a
// capped Dijkstra, so a query faults the reachable region instead of loading
// the whole CH and sweeping every node.
//
// The adjacency is stored REVERSED (one arc `v -> u` for every original
// directed arc `u -> v`). A plain Dijkstra from the POI's snapped node over the
// reversed graph yields `costs[node]` = shortest travel time from `node` to the
// POI, which is exactly what `--orientation reverse` produces in the full PHAST
// path, so the existing rasterizer consumes it unchanged.

#include "contractor/files.hpp"
#include "contractor/iso_adj.hpp"
#include "contractor/ordinal_identity.hpp"

#include "extractor/edge_based_edge.hpp"
#include "extractor/files.hpp"
#include "extractor/node_data_container.hpp"
#include "extractor/profile_properties.hpp"
#include "extractor/segment_data_container.hpp"

#include "util/coordinate.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/program_options.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <ranges>
#include <string>
#include <vector>

using namespace osrm;

namespace
{
constexpr std::array<char, 8> SNAP_MAGIC = {'P', 'H', 'S', 'N', 'A', 'P', '2', '\0'};
constexpr std::uint32_t SNAP_VERSION = 2;
constexpr std::size_t SNAP_HEADER_BYTES = 56;
constexpr std::size_t SNAP_RECORD_BYTES = 28;
constexpr std::uint8_t HAS_FORWARD = 1U << 0;
constexpr std::uint8_t FORWARD_OFFSET_VALID = 1U << 1;
constexpr std::uint8_t HAS_REVERSE = 1U << 2;
constexpr std::uint8_t REVERSE_OFFSET_VALID = 1U << 3;

struct SnapMetadata
{
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t resolution = 0;
    contractor::phast::OrdinalsIdentity ordinals;
};

std::uint32_t SpreadCoordinateBits(std::uint32_t value)
{
    value &= 0xffffU;
    value = (value | (value << 8U)) & 0x00ff00ffU;
    value = (value | (value << 4U)) & 0x0f0f0f0fU;
    value = (value | (value << 2U)) & 0x33333333U;
    return (value | (value << 1U)) & 0x55555555U;
}

std::uint32_t SpatialKey(const util::Coordinate coordinate)
{
    const auto longitude = static_cast<std::int64_t>(static_cast<std::int32_t>(coordinate.lon)) +
                           180'000'000LL;
    const auto latitude = static_cast<std::int64_t>(static_cast<std::int32_t>(coordinate.lat)) +
                          90'000'000LL;
    const auto x = static_cast<std::uint32_t>(longitude * 65'535LL / 360'000'000LL);
    const auto y = static_cast<std::uint32_t>(latitude * 65'535LL / 180'000'000LL);
    return SpreadCoordinateBits(x) | (SpreadCoordinateBits(y) << 1U);
}

template <typename T> T ReadUnaligned(const char *data)
{
    T value;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

std::filesystem::path BaseArtifactPath(const std::filesystem::path &base, const char *suffix)
{
    auto normalized = base;
    if (normalized.extension() == ".osrm")
    {
        normalized.replace_extension();
    }
    return std::filesystem::path{normalized.string() + suffix};
}

bool ReadSnapMetadata(const std::filesystem::path &path, SnapMetadata &metadata)
{
    std::ifstream input(path, std::ios::binary);
    std::array<char, SNAP_HEADER_BYTES> header{};
    if (!input.read(header.data(), header.size()) ||
        !std::equal(SNAP_MAGIC.begin(), SNAP_MAGIC.end(), header.begin()) ||
        ReadUnaligned<std::uint32_t>(header.data() + 8) != SNAP_VERSION ||
        ReadUnaligned<std::uint32_t>(header.data() + 20) != 0)
    {
        util::Log(logERROR) << "Invalid snap cache header: " << path.string();
        return false;
    }
    metadata.connectivity_checksum = ReadUnaligned<std::uint32_t>(header.data() + 12);
    metadata.resolution = ReadUnaligned<std::uint32_t>(header.data() + 16);
    metadata.ordinals.sample_count = ReadUnaligned<std::uint64_t>(header.data() + 24);
    metadata.ordinals.ordinals_file_size = ReadUnaligned<std::uint64_t>(header.data() + 32);
    metadata.ordinals.ordinals_hash_hi = ReadUnaligned<std::uint64_t>(header.data() + 40);
    metadata.ordinals.ordinals_hash_lo = ReadUnaligned<std::uint64_t>(header.data() + 48);
    if (metadata.ordinals.sample_count == 0 ||
        metadata.ordinals.sample_count > std::numeric_limits<std::uint32_t>::max() ||
        metadata.ordinals.sample_count >
            (std::numeric_limits<std::uint64_t>::max() - SNAP_HEADER_BYTES) / SNAP_RECORD_BYTES)
    {
        util::Log(logERROR) << "Invalid snap cache dimensions: " << path.string();
        return false;
    }
    std::error_code size_error;
    const auto expected = SNAP_HEADER_BYTES + metadata.ordinals.sample_count * SNAP_RECORD_BYTES;
    if (std::filesystem::file_size(path, size_error) != expected || size_error)
    {
        util::Log(logERROR) << "Snap cache size mismatch: " << path.string();
        return false;
    }
    return true;
}

bool BuildSpatialNodeMap(const std::filesystem::path &base_path,
                         const std::uint64_t node_count,
                         std::vector<std::uint32_t> &original_to_spatial)
{
    extractor::EdgeBasedNodeDataContainer node_data;
    extractor::SegmentDataContainer segment_data;
    std::vector<util::Coordinate> coordinates;
    try
    {
        extractor::files::readNodeData(BaseArtifactPath(base_path, ".osrm.ebg_nodes"), node_data);
        extractor::files::readSegmentData(BaseArtifactPath(base_path, ".osrm.geometry"),
                                          segment_data);
        extractor::files::readNodeCoordinates(BaseArtifactPath(base_path, ".osrm.nbg_nodes"),
                                              coordinates);
    }
    catch (const std::exception &error)
    {
        util::Log(logERROR) << "Failed reading spatial ordering artifacts: " << error.what();
        return false;
    }
    if (node_data.NumberOfNodes() != node_count || coordinates.empty())
    {
        util::Log(logERROR) << "Spatial ordering artifacts do not match the edge-based graph.";
        return false;
    }

    std::vector<std::uint64_t> order(node_count);
    for (std::uint32_t original = 0; original < node_count; ++original)
    {
        const auto geometry_id = node_data.GetGeometryID(original).id;
        if (geometry_id >= segment_data.GetNumberOfGeometries())
        {
            util::Log(logERROR) << "Edge-based node has an invalid geometry: " << original;
            return false;
        }
        const auto geometry = segment_data.GetForwardGeometry(geometry_id);
        const auto geometry_size = std::ranges::distance(geometry);
        if (geometry_size == 0)
        {
            util::Log(logERROR) << "Edge-based node has an empty geometry: " << original;
            return false;
        }
        const auto coordinate_node = *(geometry.begin() + geometry_size / 2);
        if (coordinate_node >= coordinates.size() || !coordinates[coordinate_node].IsValid())
        {
            util::Log(logERROR) << "Edge-based node geometry has an invalid coordinate: "
                                << original;
            return false;
        }
        order[original] = (static_cast<std::uint64_t>(SpatialKey(coordinates[coordinate_node]))
                           << 32U) |
                          original;
    }
    std::sort(order.begin(), order.end());
    original_to_spatial.resize(node_count);
    for (std::uint32_t spatial = 0; spatial < node_count; ++spatial)
    {
        original_to_spatial[static_cast<std::uint32_t>(order[spatial])] = spatial;
    }
    util::Log() << "Built Morton node order for " << node_count << " edge-based states.";
    return true;
}

template <typename Callback>
bool VisitSnapCandidates(const std::filesystem::path &path,
                         const std::uint64_t node_count,
                         Callback callback)
{
    std::ifstream input(path, std::ios::binary);
    input.seekg(SNAP_HEADER_BYTES);
    std::array<char, SNAP_RECORD_BYTES> record{};
    std::uint32_t ordinal = 0;
    while (input.read(record.data(), record.size()))
    {
        const auto flags = ReadUnaligned<std::uint8_t>(record.data() + 24);
        const auto visit = [&](const bool valid,
                               const std::size_t node_offset,
                               const std::size_t cost_offset) {
            if (!valid)
            {
                return true;
            }
            const auto node = ReadUnaligned<std::uint32_t>(record.data() + node_offset);
            if (node >= node_count)
            {
                util::Log(logERROR) << "Snap cache node is out of range at ordinal " << ordinal;
                return false;
            }
            callback(node, ordinal, ReadUnaligned<std::int32_t>(record.data() + cost_offset));
            return true;
        };
        if (!visit((flags & (HAS_FORWARD | FORWARD_OFFSET_VALID)) ==
                       (HAS_FORWARD | FORWARD_OFFSET_VALID),
                   8,
                   16) ||
            !visit((flags & (HAS_REVERSE | REVERSE_OFFSET_VALID)) ==
                       (HAS_REVERSE | REVERSE_OFFSET_VALID),
                   12,
                   20))
        {
            return false;
        }
        ++ordinal;
    }
    if (!input.eof())
    {
        util::Log(logERROR) << "Failed reading snap cache: " << path.string();
        return false;
    }
    return true;
}
} // namespace

int main(int argc, char *argv[])
{
    util::LogPolicy::GetInstance().Unmute();
    std::filesystem::path base_path;
    std::filesystem::path output_path;
    std::filesystem::path sample_snap_cache;
    std::filesystem::path sample_ordinals;
    std::string metric = "duration";
    std::string artifact_version;
    std::uint32_t resolution = 9;

    namespace po = boost::program_options;
    po::options_description options(
        "osrm-iso-adj <input.osrm> --output <path> --sample-snap-cache <path> "
        "--sample-ordinals <path> --artifact-version <version>");
    options.add_options()("help,h", "Show help")(
        "metric",
        po::value<std::string>(&metric)->default_value("duration"),
        "Edge cost metric to export: duration|weight")(
        "output,o",
        po::value<std::filesystem::path>(&output_path),
        "Output adjacency file (default: <input>.iso_adj.<metric>)")(
        "sample-snap-cache",
        po::value<std::filesystem::path>(&sample_snap_cache),
        "r9 primary snap cache to invert into the reached-state cell index")(
        "sample-ordinals",
        po::value<std::filesystem::path>(&sample_ordinals),
        "Ordinal table whose identity must match the snap cache")(
        "resolution",
        po::value<std::uint32_t>(&resolution)->default_value(9),
        "H3 resolution of the custom result")(
        "artifact-version",
        po::value<std::string>(&artifact_version),
        "Immutable application artifact version");
    po::options_description hidden;
    hidden.add_options()("input,i", po::value<std::filesystem::path>(&base_path), "Input base path");
    po::positional_options_description positional;
    positional.add("input", 1);
    po::options_description cmdline;
    cmdline.add(options).add(hidden);

    po::variables_map vm;
    try
    {
        po::store(po::command_line_parser(argc, argv).options(cmdline).positional(positional).run(),
                  vm);
        if (vm.contains("help"))
        {
            std::cout << options << std::endl;
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch (const po::error &e)
    {
        util::Log(logERROR) << e.what();
        return EXIT_FAILURE;
    }

    if (!vm.contains("input"))
    {
        std::cout << options << std::endl;
        return EXIT_FAILURE;
    }
    if (metric != "duration" && metric != "weight")
    {
        util::Log(logERROR) << "--metric must be 'duration' or 'weight'.";
        return EXIT_FAILURE;
    }
    if (!vm.contains("sample-snap-cache") || !vm.contains("sample-ordinals") ||
        !vm.contains("artifact-version") || artifact_version.empty() ||
        artifact_version.size() >= contractor::phast::ISO_ADJ_ARTIFACT_VERSION_BYTES)
    {
        util::Log(logERROR) << "--sample-snap-cache, --sample-ordinals, and a short non-empty "
                               "--artifact-version are required.";
        return EXIT_FAILURE;
    }
    const bool use_duration = (metric == "duration");
    if (!vm.contains("output"))
    {
        output_path = std::filesystem::path{base_path.string() + ".iso_adj." + metric};
    }

    const auto ebg_path = BaseArtifactPath(base_path, ".osrm.ebg");
    if (!std::filesystem::exists(ebg_path))
    {
        util::Log(logERROR) << "Edge-based graph file not found: " << ebg_path.string();
        return EXIT_FAILURE;
    }

    EdgeID number_of_edge_based_nodes = 0;
    std::vector<extractor::EdgeBasedEdge> edges;
    std::uint32_t connectivity_checksum = 0;
    contractor::PhastData phast_metadata;
    extractor::ProfileProperties properties;
    std::string dataset_timestamp;
    try
    {
        extractor::files::readEdgeBasedGraph(
            ebg_path, number_of_edge_based_nodes, edges, connectivity_checksum);
        contractor::files::readPhastMetadata(BaseArtifactPath(base_path, ".osrm.phast"),
                                             phast_metadata);
        extractor::files::readProfileProperties(BaseArtifactPath(base_path, ".osrm.properties"),
                                                properties);
        extractor::files::readTimestamp(BaseArtifactPath(base_path, ".osrm.timestamp"),
                                        dataset_timestamp);
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed reading OSRM artifacts for " << base_path.string() << ": "
                            << e.what();
        return EXIT_FAILURE;
    }

    const std::uint64_t node_count = number_of_edge_based_nodes;
    if (phast_metadata.version < contractor::PHAST_SCHEMA_VERSION ||
        phast_metadata.connectivity_checksum != connectivity_checksum ||
        phast_metadata.node_count != node_count ||
        phast_metadata.metric_name != properties.GetWeightName())
    {
        util::Log(logERROR)
            << "PHAST metadata does not match the edge-based graph/profile. Rebuild the OSRM base.";
        return EXIT_FAILURE;
    }
    if (phast_metadata.orientation != contractor::PHASTOrientation::Reverse)
    {
        util::Log(logERROR) << "osrm-iso-adj requires reverse PHAST orientation.";
        return EXIT_FAILURE;
    }
    if (phast_metadata.exclude_index >= properties.excludable_classes.size())
    {
        util::Log(logERROR) << "PHAST exclude index is out of range: "
                            << phast_metadata.exclude_index;
        return EXIT_FAILURE;
    }
    util::Log() << "Edge-based nodes: " << node_count << ", directed edges: " << edges.size();

    std::vector<std::uint32_t> original_to_spatial;
    if (!BuildSpatialNodeMap(base_path, node_count, original_to_spatial))
    {
        return EXIT_FAILURE;
    }

    const auto cost_of = [&](const extractor::EdgeBasedEdge &e) -> std::uint32_t {
        if (use_duration)
        {
            const auto ticks = static_cast<std::int64_t>(e.data.duration);
            return ticks > 0 ? static_cast<std::uint32_t>(ticks) : 0u;
        }
        const auto ticks = from_alias<std::int64_t>(e.data.weight);
        return ticks > 0 ? static_cast<std::uint32_t>(ticks) : 0u;
    };

    // Pass 1: out-degree of every reversed-graph source node.
    // Original forward arc src->tgt  => reversed arc tgt->src (out of `tgt`).
    // Original backward arc tgt->src => reversed arc src->tgt (out of `src`).
    std::vector<std::uint64_t> offsets(node_count + 1, 0);
    for (const auto &e : edges)
    {
        if (e.data.forward)
        {
            ++offsets[static_cast<std::uint64_t>(original_to_spatial[e.target]) + 1];
        }
        if (e.data.backward)
        {
            ++offsets[static_cast<std::uint64_t>(original_to_spatial[e.source]) + 1];
        }
    }
    for (std::uint64_t i = 0; i < node_count; ++i)
    {
        offsets[i + 1] += offsets[i];
    }
    const std::uint64_t edge_count = offsets[node_count];

    std::vector<std::uint32_t> targets(edge_count);
    std::vector<std::uint32_t> costs(edge_count);
    std::vector<std::uint64_t> cursor(offsets.begin(), offsets.begin() + node_count);
    for (const auto &e : edges)
    {
        const auto cost = cost_of(e);
        if (e.data.forward)
        {
            const auto pos = cursor[original_to_spatial[e.target]]++;
            targets[pos] = original_to_spatial[e.source];
            costs[pos] = cost;
        }
        if (e.data.backward)
        {
            const auto pos = cursor[original_to_spatial[e.source]]++;
            targets[pos] = original_to_spatial[e.target];
            costs[pos] = cost;
        }
    }

    SnapMetadata snap_metadata;
    contractor::phast::OrdinalsIdentity ordinal_identity;
    std::string identity_error;
    if (!ReadSnapMetadata(sample_snap_cache, snap_metadata) ||
        !contractor::phast::ReadOrdinalsIdentity(
            sample_ordinals, resolution, ordinal_identity, identity_error) ||
        snap_metadata.connectivity_checksum != connectivity_checksum ||
        snap_metadata.resolution != resolution || !(snap_metadata.ordinals == ordinal_identity))
    {
        util::Log(logERROR) << "Snap cache, ordinals, or graph identity mismatch. "
                            << identity_error;
        return EXIT_FAILURE;
    }

    std::vector<std::uint64_t> cell_offsets(node_count + 1, 0);
    if (!VisitSnapCandidates(sample_snap_cache,
                             node_count,
                             [&](const auto node, const auto, const auto) {
                                 ++cell_offsets[static_cast<std::uint64_t>(
                                                    original_to_spatial[node]) +
                                                1];
                             }))
    {
        return EXIT_FAILURE;
    }
    for (std::uint64_t node = 0; node < node_count; ++node)
    {
        cell_offsets[node + 1] += cell_offsets[node];
    }
    const auto candidate_count = cell_offsets[node_count];
    std::vector<std::uint32_t> candidate_ordinals(candidate_count);
    std::vector<std::int32_t> candidate_costs(candidate_count);
    std::vector<std::uint64_t> cell_cursor(cell_offsets.begin(), cell_offsets.begin() + node_count);
    if (!VisitSnapCandidates(sample_snap_cache,
                             node_count,
                             [&](const auto node, const auto ordinal, const auto offset) {
                                 const auto position = cell_cursor[original_to_spatial[node]]++;
                                 candidate_ordinals[position] = ordinal;
                                 candidate_costs[position] = offset;
                             }))
    {
        return EXIT_FAILURE;
    }

    contractor::phast::IsoAdjMetadata metadata;
    metadata.phast_schema_version = phast_metadata.version;
    metadata.metric_kind = use_duration ? contractor::phast::MetricKind::Duration
                                        : contractor::phast::MetricKind::Weight;
    metadata.orientation = phast_metadata.orientation;
    metadata.exclude_index = phast_metadata.exclude_index;
    metadata.connectivity_checksum = connectivity_checksum;
    metadata.node_count = node_count;
    metadata.edge_count = edge_count;
    metadata.candidate_count = candidate_count;
    metadata.metric_name = phast_metadata.metric_name;
    metadata.resolution = resolution;
    metadata.artifact_version = artifact_version;
    metadata.ordinal_count = ordinal_identity.sample_count;
    metadata.ordinals_file_size = ordinal_identity.ordinals_file_size;
    metadata.ordinals_hash_hi = ordinal_identity.ordinals_hash_hi;
    metadata.ordinals_hash_lo = ordinal_identity.ordinals_hash_lo;
    metadata.build_id = contractor::phast::BuildIsoAdjBuildID(dataset_timestamp, metadata);
    if (!contractor::phast::WriteIsoAdj(output_path,
                                        metadata,
                                        offsets,
                                        targets,
                                        costs,
                                        cell_offsets,
                                        candidate_ordinals,
                                        candidate_costs,
                                        original_to_spatial))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Wrote " << output_path.string() << " (nodes=" << node_count
                << ", arcs=" << edge_count << ", cell_candidates=" << candidate_count
                << ", checksum=" << connectivity_checksum
                << ", metric=" << metric << ", profile=" << metadata.metric_name
                << ", exclude=" << metadata.exclude_index << ", orientation=reverse, r"
                << resolution << ", artifact=" << artifact_version << ")";
    return EXIT_SUCCESS;
}
