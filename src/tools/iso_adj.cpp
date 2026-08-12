// osrm-iso-adj
//
// One-time exporter that turns the uncontracted edge-based graph (`.osrm.ebg`)
// into a compact, memory-mappable adjacency in CSR form, keyed to the
// edge-based node-id space (the same space used by the PHAST snap cache and
// seeds). The lean custom-isochrone path in `osrm-phast --iso-adj` mmaps this
// file and runs a capped Dijkstra, so a single 60-minute query only ever
// touches the reachable region instead of loading the whole CH and sweeping
// every node.
//
// The adjacency is stored REVERSED (one arc `v -> u` for every original
// directed arc `u -> v`). A plain Dijkstra from the POI's snapped node over the
// reversed graph yields `costs[node]` = shortest travel time from `node` to the
// POI, which is exactly what `--orientation reverse` produces in the full PHAST
// path, so the existing rasterizer consumes it unchanged.

#include "contractor/files.hpp"
#include "contractor/iso_adj.hpp"

#include "extractor/edge_based_edge.hpp"
#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"

#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/program_options.hpp>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

using namespace osrm;

namespace
{
std::filesystem::path BaseArtifactPath(const std::filesystem::path &base, const char *suffix)
{
    auto normalized = base;
    if (normalized.extension() == ".osrm")
    {
        normalized.replace_extension();
    }
    return std::filesystem::path{normalized.string() + suffix};
}
} // namespace

int main(int argc, char *argv[])
{
    std::filesystem::path base_path;
    std::filesystem::path output_path;
    std::string metric = "duration";

    namespace po = boost::program_options;
    po::options_description options("osrm-iso-adj <input.osrm> --output <path> [--metric duration]");
    options.add_options()("help,h", "Show help")(
        "metric",
        po::value<std::string>(&metric)->default_value("duration"),
        "Edge cost metric to export: duration|weight")(
        "output,o",
        po::value<std::filesystem::path>(&output_path),
        "Output adjacency file (default: <input>.iso_adj.<metric>)");
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
            ++offsets[static_cast<std::uint64_t>(e.target) + 1];
        }
        if (e.data.backward)
        {
            ++offsets[static_cast<std::uint64_t>(e.source) + 1];
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
            const auto pos = cursor[e.target]++;
            targets[pos] = e.source;
            costs[pos] = cost;
        }
        if (e.data.backward)
        {
            const auto pos = cursor[e.source]++;
            targets[pos] = e.target;
            costs[pos] = cost;
        }
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
    metadata.metric_name = phast_metadata.metric_name;
    metadata.build_id = contractor::phast::BuildIsoAdjBuildID(dataset_timestamp, metadata);
    if (!contractor::phast::WriteIsoAdj(output_path, metadata, offsets, targets, costs))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Wrote " << output_path.string() << " (nodes=" << node_count
                << ", arcs=" << edge_count << ", checksum=" << connectivity_checksum
                << ", metric=" << metric << ", profile=" << metadata.metric_name
                << ", exclude=" << metadata.exclude_index << ", orientation=reverse)";
    return EXIT_SUCCESS;
}
