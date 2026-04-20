#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
#include "contractor/phast_cap.hpp"
#include "contractor/phast_cli.hpp"
#include "contractor/phast_io.hpp"
#include "contractor/phast_runtime.hpp"
#include "contractor/phast_seeds.hpp"

#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"

#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/search_engine_data.hpp"

#include "osrm/exception.hpp"

#include "util/exception.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace osrm;

namespace
{

struct NodeOracleStats
{
    std::size_t sample_count = 0;
    std::size_t both_reachable_count = 0;
    std::size_t both_unreachable_count = 0;
    std::size_t reachability_mismatch_count = 0;
    std::size_t value_mismatch_count = 0;
    std::int64_t max_abs_delta_ticks = 0;
};

bool LoadOracleNodeFile(const std::filesystem::path &path,
                        const std::uint32_t node_count,
                        std::vector<NodeID> &nodes)
{
    std::ifstream input(path);
    if (!input)
    {
        util::Log(logERROR) << "Could not open oracle node file: " << path.string();
        return false;
    }

    nodes.clear();
    std::string line;
    std::uint64_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#')
        {
            continue;
        }
        const auto last = line.find_last_not_of(" \t\r\n");
        const auto token = line.substr(first, last - first + 1);

        std::istringstream parser(token);
        std::uint64_t source_node = 0;
        if (!(parser >> source_node))
        {
            util::Log(logERROR) << "Invalid oracle node value in " << path.string() << ":" << line_number;
            return false;
        }
        parser >> std::ws;
        if (!parser.eof())
        {
            util::Log(logERROR) << "Unexpected trailing data in " << path.string() << ":" << line_number;
            return false;
        }
        if (source_node >= node_count)
        {
            util::Log(logERROR) << "Oracle node id out of range in " << path.string() << ":"
                                << line_number;
            return false;
        }
        nodes.push_back(static_cast<NodeID>(source_node));
    }

    if (nodes.empty())
    {
        util::Log(logERROR) << "Oracle node file did not contain any node IDs: " << path.string();
        return false;
    }

    return true;
}

void InsertOrDecreaseReverseSeed(engine::SearchEngineData<engine::routing_algorithms::ch::Algorithm>::QueryHeap
                                     &reverse_heap,
                                 const contractor::phast::PHASTSeed &seed)
{
    const auto heap_node = reverse_heap.GetHeapNodeIfWasInserted(seed.node);
    if (!heap_node)
    {
        reverse_heap.Insert(seed.node, seed.cost, seed.node);
        return;
    }
    if (seed.cost >= heap_node->weight)
    {
        return;
    }
    heap_node->weight = seed.cost;
    heap_node->data.parent = seed.node;
    reverse_heap.DecreaseKey(*heap_node);
}

std::string WeightToString(const EdgeWeight weight)
{
    if (weight == INVALID_EDGE_WEIGHT)
    {
        return "INF";
    }
    return std::to_string(from_alias<std::int64_t>(weight));
}

bool RunNodeOracle(const contractor::phast::RuntimeConfig &runtime_config,
                   const contractor::phast::CHDataFacade &facade,
                   const std::vector<contractor::phast::PHASTSeed> &seeds,
                   const std::vector<EdgeWeight> &phast_distances,
                   NodeOracleStats &stats)
{
    std::vector<NodeID> source_nodes;
    if (!LoadOracleNodeFile(runtime_config.oracle_node_file, facade.GetNumberOfNodes(), source_nodes))
    {
        return false;
    }

    std::ofstream report;
    if (runtime_config.has_oracle_report_path)
    {
        report.open(runtime_config.oracle_report_path);
        if (!report)
        {
            util::Log(logERROR) << "Could not open oracle report for writing: "
                                << runtime_config.oracle_report_path.string();
            return false;
        }
        report << "source_node_id,phast_ticks,oracle_ticks,delta_ticks,status\n";
    }

    engine::SearchEngineData<engine::routing_algorithms::ch::Algorithm> search_data;
    search_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());
    auto &forward_heap = *search_data.forward_heap_1;
    auto &reverse_heap = *search_data.reverse_heap_1;

    std::vector<NodeID> packed_leg;
    packed_leg.reserve(32);

    stats = {};
    const auto tolerance_ticks = static_cast<std::int64_t>(runtime_config.oracle_tolerance);
    for (const auto source_node : source_nodes)
    {
        ++stats.sample_count;
        forward_heap.Clear();
        reverse_heap.Clear();
        packed_leg.clear();

        forward_heap.Insert(source_node, EdgeWeight{0}, source_node);
        for (const auto &seed : seeds)
        {
            InsertOrDecreaseReverseSeed(reverse_heap, seed);
        }

        EdgeWeight oracle_weight = INVALID_EDGE_WEIGHT;
        engine::routing_algorithms::ch::search(search_data,
                                               facade,
                                               forward_heap,
                                               reverse_heap,
                                               oracle_weight,
                                               packed_leg,
                                               {});

        const auto phast_weight = phast_distances[source_node];
        const bool phast_reachable = phast_weight != INVALID_EDGE_WEIGHT;
        const bool oracle_reachable = oracle_weight != INVALID_EDGE_WEIGHT;

        if (!phast_reachable && !oracle_reachable)
        {
            ++stats.both_unreachable_count;
            if (report)
            {
                report << source_node << ",INF,INF,,match\n";
            }
            continue;
        }

        if (phast_reachable != oracle_reachable)
        {
            ++stats.reachability_mismatch_count;
            if (report)
            {
                report << source_node << "," << WeightToString(phast_weight) << ","
                       << WeightToString(oracle_weight) << ",,reachability_mismatch\n";
            }
            continue;
        }

        ++stats.both_reachable_count;
        const auto phast_ticks = from_alias<std::int64_t>(phast_weight);
        const auto oracle_ticks = from_alias<std::int64_t>(oracle_weight);
        const auto delta_ticks = phast_ticks - oracle_ticks;
        const auto abs_delta_ticks = std::abs(delta_ticks);
        if (abs_delta_ticks > stats.max_abs_delta_ticks)
        {
            stats.max_abs_delta_ticks = abs_delta_ticks;
        }

        const bool within_tolerance = abs_delta_ticks <= tolerance_ticks;
        if (!within_tolerance)
        {
            ++stats.value_mismatch_count;
        }

        if (report)
        {
            report << source_node << "," << phast_ticks << "," << oracle_ticks << "," << delta_ticks
                   << "," << (within_tolerance ? "match" : "value_mismatch") << "\n";
        }
    }

    return true;
}

} // namespace

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();

    std::string verbosity;
    osrm::contractor::phast::PhastConfig phast_config;
    osrm::contractor::phast::RuntimeConfig runtime_config;

    const auto parse_result =
        osrm::contractor::phast::ParseArguments(argc, argv, verbosity, phast_config, runtime_config);
    if (parse_result == osrm::contractor::phast::ReturnCode::Fail)
    {
        return EXIT_FAILURE;
    }
    if (parse_result == osrm::contractor::phast::ReturnCode::Exit)
    {
        return EXIT_SUCCESS;
    }

    util::LogPolicy::GetInstance().SetLevel(verbosity);
    phast_config.UseDefaultOutputNames(phast_config.base_path);
    if (!phast_config.IsValid())
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Input file: " << phast_config.base_path.string() << ".osrm";

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(phast_config.GetPath(".osrm.properties"), properties);
    const std::string metric_name = properties.GetWeightName();

    std::unordered_map<std::string, osrm::contractor::ContractedMetric> metrics = {{metric_name, {}}};
    std::uint32_t hsgr_checksum = 0;
    osrm::contractor::files::readGraph(phast_config.GetPath(".osrm.hsgr"), metrics, hsgr_checksum);

    osrm::contractor::PhastData phast_data;
    osrm::contractor::files::readPhast(phast_config.GetPath(".osrm.phast"), phast_data);

    const auto &metric = metrics.at(metric_name);
    const auto &graph = metric.graph;

    if (!osrm::contractor::phast::ValidateMetadata(metric_name, metric, hsgr_checksum, phast_data) ||
        !osrm::contractor::phast::ValidateOrdering(phast_data))
    {
        return EXIT_FAILURE;
    }

    osrm::contractor::phast::MetricKind runtime_metric_kind = osrm::contractor::phast::MetricKind::Weight;
    if (!osrm::contractor::phast::ParseMetricKind(
            runtime_config.metric, metric_name, "--metric", runtime_metric_kind))
    {
        return EXIT_FAILURE;
    }

    osrm::contractor::phast::MetricKind seed_metric_kind = osrm::contractor::phast::MetricKind::Weight;
    if (!osrm::contractor::phast::ParseMetricKind(
            runtime_config.seed_metric, metric_name, "--seed-metric", seed_metric_kind))
    {
        return EXIT_FAILURE;
    }
    if (seed_metric_kind != runtime_metric_kind && metric_name != "duration")
    {
        util::Log(logERROR) << "--seed-metric must match --metric unless weight_name=duration.";
        return EXIT_FAILURE;
    }

    osrm::contractor::phast::OutputFormatKind output_format_kind =
        osrm::contractor::phast::OutputFormatKind::PhastFieldV1;
    if (!osrm::contractor::phast::ParseOutputFormatKind(runtime_config.output_format, output_format_kind))
    {
        util::Log(logERROR) << "Unsupported output format: " << runtime_config.output_format;
        return EXIT_FAILURE;
    }

    std::size_t selected_exclude_index = phast_data.exclude_index;
    if (!osrm::contractor::phast::ResolveExcludeIndex(
            runtime_config, phast_data, selected_exclude_index))
    {
        return EXIT_FAILURE;
    }

    osrm::contractor::PHASTOrientation selected_orientation = phast_data.orientation;
    if (!osrm::contractor::phast::ResolveOrientation(runtime_config, phast_data, selected_orientation))
    {
        return EXIT_FAILURE;
    }

    const auto output_path = runtime_config.has_output_path
                                 ? runtime_config.output_path
                                 : osrm::contractor::phast::DefaultOutputPath(
                                       phast_config.base_path, output_format_kind);

    std::shared_ptr<const osrm::contractor::phast::CHDataFacade> facade;
    if (!osrm::contractor::phast::LoadCHFacade(
            phast_config.base_path, metric_name, selected_exclude_index, facade) ||
        !osrm::contractor::phast::ValidateFacadeMetadata(*facade, phast_data, metric_name))
    {
        return EXIT_FAILURE;
    }

    const auto &selected_edge_filter = metric.edge_filter[selected_exclude_index];
    osrm::contractor::phast::DerivedAdjacency adjacency;
    if (!osrm::contractor::phast::DeriveAdjacency(graph,
                                                  selected_edge_filter,
                                                  phast_data,
                                                  selected_orientation,
                                                  runtime_metric_kind,
                                                  adjacency))
    {
        return EXIT_FAILURE;
    }

    std::vector<osrm::contractor::phast::PHASTSeed> seeds;
    osrm::contractor::phast::SeedBuildStats seed_stats;
    if (!osrm::contractor::phast::BuildSeeds(runtime_config,
                                             phast_data.node_count,
                                             seed_metric_kind,
                                             selected_orientation,
                                             *facade,
                                             seeds,
                                             seed_stats) ||
        !osrm::contractor::phast::ValidateSeeds(phast_data.node_count, seeds))
    {
        return EXIT_FAILURE;
    }

    std::optional<EdgeWeight> cap_metric;
    if (!osrm::contractor::phast::ParseTraversalCap(
            runtime_config, properties, runtime_metric_kind, cap_metric))
    {
        return EXIT_FAILURE;
    }

    std::vector<EdgeWeight> phast_distances;
    std::size_t upward_settled_nodes = 0;
    std::size_t downward_updates = 0;
    const auto phast_start = std::chrono::steady_clock::now();
    if (!osrm::contractor::phast::RunUpwardSearch(
            adjacency, seeds, cap_metric, phast_distances, upward_settled_nodes) ||
        !osrm::contractor::phast::RunDownwardSweep(
            phast_data, adjacency, cap_metric, phast_distances, &downward_updates))
    {
        return EXIT_FAILURE;
    }
    const auto phast_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               phast_start)
            .count();

    util::Log() << "CH metric: " << metric_name;
    util::Log() << "PHAST nodes: " << phast_data.node_count;
    util::Log() << "Runtime metric: "
                << (runtime_metric_kind == osrm::contractor::phast::MetricKind::Weight ? "weight"
                                                                                        : "duration");
    util::Log() << "Runtime orientation: "
                << osrm::contractor::phast::OrientationToString(selected_orientation);
    util::Log() << "Runtime exclude index: " << selected_exclude_index;
    util::Log() << "Pre-synthesis upward arcs: " << adjacency.pre_synthesis_upward_arc_count;
    util::Log() << "Pre-synthesis downward arcs: " << adjacency.pre_synthesis_downward_arc_count;
    util::Log() << "Downward adjacency built from transpose: "
                << (adjacency.downward_built_from_transpose ? "yes" : "no");
    util::Log() << "Derived upward arcs: " << adjacency.up_targets.size();
    util::Log() << "Derived downward arcs: " << adjacency.down_targets.size();
    util::Log() << "Seed count: " << seeds.size();
    if (runtime_config.has_poi_file)
    {
        util::Log() << "POIs parsed: " << seed_stats.poi_count;
        util::Log() << "Snap cache hits: " << seed_stats.cache_hits;
        util::Log() << "Big-component snap fallbacks: " << seed_stats.big_component_fallbacks;
    }

    if (cap_metric.has_value())
    {
        util::Log() << "Traversal cap: " << cap_metric.value() << " ticks";
    }
    else
    {
        util::Log() << "Traversal cap: none";
    }
    util::Log() << "Upward search settled nodes: " << upward_settled_nodes;
    util::Log() << "Downward sweep updates: " << downward_updates;
    util::Log() << "PHAST reachable nodes: "
                << osrm::contractor::phast::CountReachable(phast_distances)
                << ", max distance: " << osrm::contractor::phast::MaxDistance(phast_distances)
                << " ticks";
    util::Log() << "PHAST runtime: " << phast_elapsed << " ms";

    const auto poi_count = runtime_config.has_poi_file ? seed_stats.poi_count : 0;
    const osrm::contractor::phast::OutputMetadata output_metadata{
        metric_name,
        runtime_metric_kind,
        selected_orientation,
        selected_exclude_index,
        phast_data.connectivity_checksum,
        poi_count,
    };

    if (!osrm::contractor::phast::WriteOutput(
            output_path, output_format_kind, phast_distances, cap_metric, output_metadata))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Output format: " << runtime_config.output_format;
    util::Log() << "Wrote PHAST field output: " << output_path.string();

    if (runtime_config.has_oracle_node_file)
    {
        NodeOracleStats oracle_stats;
        if (!RunNodeOracle(runtime_config, *facade, seeds, phast_distances, oracle_stats))
        {
            return EXIT_FAILURE;
        }

        util::Log() << "Node-state oracle samples: " << oracle_stats.sample_count;
        util::Log() << "Node-state oracle both reachable: " << oracle_stats.both_reachable_count;
        util::Log() << "Node-state oracle both unreachable: " << oracle_stats.both_unreachable_count;
        util::Log() << "Node-state oracle reachability mismatches: "
                    << oracle_stats.reachability_mismatch_count;
        util::Log() << "Node-state oracle value mismatches (> " << runtime_config.oracle_tolerance
                    << " ticks): " << oracle_stats.value_mismatch_count;
        util::Log() << "Node-state oracle max abs delta: " << oracle_stats.max_abs_delta_ticks
                    << " ticks";
        if (runtime_config.has_oracle_report_path)
        {
            util::Log() << "Wrote node-state oracle report: "
                        << runtime_config.oracle_report_path.string();
        }

        if (oracle_stats.reachability_mismatch_count > 0 || oracle_stats.value_mismatch_count > 0)
        {
            util::Log(logERROR) << "Node-state oracle validation failed.";
            return EXIT_FAILURE;
        }
    }

    util::DumpMemoryStats();
    return EXIT_SUCCESS;
}
catch (const osrm::RuntimeError &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return e.GetCode();
}
catch (const util::exception &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return EXIT_FAILURE;
}
catch (const std::bad_alloc &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    util::Log(logERROR) << "Please provide more memory or consider using a larger swapfile";
    return EXIT_FAILURE;
}
#ifdef _WIN32
catch (const std::exception &e)
{
    util::Log(logERROR) << "[exception] " << e.what();
    return EXIT_FAILURE;
}
#endif
