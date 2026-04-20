#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
#include "contractor/phast_cap.hpp"
#include "contractor/phast_cli.hpp"
#include "contractor/phast_io.hpp"
#include "contractor/phast_runtime.hpp"
#include "contractor/phast_seeds.hpp"

#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"

#include "osrm/exception.hpp"

#include "util/exception.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using namespace osrm;

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
