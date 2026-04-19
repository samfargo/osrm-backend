#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
#include "engine/approach.hpp"
#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/process_memory_allocator.hpp"
#include "engine/hint.hpp"
#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/exception.hpp"
#include "storage/io_config.hpp"
#include "storage/storage_config.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/query_heap.hpp"
#include "util/version.hpp"

#include <boost/program_options.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace osrm;

namespace
{
struct PhastConfig final : storage::IOConfig
{
    PhastConfig() : IOConfig({".osrm.hsgr", ".osrm.properties", ".osrm.phast"}, {}, {}) {}

    void UseDefaultOutputNames(const std::filesystem::path &base) { IOConfig::UseDefaultOutputNames(base); }

    bool IsValid() const { return IOConfig::IsValid(); }
};

struct RuntimeConfig final
{
    std::vector<NodeID> seed_nodes;
    std::filesystem::path seed_file;
    bool has_seed_file = false;
    std::filesystem::path poi_file;
    bool has_poi_file = false;
    std::string seed_metric = "weight";
    double cap_seconds = -1.;
    std::size_t debug_mismatch_limit = 0;
    std::size_t debug_arc_sample_limit = 8;
};

enum class return_code : unsigned
{
    ok,
    fail,
    exit
};

enum class SeedMetricKind : std::uint8_t
{
    Weight = 0,
    Duration = 1
};

enum class SeedClass : std::uint8_t
{
    Bidirectional = 0,
    ForwardOnly = 1,
    ReverseOnly = 2,
    Manual = 3
};

struct PHASTSeed
{
    NodeID node = SPECIAL_NODEID;
    EdgeWeight cost = INVALID_EDGE_WEIGHT;
    SeedClass seed_class = SeedClass::Bidirectional;
};

return_code
parseArguments(int argc,
               char *argv[],
               std::string &verbosity,
               PhastConfig &phast_config,
               RuntimeConfig &runtime_config)
{
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
        "list-inputs", "List required and optional input file extensions")(
        "verbosity,l",
        boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
        std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

    boost::program_options::options_description execution_options("Execution");
    execution_options.add_options()(
        "seed-node",
        boost::program_options::value<std::vector<NodeID>>(&runtime_config.seed_nodes)
            ->multitoken()
            ->composing(),
        "Seed node id(s) in CH node space")(
        "seed-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.seed_file),
        "Path to a text file containing one seed node id per line")(
        "poi-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.poi_file),
        "Path to a text file containing one POI coordinate per line as 'lon,lat'")(
        "seed-metric",
        boost::program_options::value<std::string>(&runtime_config.seed_metric)
            ->default_value("weight"),
        "Seed offset metric: weight|duration")(
        "cap-seconds",
        boost::program_options::value<double>(&runtime_config.cap_seconds),
        "Optional traversal cap in seconds")(
        "debug-mismatches",
        boost::program_options::value<std::size_t>(&runtime_config.debug_mismatch_limit)
            ->implicit_value(8),
        "Enable mismatch diagnostics and inspect up to N mismatches (default 8)")(
        "debug-arc-sample",
        boost::program_options::value<std::size_t>(&runtime_config.debug_arc_sample_limit)
            ->default_value(8),
        "Maximum arc samples per mismatch debug section");

    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i",
        boost::program_options::value<std::filesystem::path>(&phast_config.base_path),
        "Input base file path");

    boost::program_options::positional_options_description positional_options;
    positional_options.add("input", 1);

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(execution_options).add(hidden_options);

    const auto *executable = argv[0];
    boost::program_options::options_description visible_options(
        "Usage: " + std::filesystem::path(executable).filename().string() + " <input.osrm> [options]");
    visible_options.add(generic_options).add(execution_options);

    boost::program_options::variables_map option_variables;
    try
    {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                          .options(cmdline_options)
                                          .positional(positional_options)
                                          .run(),
                                      option_variables);
    }
    catch (const boost::program_options::error &e)
    {
        util::Log(logERROR) << e.what();
        return return_code::fail;
    }

    if (option_variables.contains("version"))
    {
        std::cout << OSRM_VERSION << std::endl;
        return return_code::exit;
    }

    if (option_variables.contains("help"))
    {
        std::cout << visible_options;
        return return_code::exit;
    }

    if (option_variables.contains("list-inputs"))
    {
        PhastConfig config;
        std::set<std::string> seen;
        config.ListInputFiles(std::cout, seen);
        return return_code::exit;
    }

    boost::program_options::notify(option_variables);

    if (!option_variables.contains("input"))
    {
        std::cout << visible_options;
        return return_code::fail;
    }
    if (option_variables.contains("seed-file"))
    {
        runtime_config.has_seed_file = true;
    }
    if (option_variables.contains("poi-file"))
    {
        runtime_config.has_poi_file = true;
    }
    const bool has_manual_seeds = !runtime_config.seed_nodes.empty() || runtime_config.has_seed_file;
    if (has_manual_seeds == runtime_config.has_poi_file)
    {
        util::Log(logERROR)
            << "Provide exactly one seed source: --poi-file or --seed-node/--seed-file.";
        return return_code::fail;
    }
    if (option_variables.contains("cap-seconds") && runtime_config.cap_seconds < 0.)
    {
        util::Log(logERROR) << "--cap-seconds must be >= 0.";
        return return_code::fail;
    }
    if (runtime_config.seed_metric != "weight" && runtime_config.seed_metric != "duration")
    {
        util::Log(logERROR) << "--seed-metric must be 'weight' or 'duration'.";
        return return_code::fail;
    }
    if (runtime_config.debug_mismatch_limit > 0 && runtime_config.debug_arc_sample_limit == 0)
    {
        util::Log(logERROR) << "--debug-arc-sample must be > 0 when debug mode is enabled.";
        return return_code::fail;
    }

    return return_code::ok;
}

bool validateOrdering(const contractor::PhastData &phast_data)
{
    const auto node_count = phast_data.node_count;
    const auto &ordering = phast_data.ordering;

    if (ordering.order.size() != node_count || ordering.rank.size() != node_count)
    {
        util::Log(logERROR) << "PHAST ordering size mismatch.";
        return false;
    }
    if (ordering.contraction_to_original.size() != node_count ||
        ordering.original_to_contraction.size() != node_count)
    {
        util::Log(logERROR) << "PHAST permutation size mismatch.";
        return false;
    }

    for (const auto node : ordering.rank)
    {
        if (node >= node_count)
        {
            util::Log(logERROR) << "PHAST rank contains invalid node id.";
            return false;
        }
    }

    for (const auto i : util::irange<std::size_t>(0, node_count))
    {
        const auto node = ordering.order[i];
        if (node >= node_count)
        {
            util::Log(logERROR) << "PHAST order contains invalid node id at position " << i;
            return false;
        }
        if (ordering.rank[node] != i)
        {
            util::Log(logERROR) << "PHAST rank/order inverse mismatch at position " << i;
            return false;
        }
    }

    for (const auto local_id : util::irange<NodeID>(0, node_count))
    {
        if (ordering.contraction_to_original[local_id] >= node_count)
        {
            util::Log(logERROR) << "PHAST contraction_to_original contains invalid node id.";
            return false;
        }
    }

    for (const auto original_id : util::irange<NodeID>(0, node_count))
    {
        const auto local_id = ordering.original_to_contraction[original_id];
        if (local_id >= node_count || ordering.contraction_to_original[local_id] != original_id)
        {
            util::Log(logERROR) << "PHAST permutation inverse mismatch at node " << original_id;
            return false;
        }
    }

    return true;
}

const char *orientationToString(const contractor::PHASTOrientation orientation)
{
    switch (orientation)
    {
    case contractor::PHASTOrientation::Forward:
        return "forward";
    case contractor::PHASTOrientation::Reverse:
        return "reverse";
    }
    return "unknown";
}

bool isValidOrientation(const contractor::PHASTOrientation orientation)
{
    return orientation == contractor::PHASTOrientation::Forward ||
           orientation == contractor::PHASTOrientation::Reverse;
}

bool usesArc(const contractor::QueryEdge::EdgeData &edge_data,
             const contractor::PHASTOrientation orientation)
{
    switch (orientation)
    {
    case contractor::PHASTOrientation::Forward:
        return edge_data.forward;
    case contractor::PHASTOrientation::Reverse:
        return edge_data.backward;
    }
    return false;
}

bool validateMetadata(const std::string &metric_name,
                      const contractor::ContractedMetric &metric,
                      const std::uint32_t hsgr_checksum,
                      const contractor::PhastData &phast_data)
{
    if (phast_data.version < contractor::PHAST_SCHEMA_VERSION)
    {
        util::Log(logERROR) << "Unsupported .osrm.phast version " << phast_data.version
                            << ". Expected version >= " << contractor::PHAST_SCHEMA_VERSION << ".";
        return false;
    }
    if (!isValidOrientation(phast_data.orientation))
    {
        util::Log(logERROR) << "Unsupported PHAST orientation value.";
        return false;
    }
    if (metric_name != phast_data.metric_name)
    {
        util::Log(logERROR) << "Metric name mismatch between .osrm.properties and .osrm.phast";
        return false;
    }
    if (hsgr_checksum != phast_data.connectivity_checksum)
    {
        util::Log(logERROR) << "Checksum mismatch between .osrm.hsgr and .osrm.phast";
        return false;
    }
    if (metric.graph.GetNumberOfNodes() != phast_data.node_count)
    {
        util::Log(logERROR) << "Node count mismatch between .osrm.hsgr and .osrm.phast";
        return false;
    }
    if (phast_data.exclude_index >= metric.edge_filter.size())
    {
        util::Log(logERROR) << "PHAST exclude index " << phast_data.exclude_index
                            << " is out of range for this CH metric.";
        return false;
    }
    if (metric.edge_filter[phast_data.exclude_index].size() != metric.graph.GetNumberOfEdges())
    {
        util::Log(logERROR) << "Edge filter size mismatch for exclude index "
                            << phast_data.exclude_index << ".";
        return false;
    }
    return true;
}

struct DerivedAdjacency
{
    std::vector<std::uint32_t> up_offsets;
    std::vector<NodeID> up_targets;
    std::vector<EdgeWeight> up_costs;
    std::vector<std::uint32_t> down_offsets;
    std::vector<NodeID> down_targets;
    std::vector<EdgeWeight> down_costs;
    std::vector<std::uint32_t> all_offsets;
    std::vector<NodeID> all_targets;
    std::vector<EdgeWeight> all_costs;
    std::size_t oriented_arc_count = 0;
    std::size_t skipped_self_loops = 0;
};

using DistanceHeap = util::QueryHeap<NodeID, NodeID, EdgeWeight, NodeID>;

struct DistanceTrace
{
    std::vector<NodeID> predecessor;
    std::vector<EdgeWeight> incoming_cost;
};

bool classifyArc(const contractor::PhastData &phast_data,
                 const NodeID source,
                 const NodeID target,
                 bool &is_upward)
{
    const auto source_rank = phast_data.ordering.rank[source];
    const auto target_rank = phast_data.ordering.rank[target];
    if (source_rank == target_rank)
    {
        util::Log(logERROR) << "Rank tie detected for edge " << source << " -> " << target;
        return false;
    }

    is_upward = source_rank < target_rank;
    return true;
}

bool deriveAdjacency(const contractor::QueryGraph &graph,
                     const std::vector<bool> &edge_filter,
                     const contractor::PhastData &phast_data,
                     DerivedAdjacency &adjacency)
{
    const auto node_count = phast_data.node_count;
    adjacency.up_offsets.assign(node_count + 1, 0);
    adjacency.down_offsets.assign(node_count + 1, 0);
    adjacency.all_offsets.assign(node_count + 1, 0);

    for (const auto source : util::irange<NodeID>(0, node_count))
    {
        for (const auto edge : graph.GetAdjacentEdgeRange(source))
        {
            const auto edge_index = static_cast<std::size_t>(edge);
            if (edge_index >= edge_filter.size())
            {
                util::Log(logERROR) << "Edge filter index overflow at edge " << edge_index;
                return false;
            }
            if (!edge_filter[edge_index])
            {
                continue;
            }
            const auto target = graph.GetTarget(edge);
            const auto &edge_data = graph.GetEdgeData(edge);
            if (!usesArc(edge_data, phast_data.orientation))
            {
                continue;
            }
            if (source == target)
            {
                ++adjacency.skipped_self_loops;
                continue;
            }
            const auto edge_weight = edge_data.weight;
            if (edge_weight <= EdgeWeight{0} || edge_weight == INVALID_EDGE_WEIGHT)
            {
                util::Log(logERROR) << "Encountered invalid arc weight on edge " << source << " -> "
                                    << target;
                return false;
            }

            ++adjacency.all_offsets[source + 1];
            ++adjacency.oriented_arc_count;

            bool is_upward = false;
            if (!classifyArc(phast_data, source, target, is_upward))
            {
                return false;
            }

            if (is_upward)
            {
                ++adjacency.up_offsets[source + 1];
            }
            else
            {
                ++adjacency.down_offsets[source + 1];
            }
        }
    }

    for (const auto index : util::irange<std::size_t>(1, adjacency.up_offsets.size()))
    {
        adjacency.up_offsets[index] += adjacency.up_offsets[index - 1];
        adjacency.down_offsets[index] += adjacency.down_offsets[index - 1];
        adjacency.all_offsets[index] += adjacency.all_offsets[index - 1];
    }

    adjacency.up_targets.resize(adjacency.up_offsets.back());
    adjacency.up_costs.resize(adjacency.up_offsets.back());
    adjacency.down_targets.resize(adjacency.down_offsets.back());
    adjacency.down_costs.resize(adjacency.down_offsets.back());
    adjacency.all_targets.resize(adjacency.all_offsets.back());
    adjacency.all_costs.resize(adjacency.all_offsets.back());

    auto up_cursor = adjacency.up_offsets;
    auto down_cursor = adjacency.down_offsets;
    auto all_cursor = adjacency.all_offsets;

    for (const auto source : util::irange<NodeID>(0, node_count))
    {
        for (const auto edge : graph.GetAdjacentEdgeRange(source))
        {
            const auto edge_index = static_cast<std::size_t>(edge);
            if (edge_index >= edge_filter.size())
            {
                util::Log(logERROR) << "Edge filter index overflow at edge " << edge_index;
                return false;
            }
            if (!edge_filter[edge_index])
            {
                continue;
            }
            const auto target = graph.GetTarget(edge);
            const auto &edge_data = graph.GetEdgeData(edge);
            if (!usesArc(edge_data, phast_data.orientation))
            {
                continue;
            }
            if (source == target)
            {
                continue;
            }
            const auto edge_weight = edge_data.weight;

            bool is_upward = false;
            if (!classifyArc(phast_data, source, target, is_upward))
            {
                return false;
            }

            if (all_cursor[source] >= adjacency.all_offsets[source + 1])
            {
                util::Log(logERROR) << "Derived full adjacency buffer overflow for source node "
                                    << source;
                return false;
            }
            adjacency.all_targets[all_cursor[source]] = target;
            adjacency.all_costs[all_cursor[source]] = edge_weight;
            ++all_cursor[source];

            auto &cursor = is_upward ? up_cursor[source] : down_cursor[source];
            const auto limit =
                is_upward ? adjacency.up_offsets[source + 1] : adjacency.down_offsets[source + 1];
            auto &targets = is_upward ? adjacency.up_targets : adjacency.down_targets;
            auto &costs = is_upward ? adjacency.up_costs : adjacency.down_costs;
            if (cursor >= limit)
            {
                util::Log(logERROR) << "Derived adjacency buffer overflow for source node " << source;
                return false;
            }
            targets[cursor++] = target;
            costs[cursor - 1] = edge_weight;
        }
    }

    for (const auto source : util::irange<NodeID>(0, node_count))
    {
        if (up_cursor[source] != adjacency.up_offsets[source + 1] ||
            down_cursor[source] != adjacency.down_offsets[source + 1] ||
            all_cursor[source] != adjacency.all_offsets[source + 1])
        {
            util::Log(logERROR) << "Derived adjacency fill mismatch for source node " << source;
            return false;
        }
    }

    return true;
}

bool tryAddWeights(const EdgeWeight lhs, const EdgeWeight rhs, EdgeWeight &result)
{
    if (lhs == INVALID_EDGE_WEIGHT || rhs == INVALID_EDGE_WEIGHT)
    {
        return false;
    }

    const auto sum = from_alias<std::int64_t>(lhs) + from_alias<std::int64_t>(rhs);
    const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
    if (sum < 0 || sum > max_valid)
    {
        return false;
    }

    result = to_alias<EdgeWeight>(sum);
    return true;
}

bool exceedsCap(const EdgeWeight distance, const std::optional<EdgeWeight> &cap_weight)
{
    return cap_weight.has_value() && distance > cap_weight.value();
}

bool relaxWithHeap(const NodeID source,
                   const std::vector<std::uint32_t> &offsets,
                   const std::vector<NodeID> &targets,
                   const std::vector<EdgeWeight> &costs,
                   const std::optional<EdgeWeight> &cap_weight,
                   std::vector<EdgeWeight> &distances,
                   DistanceHeap &heap,
                   DistanceTrace *trace = nullptr,
                   std::size_t *update_count = nullptr)
{
    const auto source_distance = distances[source];
    if (source_distance == INVALID_EDGE_WEIGHT)
    {
        return true;
    }

    const auto begin = static_cast<std::size_t>(offsets[source]);
    const auto end = static_cast<std::size_t>(offsets[source + 1]);
    for (const auto index : util::irange<std::size_t>(begin, end))
    {
        const auto target = targets[index];
        const auto edge_weight = costs[index];
        EdgeWeight candidate = INVALID_EDGE_WEIGHT;
        if (!tryAddWeights(source_distance, edge_weight, candidate))
        {
            util::Log(logERROR) << "Distance overflow while relaxing " << source << " -> " << target;
            return false;
        }
        if (exceedsCap(candidate, cap_weight))
        {
            continue;
        }
        if (candidate >= distances[target])
        {
            continue;
        }

        distances[target] = candidate;
        if (update_count)
        {
            ++(*update_count);
        }
        if (trace)
        {
            trace->predecessor[target] = source;
            trace->incoming_cost[target] = edge_weight;
        }
        const auto to_heap_node = heap.GetHeapNodeIfWasInserted(target);
        if (!to_heap_node)
        {
            heap.Insert(target, candidate, source);
            continue;
        }
        if (to_heap_node->WasRemoved())
        {
            util::Log(logERROR) << "Attempted to decrease a settled node " << target;
            return false;
        }
        to_heap_node->data = source;
        to_heap_node->weight = candidate;
        heap.DecreaseKey(*to_heap_node);
    }
    return true;
}

bool relaxWithoutHeap(const NodeID source,
                      const std::vector<std::uint32_t> &offsets,
                      const std::vector<NodeID> &targets,
                      const std::vector<EdgeWeight> &costs,
                      const std::optional<EdgeWeight> &cap_weight,
                      std::vector<EdgeWeight> &distances,
                      std::size_t *update_count = nullptr)
{
    const auto source_distance = distances[source];
    if (source_distance == INVALID_EDGE_WEIGHT)
    {
        return true;
    }
    if (exceedsCap(source_distance, cap_weight))
    {
        return true;
    }

    const auto begin = static_cast<std::size_t>(offsets[source]);
    const auto end = static_cast<std::size_t>(offsets[source + 1]);
    for (const auto index : util::irange<std::size_t>(begin, end))
    {
        const auto target = targets[index];
        const auto edge_weight = costs[index];
        EdgeWeight candidate = INVALID_EDGE_WEIGHT;
        if (!tryAddWeights(source_distance, edge_weight, candidate))
        {
            util::Log(logERROR) << "Distance overflow while sweeping " << source << " -> " << target;
            return false;
        }
        if (exceedsCap(candidate, cap_weight))
        {
            continue;
        }
        if (candidate < distances[target])
        {
            distances[target] = candidate;
            if (update_count)
            {
                ++(*update_count);
            }
        }
    }
    return true;
}

bool runUpwardSearch(const DerivedAdjacency &adjacency,
                     const std::vector<PHASTSeed> &seeds,
                     const std::optional<EdgeWeight> &cap_weight,
                     std::vector<EdgeWeight> &distances,
                     std::size_t &settled_nodes)
{
    const auto node_count = adjacency.up_offsets.size() - 1;
    distances.assign(node_count, INVALID_EDGE_WEIGHT);
    DistanceHeap heap(node_count);

    for (const auto &seed : seeds)
    {
        if (seed.node >= node_count)
        {
            util::Log(logERROR) << "Seed node id out of range: " << seed.node << " >= "
                                << node_count;
            return false;
        }
        if (seed.cost == INVALID_EDGE_WEIGHT || exceedsCap(seed.cost, cap_weight))
        {
            continue;
        }
        if (seed.cost >= distances[seed.node])
        {
            continue;
        }
        distances[seed.node] = seed.cost;
        heap.Insert(seed.node, seed.cost, seed.node);
    }

    settled_nodes = 0;
    while (!heap.Empty())
    {
        if (cap_weight.has_value() && heap.MinKey() > cap_weight.value())
        {
            break;
        }
        const auto heap_node = heap.DeleteMinGetHeapNode();
        if (heap_node.weight != distances[heap_node.node])
        {
            continue;
        }
        ++settled_nodes;
        if (!relaxWithHeap(heap_node.node,
                           adjacency.up_offsets,
                           adjacency.up_targets,
                           adjacency.up_costs,
                           cap_weight,
                           distances,
                           heap))
        {
            return false;
        }
    }
    return true;
}

bool runDownwardSweep(const contractor::PhastData &phast_data,
                      const DerivedAdjacency &adjacency,
                      const std::optional<EdgeWeight> &cap_weight,
                      std::vector<EdgeWeight> &distances,
                      std::size_t *update_count = nullptr)
{
    const auto node_count = phast_data.node_count;
    for (auto sweep_pos = node_count; sweep_pos > 0; --sweep_pos)
    {
        const auto source = phast_data.ordering.order[sweep_pos - 1];
        if (!relaxWithoutHeap(source,
                              adjacency.down_offsets,
                              adjacency.down_targets,
                              adjacency.down_costs,
                              cap_weight,
                              distances,
                              update_count))
        {
            return false;
        }
    }
    return true;
}

bool runReferenceDijkstra(const DerivedAdjacency &adjacency,
                          const std::vector<PHASTSeed> &seeds,
                          const std::optional<EdgeWeight> &cap_weight,
                          std::vector<EdgeWeight> &distances,
                          std::size_t &settled_nodes,
                          DistanceTrace &trace)
{
    const auto node_count = adjacency.all_offsets.size() - 1;
    distances.assign(node_count, INVALID_EDGE_WEIGHT);
    trace.predecessor.assign(node_count, SPECIAL_NODEID);
    trace.incoming_cost.assign(node_count, INVALID_EDGE_WEIGHT);
    DistanceHeap heap(node_count);

    for (const auto &seed : seeds)
    {
        if (seed.node >= node_count)
        {
            util::Log(logERROR) << "Seed node id out of range: " << seed.node << " >= "
                                << node_count;
            return false;
        }
        if (seed.cost == INVALID_EDGE_WEIGHT || exceedsCap(seed.cost, cap_weight))
        {
            continue;
        }
        if (seed.cost >= distances[seed.node])
        {
            continue;
        }
        distances[seed.node] = seed.cost;
        trace.predecessor[seed.node] = seed.node;
        trace.incoming_cost[seed.node] = EdgeWeight{0};
        heap.Insert(seed.node, seed.cost, seed.node);
    }

    settled_nodes = 0;
    while (!heap.Empty())
    {
        if (cap_weight.has_value() && heap.MinKey() > cap_weight.value())
        {
            break;
        }
        const auto heap_node = heap.DeleteMinGetHeapNode();
        if (heap_node.weight != distances[heap_node.node])
        {
            continue;
        }
        ++settled_nodes;
        if (!relaxWithHeap(heap_node.node,
                           adjacency.all_offsets,
                           adjacency.all_targets,
                           adjacency.all_costs,
                           cap_weight,
                           distances,
                           heap,
                           &trace))
        {
            return false;
        }
    }
    return true;
}

using CHDataFacade =
    engine::datafacade::ContiguousInternalMemoryDataFacade<engine::routing_algorithms::ch::Algorithm>;

const char *seedClassToString(const SeedClass seed_class)
{
    switch (seed_class)
    {
    case SeedClass::Bidirectional:
        return "bidirectional";
    case SeedClass::ForwardOnly:
        return "forward_only";
    case SeedClass::ReverseOnly:
        return "reverse_only";
    case SeedClass::Manual:
        return "manual";
    }
    return "unknown";
}

bool parseSeedMetricKind(const RuntimeConfig &runtime_config,
                         const std::string &weight_name,
                         SeedMetricKind &seed_metric_kind)
{
    if (runtime_config.seed_metric == "weight")
    {
        seed_metric_kind = SeedMetricKind::Weight;
        return true;
    }
    if (runtime_config.seed_metric == "duration")
    {
        if (weight_name != "duration")
        {
            util::Log(logERROR) << "--seed-metric=duration requires a duration-weighted dataset.";
            return false;
        }
        seed_metric_kind = SeedMetricKind::Duration;
        return true;
    }

    util::Log(logERROR) << "Unknown seed metric: " << runtime_config.seed_metric;
    return false;
}

bool loadManualSeedFile(const std::filesystem::path &seed_file, std::vector<NodeID> &seed_nodes)
{
    std::ifstream input(seed_file);
    if (!input)
    {
        util::Log(logERROR) << "Could not open manual seed file: " << seed_file.string();
        return false;
    }

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
        std::uint64_t node_id = 0;
        if (!(parser >> node_id))
        {
            util::Log(logERROR) << "Invalid seed value in " << seed_file.string() << ":"
                                << line_number;
            return false;
        }
        parser >> std::ws;
        if (!parser.eof())
        {
            util::Log(logERROR) << "Unexpected trailing data in " << seed_file.string() << ":"
                                << line_number;
            return false;
        }
        if (node_id > std::numeric_limits<NodeID>::max())
        {
            util::Log(logERROR) << "Seed id out of range in " << seed_file.string() << ":"
                                << line_number;
            return false;
        }
        seed_nodes.push_back(static_cast<NodeID>(node_id));
    }

    return true;
}

bool prepareManualSeeds(const RuntimeConfig &runtime_config,
                        const std::uint32_t node_count,
                        std::vector<PHASTSeed> &seeds)
{
    std::vector<NodeID> manual_seed_nodes = runtime_config.seed_nodes;
    if (runtime_config.has_seed_file &&
        !loadManualSeedFile(runtime_config.seed_file, manual_seed_nodes))
    {
        return false;
    }
    if (manual_seed_nodes.empty())
    {
        util::Log(logERROR) << "No manual seed nodes loaded.";
        return false;
    }

    std::sort(manual_seed_nodes.begin(), manual_seed_nodes.end());
    manual_seed_nodes.erase(std::unique(manual_seed_nodes.begin(), manual_seed_nodes.end()),
                            manual_seed_nodes.end());

    seeds.clear();
    seeds.reserve(manual_seed_nodes.size());
    for (const auto seed_node : manual_seed_nodes)
    {
        if (seed_node >= node_count)
        {
            util::Log(logERROR) << "Manual seed node id out of range: " << seed_node << " >= "
                                << node_count;
            return false;
        }
        seeds.push_back({seed_node, EdgeWeight{0}, SeedClass::Manual});
    }
    return true;
}

struct POIRecord
{
    util::Coordinate coordinate;
    std::uint64_t line_number = 0;
};

struct SeedBuildStats
{
    std::size_t poi_count = 0;
    std::size_t cache_hits = 0;
    std::size_t big_component_fallbacks = 0;
    std::size_t bidirectional_count = 0;
    std::size_t forward_only_count = 0;
    std::size_t reverse_only_count = 0;
    std::size_t directional_state_count = 0;
    std::size_t dedup_dropped_count = 0;
};

bool parsePOILine(const std::string &line,
                  const std::filesystem::path &poi_file,
                  const std::uint64_t line_number,
                  std::optional<POIRecord> &record)
{
    std::string token = line;
    const auto comment_pos = token.find('#');
    if (comment_pos != std::string::npos)
    {
        token.erase(comment_pos);
    }

    const auto first = token.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        record = std::nullopt;
        return true;
    }

    const auto last = token.find_last_not_of(" \t\r\n");
    token = token.substr(first, last - first + 1);
    std::replace(token.begin(), token.end(), ',', ' ');

    std::istringstream parser(token);
    double lon = 0.;
    double lat = 0.;
    if (!(parser >> lon >> lat))
    {
        util::Log(logERROR) << "Invalid POI coordinate in " << poi_file.string() << ":"
                            << line_number;
        return false;
    }
    parser >> std::ws;
    if (!parser.eof())
    {
        util::Log(logERROR) << "Unexpected trailing token in " << poi_file.string() << ":"
                            << line_number;
        return false;
    }
    if (!std::isfinite(lon) || !std::isfinite(lat))
    {
        util::Log(logERROR) << "Non-finite POI coordinate in " << poi_file.string() << ":"
                            << line_number;
        return false;
    }

    util::Coordinate coordinate;
    try
    {
        coordinate = util::Coordinate{
            util::UnsafeFloatLongitude{lon},
            util::UnsafeFloatLatitude{lat},
        };
    }
    catch (const std::exception &)
    {
        util::Log(logERROR) << "POI coordinate out of range in " << poi_file.string() << ":"
                            << line_number;
        return false;
    }

    if (!coordinate.IsValid())
    {
        util::Log(logERROR) << "POI coordinate invalid in " << poi_file.string() << ":"
                            << line_number;
        return false;
    }

    record = POIRecord{coordinate, line_number};
    return true;
}

bool loadPOIFile(const std::filesystem::path &poi_file, std::vector<POIRecord> &pois)
{
    std::ifstream input(poi_file);
    if (!input)
    {
        util::Log(logERROR) << "Could not open POI file: " << poi_file.string();
        return false;
    }

    pois.clear();
    std::string line;
    std::uint64_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        std::optional<POIRecord> record;
        if (!parsePOILine(line, poi_file, line_number, record))
        {
            return false;
        }
        if (record)
        {
            pois.push_back(record.value());
        }
    }

    if (pois.empty())
    {
        util::Log(logERROR) << "POI file did not contain any coordinates: " << poi_file.string();
        return false;
    }

    return true;
}

bool convertSeedCost(const engine::PhantomNode &phantom,
                     const bool forward_state,
                     const SeedMetricKind seed_metric_kind,
                     EdgeWeight &seed_cost)
{
    if (seed_metric_kind == SeedMetricKind::Duration)
    {
        const auto duration =
            forward_state ? phantom.GetForwardDuration() : phantom.GetReverseDuration();
        if (duration == MAXIMAL_EDGE_DURATION)
        {
            return false;
        }
        const auto duration_value = from_alias<std::int64_t>(duration);
        const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
        if (duration_value < 0 || duration_value > max_valid)
        {
            return false;
        }
        seed_cost = to_alias<EdgeWeight>(duration_value);
        return true;
    }

    seed_cost =
        forward_state ? phantom.GetForwardWeightPlusOffset() : phantom.GetReverseWeightPlusOffset();
    return seed_cost != INVALID_EDGE_WEIGHT && seed_cost >= EdgeWeight{0};
}

struct CoordinateKey
{
    std::int32_t lon;
    std::int32_t lat;

    bool operator==(const CoordinateKey &) const = default;
};

struct CoordinateKeyHash
{
    std::size_t operator()(const CoordinateKey &key) const
    {
        const auto lon = static_cast<std::uint32_t>(key.lon);
        const auto lat = static_cast<std::uint32_t>(key.lat);
        return (static_cast<std::uint64_t>(lon) << 32) | lat;
    }
};

SeedClass classifyTargetSeed(const bool forward_valid, const bool reverse_valid)
{
    if (forward_valid && reverse_valid)
    {
        return SeedClass::Bidirectional;
    }
    if (forward_valid)
    {
        return SeedClass::ForwardOnly;
    }
    return SeedClass::ReverseOnly;
}

bool buildPOISeeds(const RuntimeConfig &runtime_config,
                   const std::uint32_t node_count,
                   const SeedMetricKind seed_metric_kind,
                   const CHDataFacade &facade,
                   std::vector<PHASTSeed> &seeds,
                   SeedBuildStats &stats)
{
    std::vector<POIRecord> pois;
    if (!loadPOIFile(runtime_config.poi_file, pois))
    {
        return false;
    }
    stats.poi_count = pois.size();

    std::unordered_map<CoordinateKey, engine::SegmentHint, CoordinateKeyHash> snap_cache;
    std::vector<EdgeWeight> minimum_seed_cost(node_count, INVALID_EDGE_WEIGHT);
    std::vector<SeedClass> minimum_seed_class(node_count, SeedClass::Bidirectional);

    for (const auto &poi : pois)
    {
        const auto key = CoordinateKey{
            static_cast<std::int32_t>(poi.coordinate.lon),
            static_cast<std::int32_t>(poi.coordinate.lat),
        };

        engine::PhantomNode snapped_phantom;
        auto cache_iterator = snap_cache.find(key);
        if (cache_iterator != snap_cache.end() &&
            cache_iterator->second.IsValid(poi.coordinate, facade))
        {
            snapped_phantom = cache_iterator->second.phantom;
            ++stats.cache_hits;
        }
        else
        {
            auto alternatives = facade.NearestCandidatesWithAlternativeFromBigComponent(
                poi.coordinate, std::nullopt, std::nullopt, engine::Approach::UNRESTRICTED, true);
            if (!alternatives.first.empty())
            {
                snapped_phantom = alternatives.first.front();
            }
            else if (!alternatives.second.empty())
            {
                snapped_phantom = alternatives.second.front();
                ++stats.big_component_fallbacks;
            }
            else
            {
                util::Log(logERROR) << "No phantom candidate found for POI at line "
                                    << poi.line_number;
                return false;
            }

            snap_cache[key] = engine::SegmentHint{snapped_phantom, facade.GetCheckSum()};
        }

        if (!snapped_phantom.IsValid(node_count, poi.coordinate))
        {
            util::Log(logERROR) << "Invalid snapped phantom for POI at line " << poi.line_number;
            return false;
        }

        const auto forward_valid = snapped_phantom.IsValidForwardTarget();
        const auto reverse_valid = snapped_phantom.IsValidReverseTarget();
        if (!forward_valid && !reverse_valid)
        {
            util::Log(logERROR) << "Snapped phantom has no valid target states for POI line "
                                << poi.line_number;
            return false;
        }

        const auto seed_class = classifyTargetSeed(forward_valid, reverse_valid);
        switch (seed_class)
        {
        case SeedClass::Bidirectional:
            ++stats.bidirectional_count;
            break;
        case SeedClass::ForwardOnly:
            ++stats.forward_only_count;
            break;
        case SeedClass::ReverseOnly:
            ++stats.reverse_only_count;
            break;
        case SeedClass::Manual:
            break;
        }

        const auto push_seed = [&](const NodeID node, const EdgeWeight cost)
        {
            if (node >= node_count)
            {
                util::Log(logERROR) << "Snapped seed node out of range: " << node << " >= "
                                    << node_count;
                return false;
            }
            if (cost == INVALID_EDGE_WEIGHT || cost < EdgeWeight{0})
            {
                util::Log(logERROR) << "Invalid snapped seed cost for node " << node;
                return false;
            }

            auto &minimum = minimum_seed_cost[node];
            if (minimum == INVALID_EDGE_WEIGHT || cost < minimum)
            {
                if (minimum != INVALID_EDGE_WEIGHT)
                {
                    ++stats.dedup_dropped_count;
                }
                minimum = cost;
                minimum_seed_class[node] = seed_class;
            }
            else
            {
                ++stats.dedup_dropped_count;
            }
            ++stats.directional_state_count;
            return true;
        };

        if (forward_valid)
        {
            EdgeWeight forward_cost = INVALID_EDGE_WEIGHT;
            if (!convertSeedCost(snapped_phantom, true, seed_metric_kind, forward_cost))
            {
                util::Log(logERROR) << "Could not convert forward seed offset for POI line "
                                    << poi.line_number;
                return false;
            }
            if (!push_seed(snapped_phantom.forward_segment_id.id, forward_cost))
            {
                return false;
            }
        }
        if (reverse_valid)
        {
            EdgeWeight reverse_cost = INVALID_EDGE_WEIGHT;
            if (!convertSeedCost(snapped_phantom, false, seed_metric_kind, reverse_cost))
            {
                util::Log(logERROR) << "Could not convert reverse seed offset for POI line "
                                    << poi.line_number;
                return false;
            }
            if (!push_seed(snapped_phantom.reverse_segment_id.id, reverse_cost))
            {
                return false;
            }
        }
    }

    seeds.clear();
    for (const auto node : util::irange<NodeID>(0, node_count))
    {
        if (minimum_seed_cost[node] == INVALID_EDGE_WEIGHT)
        {
            continue;
        }
        seeds.push_back({node, minimum_seed_cost[node], minimum_seed_class[node]});
    }

    if (seeds.empty())
    {
        util::Log(logERROR) << "No usable PHAST seeds were generated from POIs.";
        return false;
    }

    return true;
}

bool validateSeeds(const std::uint32_t node_count, const std::vector<PHASTSeed> &seeds)
{
    if (seeds.empty())
    {
        util::Log(logERROR) << "No PHAST seeds available.";
        return false;
    }

    for (const auto &seed : seeds)
    {
        if (seed.node >= node_count)
        {
            util::Log(logERROR) << "Seed node out of range: " << seed.node << " >= " << node_count;
            return false;
        }
        if (seed.cost == INVALID_EDGE_WEIGHT || seed.cost < EdgeWeight{0})
        {
            util::Log(logERROR) << "Seed cost invalid for node " << seed.node;
            return false;
        }
    }
    return true;
}

bool loadCHFacade(const std::filesystem::path &base_path,
                  const std::string &metric_name,
                  const std::size_t exclude_index,
                  std::shared_ptr<const CHDataFacade> &facade)
{
    const storage::StorageConfig storage_config(base_path);
    if (!storage_config.IsValid())
    {
        util::Log(logERROR) << "Dataset files required for snapping are missing.";
        return false;
    }

    try
    {
        auto allocator =
            std::make_shared<engine::datafacade::ProcessMemoryAllocator>(storage_config);
        facade = std::make_shared<const CHDataFacade>(allocator, metric_name, exclude_index);
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to initialize CH facade for snapping: " << e.what();
        return false;
    }

    return true;
}

bool validateFacadeMetadata(const CHDataFacade &facade,
                            const contractor::PhastData &phast_data,
                            const std::string &metric_name)
{
    if (facade.GetCheckSum() != phast_data.connectivity_checksum)
    {
        util::Log(logWARNING) << "Facade checksum (" << facade.GetCheckSum()
                              << ") differs from PHAST connectivity checksum ("
                              << phast_data.connectivity_checksum << ").";
    }
    if (std::string{facade.GetWeightName()} != metric_name)
    {
        util::Log(logERROR) << "Weight name mismatch between facade and .osrm.properties";
        return false;
    }
    if (facade.GetNumberOfNodes() != phast_data.node_count)
    {
        util::Log(logERROR) << "Node count mismatch between facade and .osrm.phast";
        return false;
    }
    return true;
}

bool parseCapWeight(const RuntimeConfig &runtime_config,
                    const extractor::ProfileProperties &properties,
                    std::optional<EdgeWeight> &cap_weight)
{
    cap_weight = std::nullopt;
    if (runtime_config.cap_seconds < 0.)
    {
        return true;
    }
    if (!std::isfinite(runtime_config.cap_seconds))
    {
        util::Log(logERROR) << "--cap-seconds must be finite.";
        return false;
    }

    if (properties.GetWeightName() != "duration")
    {
        util::Log(logERROR) << "--cap-seconds requires weight_name=duration.";
        return false;
    }

    const auto cap_ticks = static_cast<std::int64_t>(
        std::ceil(runtime_config.cap_seconds * properties.GetWeightMultiplier()));
    const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
    if (cap_ticks < 0 || cap_ticks > max_valid)
    {
        util::Log(logERROR) << "--cap-seconds is out of range.";
        return false;
    }
    cap_weight = to_alias<EdgeWeight>(cap_ticks);
    return true;
}

std::size_t countReachable(const std::vector<EdgeWeight> &distances)
{
    return static_cast<std::size_t>(std::count_if(
        distances.begin(), distances.end(), [](const auto distance) { return distance != INVALID_EDGE_WEIGHT; }));
}

EdgeWeight maxDistance(const std::vector<EdgeWeight> &distances)
{
    EdgeWeight result{0};
    for (const auto distance : distances)
    {
        if (distance != INVALID_EDGE_WEIGHT && distance > result)
        {
            result = distance;
        }
    }
    return result;
}

struct DistanceValidationSummary
{
    std::size_t mismatch_count = 0;
    std::vector<NodeID> mismatch_nodes;
};

DistanceValidationSummary validateDistances(const contractor::PhastData &phast_data,
                                            const std::vector<EdgeWeight> &phast_distances,
                                            const std::vector<EdgeWeight> &reference_distances,
                                            const std::size_t sample_limit)
{
    DistanceValidationSummary summary;
    if (phast_distances.size() != reference_distances.size())
    {
        util::Log(logERROR) << "Distance vector size mismatch.";
        summary.mismatch_count = std::numeric_limits<std::size_t>::max();
        return summary;
    }

    constexpr std::size_t max_mismatch_logs = 8;
    for (const auto node : util::irange<NodeID>(0, static_cast<NodeID>(phast_distances.size())))
    {
        if (phast_distances[node] == reference_distances[node])
        {
            continue;
        }

        if (summary.mismatch_count < max_mismatch_logs)
        {
            util::Log(logERROR) << "Distance mismatch at node " << node
                                << " rank=" << phast_data.ordering.rank[node]
                                << " phast=" << phast_distances[node]
                                << " dijkstra=" << reference_distances[node];
        }
        if (summary.mismatch_nodes.size() < sample_limit)
        {
            summary.mismatch_nodes.push_back(node);
        }
        ++summary.mismatch_count;
    }

    if (summary.mismatch_count > 0)
    {
        util::Log(logERROR) << "PHAST mismatch count: " << summary.mismatch_count;
    }
    return summary;
}

void printArrayPreview(const char *label, const std::vector<NodeID> &values)
{
    std::ostringstream out;
    out << label << " [";
    const auto preview = std::min<std::size_t>(values.size(), 5);
    for (const auto i : util::irange<std::size_t>(0, preview))
    {
        if (i > 0)
            out << ", ";
        out << values[i];
    }
    if (values.size() > preview)
        out << ", ...";
    out << "]";
    util::Log() << out.str();
}

void printSeedPreview(const std::vector<PHASTSeed> &seeds)
{
    std::ostringstream out;
    out << "Seed preview [";
    const auto preview = std::min<std::size_t>(seeds.size(), 5);
    for (const auto i : util::irange<std::size_t>(0, preview))
    {
        if (i > 0)
        {
            out << ", ";
        }
        out << "{node=" << seeds[i].node << ", cost=" << seeds[i].cost
            << ", class=" << seedClassToString(seeds[i].seed_class) << "}";
    }
    if (seeds.size() > preview)
    {
        out << ", ...";
    }
    out << "]";
    util::Log() << out.str();
}

bool buildReferencePath(const NodeID target,
                        const DistanceTrace &trace,
                        std::vector<NodeID> &path_from_seed)
{
    path_from_seed.clear();
    if (target >= trace.predecessor.size())
    {
        return false;
    }
    if (trace.predecessor[target] == SPECIAL_NODEID)
    {
        return false;
    }

    std::vector<NodeID> reverse_path;
    NodeID current = target;
    for (const auto guard : util::irange<std::size_t>(0, trace.predecessor.size() + 1))
    {
        (void)guard;
        reverse_path.push_back(current);
        const auto parent = trace.predecessor[current];
        if (parent == current)
        {
            std::reverse(reverse_path.begin(), reverse_path.end());
            path_from_seed = std::move(reverse_path);
            return true;
        }
        if (parent == SPECIAL_NODEID)
        {
            return false;
        }
        current = parent;
    }

    util::Log(logERROR) << "Reference predecessor chain appears cyclic for target " << target;
    return false;
}

void logArcSamplesForNode(const contractor::PhastData &phast_data,
                          const DerivedAdjacency &adjacency,
                          const NodeID node,
                          const std::size_t sample_limit)
{
    const auto source_rank = phast_data.ordering.rank[node];

    std::size_t outgoing_logged = 0;
    const auto out_begin = static_cast<std::size_t>(adjacency.all_offsets[node]);
    const auto out_end = static_cast<std::size_t>(adjacency.all_offsets[node + 1]);
    for (const auto index : util::irange<std::size_t>(out_begin, out_end))
    {
        if (outgoing_logged >= sample_limit)
        {
            break;
        }
        const auto target = adjacency.all_targets[index];
        const auto target_rank = phast_data.ordering.rank[target];
        util::Log() << "    out " << node << " -> " << target << " cost=" << adjacency.all_costs[index]
                    << " rank=(" << source_rank << "," << target_rank
                    << ") class=" << (source_rank < target_rank ? "up" : "down");
        ++outgoing_logged;
    }
    if (outgoing_logged == 0)
    {
        util::Log() << "    out (none)";
    }

    std::size_t incoming_logged = 0;
    for (const auto source : util::irange<NodeID>(0, phast_data.node_count))
    {
        if (incoming_logged >= sample_limit)
        {
            break;
        }
        const auto begin = static_cast<std::size_t>(adjacency.all_offsets[source]);
        const auto end = static_cast<std::size_t>(adjacency.all_offsets[source + 1]);
        for (const auto index : util::irange<std::size_t>(begin, end))
        {
            if (incoming_logged >= sample_limit)
            {
                break;
            }
            if (adjacency.all_targets[index] != node)
            {
                continue;
            }
            const auto source_node_rank = phast_data.ordering.rank[source];
            util::Log() << "    in  " << source << " -> " << node
                        << " cost=" << adjacency.all_costs[index] << " rank=(" << source_node_rank
                        << "," << source_rank
                        << ") class=" << (source_node_rank < source_rank ? "up" : "down");
            ++incoming_logged;
        }
    }
    if (incoming_logged == 0)
    {
        util::Log() << "    in  (none)";
    }
}

void logMismatchDiagnostics(const contractor::PhastData &phast_data,
                            const DerivedAdjacency &adjacency,
                            const std::vector<EdgeWeight> &phast_distances,
                            const std::vector<EdgeWeight> &reference_distances,
                            const DistanceTrace &reference_trace,
                            const std::optional<EdgeWeight> &cap_weight,
                            const NodeID mismatch_node,
                            const std::size_t arc_sample_limit)
{
    util::Log() << "Mismatch debug node=" << mismatch_node
                << " rank=" << phast_data.ordering.rank[mismatch_node]
                << " phast=" << phast_distances[mismatch_node]
                << " dijkstra=" << reference_distances[mismatch_node];

    std::vector<NodeID> path;
    if (!buildReferencePath(mismatch_node, reference_trace, path))
    {
        util::Log() << "  no predecessor chain available for this node.";
        util::Log() << "  local arc samples:";
        logArcSamplesForNode(phast_data, adjacency, mismatch_node, arc_sample_limit);
        return;
    }

    util::Log() << "  predecessor path length: " << (path.size() - 1);
    {
        std::ostringstream out;
        out << "  path preview";
        const auto preview = std::min<std::size_t>(path.size(), 12);
        for (const auto i : util::irange<std::size_t>(0, preview))
        {
            const auto node = path[i];
            out << (i == 0 ? " " : " -> ") << node << "(r" << phast_data.ordering.rank[node] << ")";
        }
        if (path.size() > preview)
        {
            out << " -> ...";
        }
        util::Log() << out.str();
    }

    bool found_first_gap = false;
    NodeID first_gap_source = SPECIAL_NODEID;
    NodeID first_gap_target = SPECIAL_NODEID;
    for (const auto step : util::irange<std::size_t>(1, path.size()))
    {
        const auto source = path[step - 1];
        const auto target = path[step];
        const auto edge_cost = reference_trace.incoming_cost[target];

        if (phast_distances[source] == INVALID_EDGE_WEIGHT)
        {
            util::Log() << "  first missed relaxation: source " << source
                        << " is unreachable in PHAST along reference path.";
            found_first_gap = true;
            first_gap_source = source;
            first_gap_target = target;
            break;
        }

        EdgeWeight candidate = INVALID_EDGE_WEIGHT;
        const auto candidate_ok = tryAddWeights(phast_distances[source], edge_cost, candidate);
        if (!candidate_ok || exceedsCap(candidate, cap_weight))
        {
            util::Log() << "  first missed relaxation: source " << source
                        << " cannot relax " << target << " (candidate overflow/capped).";
            found_first_gap = true;
            first_gap_source = source;
            first_gap_target = target;
            break;
        }

        if (candidate < phast_distances[target])
        {
            util::Log() << "  first missed relaxation: " << source << " -> " << target
                        << " cost=" << edge_cost << " candidate=" << candidate
                        << " current_phast_target=" << phast_distances[target];
            found_first_gap = true;
            first_gap_source = source;
            first_gap_target = target;
            break;
        }
    }

    if (!found_first_gap)
    {
        util::Log() << "  no direct missed relaxation found along this predecessor chain.";
    }

    util::Log() << "  local arc samples around mismatch node:";
    logArcSamplesForNode(phast_data, adjacency, mismatch_node, arc_sample_limit);

    if (found_first_gap && first_gap_source != SPECIAL_NODEID && first_gap_source != mismatch_node)
    {
        util::Log() << "  local arc samples around first-gap source:";
        logArcSamplesForNode(phast_data, adjacency, first_gap_source, arc_sample_limit);
    }
    if (found_first_gap && first_gap_target != SPECIAL_NODEID && first_gap_target != mismatch_node)
    {
        util::Log() << "  local arc samples around first-gap target:";
        logArcSamplesForNode(phast_data, adjacency, first_gap_target, arc_sample_limit);
    }
}

void logDerivedArcSamples(const contractor::PhastData &phast_data,
                          const DerivedAdjacency &adjacency,
                          const std::size_t sample_limit)
{
    std::size_t logged_up = 0;
    std::size_t logged_down = 0;
    for (const auto source : util::irange<NodeID>(0, phast_data.node_count))
    {
        const auto source_rank = phast_data.ordering.rank[source];
        const auto up_begin = static_cast<std::size_t>(adjacency.up_offsets[source]);
        const auto up_end = static_cast<std::size_t>(adjacency.up_offsets[source + 1]);
        for (const auto index : util::irange<std::size_t>(up_begin, up_end))
        {
            if (logged_up >= sample_limit)
            {
                break;
            }
            const auto target = adjacency.up_targets[index];
            util::Log() << "Up sample " << source << " -> " << target
                        << " cost=" << adjacency.up_costs[index] << " rank=(" << source_rank << ","
                        << phast_data.ordering.rank[target] << ")";
            ++logged_up;
        }

        const auto down_begin = static_cast<std::size_t>(adjacency.down_offsets[source]);
        const auto down_end = static_cast<std::size_t>(adjacency.down_offsets[source + 1]);
        for (const auto index : util::irange<std::size_t>(down_begin, down_end))
        {
            if (logged_down >= sample_limit)
            {
                break;
            }
            const auto target = adjacency.down_targets[index];
            util::Log() << "Down sample " << source << " -> " << target
                        << " cost=" << adjacency.down_costs[index] << " rank=(" << source_rank << ","
                        << phast_data.ordering.rank[target] << ")";
            ++logged_down;
        }

        if (logged_up >= sample_limit && logged_down >= sample_limit)
        {
            break;
        }
    }
}
} // namespace

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();

    std::string verbosity;
    PhastConfig phast_config;
    RuntimeConfig runtime_config;

    const auto result = parseArguments(argc, argv, verbosity, phast_config, runtime_config);
    if (return_code::fail == result)
    {
        return EXIT_FAILURE;
    }
    if (return_code::exit == result)
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

    std::unordered_map<std::string, contractor::ContractedMetric> metrics = {{metric_name, {}}};
    std::uint32_t hsgr_checksum = 0;
    contractor::files::readGraph(phast_config.GetPath(".osrm.hsgr"), metrics, hsgr_checksum);

    contractor::PhastData phast_data;
    contractor::files::readPhast(phast_config.GetPath(".osrm.phast"), phast_data);

    const auto &metric = metrics.at(metric_name);
    const auto &graph = metric.graph;
    util::Log() << "CH metric: " << metric_name;
    util::Log() << "CH nodes: " << graph.GetNumberOfNodes() << ", edges: " << graph.GetNumberOfEdges();
    util::Log() << "PHAST version: " << phast_data.version;
    util::Log() << "PHAST metric: " << phast_data.metric_name;
    util::Log() << "PHAST nodes: " << phast_data.node_count;
    util::Log() << "PHAST exclude index: " << phast_data.exclude_index;
    util::Log() << "PHAST orientation: " << orientationToString(phast_data.orientation);
    util::Log() << "PHAST checksum: " << phast_data.connectivity_checksum;
    util::Log() << "HSGR checksum: " << hsgr_checksum;
    util::Log() << "PHAST order size: " << phast_data.ordering.order.size();
    util::Log() << "PHAST rank size: " << phast_data.ordering.rank.size();
    util::Log() << "PHAST c2o size: " << phast_data.ordering.contraction_to_original.size();
    util::Log() << "PHAST o2c size: " << phast_data.ordering.original_to_contraction.size();
    printArrayPreview("PHAST order preview", phast_data.ordering.order);
    printArrayPreview("PHAST rank preview", phast_data.ordering.rank);
    printArrayPreview("PHAST c2o preview", phast_data.ordering.contraction_to_original);
    printArrayPreview("PHAST o2c preview", phast_data.ordering.original_to_contraction);

    if (!validateMetadata(metric_name, metric, hsgr_checksum, phast_data))
    {
        return EXIT_FAILURE;
    }
    if (!validateOrdering(phast_data))
    {
        return EXIT_FAILURE;
    }

    std::shared_ptr<const CHDataFacade> facade;
    if (!loadCHFacade(phast_config.base_path, metric_name, phast_data.exclude_index, facade))
    {
        return EXIT_FAILURE;
    }
    if (!validateFacadeMetadata(*facade, phast_data, metric_name))
    {
        return EXIT_FAILURE;
    }

    const auto &selected_edge_filter = metric.edge_filter[phast_data.exclude_index];
    DerivedAdjacency adjacency;
    if (!deriveAdjacency(graph, selected_edge_filter, phast_data, adjacency))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Derived upward arcs: " << adjacency.up_targets.size();
    util::Log() << "Derived downward arcs: " << adjacency.down_targets.size();
    util::Log() << "Oriented candidate arcs: " << adjacency.oriented_arc_count;
    util::Log() << "Skipped self-loop arcs: " << adjacency.skipped_self_loops;
    if (runtime_config.debug_mismatch_limit > 0)
    {
        util::Log() << "Mismatch diagnostics enabled for up to "
                    << runtime_config.debug_mismatch_limit << " mismatches.";
        logDerivedArcSamples(phast_data, adjacency, runtime_config.debug_arc_sample_limit);
    }

    SeedMetricKind seed_metric_kind = SeedMetricKind::Weight;
    if (!parseSeedMetricKind(runtime_config, metric_name, seed_metric_kind))
    {
        return EXIT_FAILURE;
    }
    util::Log() << "Seed metric: "
                << (seed_metric_kind == SeedMetricKind::Weight ? "weight" : "duration");

    std::vector<PHASTSeed> seeds;
    SeedBuildStats seed_stats;
    if (runtime_config.has_poi_file)
    {
        if (!buildPOISeeds(
                runtime_config, phast_data.node_count, seed_metric_kind, *facade, seeds, seed_stats))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "POI file: " << runtime_config.poi_file.string();
        util::Log() << "POIs parsed: " << seed_stats.poi_count;
        util::Log() << "Snap cache hits: " << seed_stats.cache_hits;
        util::Log() << "Big-component snap fallbacks: " << seed_stats.big_component_fallbacks;
        util::Log() << "Seed classes: bidirectional=" << seed_stats.bidirectional_count
                    << " forward_only=" << seed_stats.forward_only_count
                    << " reverse_only=" << seed_stats.reverse_only_count;
        util::Log() << "Directional state seeds before dedup: " << seed_stats.directional_state_count;
        util::Log() << "Directional state seeds dropped by dedup: "
                    << seed_stats.dedup_dropped_count;
    }
    else
    {
        if (!prepareManualSeeds(runtime_config, phast_data.node_count, seeds))
        {
            return EXIT_FAILURE;
        }
        if (runtime_config.has_seed_file)
        {
            util::Log() << "Manual seed file: " << runtime_config.seed_file.string();
        }
        util::Log() << "Manual seeds loaded (cost 0).";
    }

    if (!validateSeeds(phast_data.node_count, seeds))
    {
        return EXIT_FAILURE;
    }
    printSeedPreview(seeds);
    util::Log() << "Seed count after dedup: " << seeds.size();

    std::optional<EdgeWeight> cap_weight;
    if (!parseCapWeight(runtime_config, properties, cap_weight))
    {
        return EXIT_FAILURE;
    }
    if (cap_weight.has_value())
    {
        util::Log() << "Traversal cap: " << runtime_config.cap_seconds << "s ("
                    << cap_weight.value() << " ticks)";
    }
    else
    {
        util::Log() << "Traversal cap: none";
    }

    std::vector<EdgeWeight> phast_distances;
    std::size_t upward_settled_nodes = 0;
    std::size_t downward_updates = 0;
    const auto phast_start = std::chrono::steady_clock::now();
    if (!runUpwardSearch(adjacency, seeds, cap_weight, phast_distances, upward_settled_nodes))
    {
        return EXIT_FAILURE;
    }
    if (!runDownwardSweep(phast_data, adjacency, cap_weight, phast_distances, &downward_updates))
    {
        return EXIT_FAILURE;
    }
    const auto phast_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               phast_start)
            .count();

    std::vector<EdgeWeight> reference_distances;
    DistanceTrace reference_trace;
    std::size_t dijkstra_settled_nodes = 0;
    const auto reference_start = std::chrono::steady_clock::now();
    if (!runReferenceDijkstra(adjacency,
                              seeds,
                              cap_weight,
                              reference_distances,
                              dijkstra_settled_nodes,
                              reference_trace))
    {
        return EXIT_FAILURE;
    }
    const auto reference_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               reference_start)
            .count();

    util::Log() << "Upward search settled nodes: " << upward_settled_nodes;
    util::Log() << "Downward sweep updates: " << downward_updates;
    util::Log() << "Reference Dijkstra settled nodes: " << dijkstra_settled_nodes;
    util::Log() << "PHAST reachable nodes: " << countReachable(phast_distances)
                << ", max distance: " << maxDistance(phast_distances) << " ticks";
    util::Log() << "Reference reachable nodes: " << countReachable(reference_distances)
                << ", max distance: " << maxDistance(reference_distances) << " ticks";
    util::Log() << "PHAST runtime: " << phast_elapsed << " ms";
    util::Log() << "Reference runtime: " << reference_elapsed << " ms";

    const auto validation = validateDistances(phast_data,
                                              phast_distances,
                                              reference_distances,
                                              runtime_config.debug_mismatch_limit);
    if (validation.mismatch_count > 0)
    {
        if (runtime_config.debug_mismatch_limit > 0)
        {
            for (const auto node : validation.mismatch_nodes)
            {
                logMismatchDiagnostics(phast_data,
                                       adjacency,
                                       phast_distances,
                                       reference_distances,
                                       reference_trace,
                                       cap_weight,
                                       node,
                                       runtime_config.debug_arc_sample_limit);
            }
        }
        return EXIT_FAILURE;
    }
    util::Log() << "PHAST prototype matches Dijkstra baseline.";

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
