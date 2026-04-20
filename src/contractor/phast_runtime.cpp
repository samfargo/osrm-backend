#include "contractor/phast_runtime.hpp"

#include "engine/datafacade/process_memory_allocator.hpp"

#include "storage/storage_config.hpp"

#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/query_heap.hpp"

#include <algorithm>
#include <cstdint>

namespace osrm::contractor::phast
{

namespace
{

using DistanceHeap = util::QueryHeap<NodeID, NodeID, EdgeWeight, NodeID>;

bool IsValidOrientation(contractor::PHASTOrientation orientation)
{
    return orientation == contractor::PHASTOrientation::Forward ||
           orientation == contractor::PHASTOrientation::Reverse;
}

bool ClassifyArc(const contractor::PhastData &phast_data,
                 NodeID source,
                 NodeID target,
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

bool OrientArc(const contractor::QueryEdge::EdgeData &edge_data,
               contractor::PHASTOrientation orientation,
               NodeID source,
               NodeID target,
               NodeID &oriented_source,
               NodeID &oriented_target)
{
    switch (orientation)
    {
    case contractor::PHASTOrientation::Forward:
        if (!edge_data.forward)
        {
            return false;
        }
        oriented_source = source;
        oriented_target = target;
        return true;
    case contractor::PHASTOrientation::Reverse:
        if (!edge_data.backward)
        {
            return false;
        }
        oriented_source = source;
        oriented_target = target;
        return true;
    }
    return false;
}

bool ExtractArcCost(const contractor::QueryEdge::EdgeData &edge_data,
                    MetricKind metric_kind,
                    EdgeWeight &edge_cost)
{
    if (metric_kind == MetricKind::Duration)
    {
        const auto duration_value = static_cast<std::int64_t>(edge_data.duration);
        const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
        if (duration_value <= 0 || duration_value > max_valid)
        {
            return false;
        }
        edge_cost = to_alias<EdgeWeight>(duration_value);
        return true;
    }

    edge_cost = edge_data.weight;
    return edge_cost > EdgeWeight{0} && edge_cost != INVALID_EDGE_WEIGHT;
}

struct DerivedArc
{
    NodeID source;
    NodeID target;
    EdgeWeight cost;
};

void BuildCSR(const std::vector<DerivedArc> &arcs,
              std::uint32_t node_count,
              std::vector<std::uint32_t> &offsets,
              std::vector<NodeID> &targets,
              std::vector<EdgeWeight> &costs)
{
    offsets.assign(node_count + 1, 0);
    for (const auto &arc : arcs)
    {
        ++offsets[arc.source + 1];
    }

    for (const auto index : util::irange<std::size_t>(1, offsets.size()))
    {
        offsets[index] += offsets[index - 1];
    }

    targets.resize(offsets.back());
    costs.resize(offsets.back());

    auto cursor = offsets;
    for (const auto &arc : arcs)
    {
        const auto insertion_index = cursor[arc.source]++;
        targets[insertion_index] = arc.target;
        costs[insertion_index] = arc.cost;
    }
}

bool TryAddWeights(EdgeWeight lhs, EdgeWeight rhs, EdgeWeight &result)
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

bool ExceedsCap(EdgeWeight distance, const std::optional<EdgeWeight> &cap_weight)
{
    return cap_weight.has_value() && distance > cap_weight.value();
}

bool RelaxWithHeap(const NodeID source,
                   const std::vector<std::uint32_t> &offsets,
                   const std::vector<NodeID> &targets,
                   const std::vector<EdgeWeight> &costs,
                   const std::optional<EdgeWeight> &cap_weight,
                   std::vector<EdgeWeight> &distances,
                   DistanceHeap &heap,
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
        if (!TryAddWeights(source_distance, edge_weight, candidate))
        {
            util::Log(logERROR) << "Distance overflow while relaxing " << source << " -> " << target;
            return false;
        }
        if (ExceedsCap(candidate, cap_weight))
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

bool RelaxWithoutHeap(const NodeID source,
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
    if (ExceedsCap(source_distance, cap_weight))
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
        if (!TryAddWeights(source_distance, edge_weight, candidate))
        {
            util::Log(logERROR) << "Distance overflow while sweeping " << source << " -> " << target;
            return false;
        }
        if (ExceedsCap(candidate, cap_weight))
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

} // namespace

const char *OrientationToString(contractor::PHASTOrientation orientation)
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

bool ParseOrientationString(const std::string &value, contractor::PHASTOrientation &orientation)
{
    if (value == "forward")
    {
        orientation = contractor::PHASTOrientation::Forward;
        return true;
    }
    if (value == "reverse")
    {
        orientation = contractor::PHASTOrientation::Reverse;
        return true;
    }
    return false;
}

bool ValidateOrdering(const contractor::PhastData &phast_data)
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

bool ValidateMetadata(const std::string &metric_name,
                      const contractor::ContractedMetric &metric,
                      std::uint32_t hsgr_checksum,
                      const contractor::PhastData &phast_data)
{
    if (phast_data.version < contractor::PHAST_SCHEMA_VERSION)
    {
        util::Log(logERROR) << "Unsupported .osrm.phast version " << phast_data.version
                            << ". Expected version >= " << contractor::PHAST_SCHEMA_VERSION << ".";
        return false;
    }
    if (!IsValidOrientation(phast_data.orientation))
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

bool ResolveExcludeIndex(const RuntimeConfig &runtime_config,
                         const contractor::PhastData &phast_data,
                         std::size_t &exclude_index)
{
    exclude_index = phast_data.exclude_index;
    if (!runtime_config.has_exclude_index)
    {
        return true;
    }
    if (runtime_config.exclude_index != phast_data.exclude_index)
    {
        util::Log(logERROR) << "--exclude=" << runtime_config.exclude_index
                            << " does not match .osrm.phast exclude index "
                            << phast_data.exclude_index << ".";
        return false;
    }

    return true;
}

bool ResolveOrientation(const RuntimeConfig &runtime_config,
                        const contractor::PhastData &phast_data,
                        contractor::PHASTOrientation &orientation)
{
    orientation = phast_data.orientation;
    if (!runtime_config.has_orientation)
    {
        return true;
    }

    contractor::PHASTOrientation requested_orientation;
    if (!ParseOrientationString(runtime_config.orientation, requested_orientation))
    {
        util::Log(logERROR) << "Invalid orientation: " << runtime_config.orientation;
        return false;
    }
    if (requested_orientation != phast_data.orientation)
    {
        util::Log(logERROR) << "--orientation=" << runtime_config.orientation
                            << " does not match .osrm.phast orientation "
                            << OrientationToString(phast_data.orientation) << ".";
        return false;
    }

    orientation = requested_orientation;
    return true;
}

bool LoadCHFacade(const std::filesystem::path &base_path,
                  const std::string &metric_name,
                  std::size_t exclude_index,
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
        auto allocator = std::make_shared<engine::datafacade::ProcessMemoryAllocator>(storage_config);
        facade = std::make_shared<const CHDataFacade>(allocator, metric_name, exclude_index);
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to initialize CH facade for snapping: " << e.what();
        return false;
    }

    return true;
}

bool ValidateFacadeMetadata(const CHDataFacade &facade,
                            const contractor::PhastData &phast_data,
                            const std::string &metric_name)
{
    if (facade.GetCheckSum() != phast_data.connectivity_checksum)
    {
        util::Log(logERROR) << "Facade checksum (" << facade.GetCheckSum()
                            << ") differs from PHAST connectivity checksum ("
                            << phast_data.connectivity_checksum << ").";
        return false;
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

bool DeriveAdjacency(const contractor::QueryGraph &graph,
                     const std::vector<bool> &edge_filter,
                     const contractor::PhastData &phast_data,
                     contractor::PHASTOrientation orientation,
                     MetricKind metric_kind,
                     DerivedAdjacency &adjacency)
{
    const auto node_count = phast_data.node_count;
    std::vector<DerivedArc> upward_arcs;
    std::vector<DerivedArc> downward_arcs;
    upward_arcs.reserve(graph.GetNumberOfEdges());
    downward_arcs.reserve(graph.GetNumberOfEdges());

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
            NodeID oriented_source = SPECIAL_NODEID;
            NodeID oriented_target = SPECIAL_NODEID;
            if (!OrientArc(edge_data, orientation, source, target, oriented_source, oriented_target))
            {
                continue;
            }
            if (oriented_source == oriented_target)
            {
                ++adjacency.skipped_self_loops;
                continue;
            }

            EdgeWeight edge_weight = INVALID_EDGE_WEIGHT;
            if (!ExtractArcCost(edge_data, metric_kind, edge_weight))
            {
                util::Log(logERROR) << "Encountered invalid arc cost on edge " << source << " -> "
                                    << target;
                return false;
            }

            ++adjacency.oriented_arc_count;

            bool is_upward = false;
            if (!ClassifyArc(phast_data, oriented_source, oriented_target, is_upward))
            {
                return false;
            }

            const auto arc = DerivedArc{oriented_source, oriented_target, edge_weight};
            if (is_upward)
            {
                upward_arcs.push_back(arc);
            }
            else
            {
                downward_arcs.push_back(arc);
            }
        }
    }

    if (adjacency.oriented_arc_count == 0)
    {
        util::Log(logERROR) << "No oriented runtime arcs were derived from the CH graph.";
        return false;
    }

    if (upward_arcs.empty() && !downward_arcs.empty())
    {
        upward_arcs.reserve(downward_arcs.size());
        for (const auto &arc : downward_arcs)
        {
            upward_arcs.push_back({arc.target, arc.source, arc.cost});
        }
    }
    if (downward_arcs.empty() && !upward_arcs.empty())
    {
        downward_arcs.reserve(upward_arcs.size());
        for (const auto &arc : upward_arcs)
        {
            downward_arcs.push_back({arc.target, arc.source, arc.cost});
        }
    }

    if (upward_arcs.empty() || downward_arcs.empty())
    {
        util::Log(logERROR) << "Could not derive both upward and downward adjacency sets.";
        return false;
    }

    BuildCSR(upward_arcs, node_count, adjacency.up_offsets, adjacency.up_targets, adjacency.up_costs);
    BuildCSR(
        downward_arcs, node_count, adjacency.down_offsets, adjacency.down_targets, adjacency.down_costs);

    return true;
}

bool RunUpwardSearch(const DerivedAdjacency &adjacency,
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
            util::Log(logERROR) << "Seed node id out of range: " << seed.node << " >= " << node_count;
            return false;
        }
        if (seed.cost == INVALID_EDGE_WEIGHT || ExceedsCap(seed.cost, cap_weight))
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
        if (!RelaxWithHeap(heap_node.node,
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

bool RunDownwardSweep(const contractor::PhastData &phast_data,
                      const DerivedAdjacency &adjacency,
                      const std::optional<EdgeWeight> &cap_weight,
                      std::vector<EdgeWeight> &distances,
                      std::size_t *update_count)
{
    const auto node_count = phast_data.node_count;
    for (auto sweep_pos = node_count; sweep_pos > 0; --sweep_pos)
    {
        const auto source = phast_data.ordering.order[sweep_pos - 1];
        if (!RelaxWithoutHeap(source,
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

std::size_t CountReachable(const std::vector<EdgeWeight> &distances)
{
    return static_cast<std::size_t>(
        std::count_if(distances.begin(),
                      distances.end(),
                      [](const auto distance) { return distance != INVALID_EDGE_WEIGHT; }));
}

EdgeWeight MaxDistance(const std::vector<EdgeWeight> &distances)
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

} // namespace osrm::contractor::phast
