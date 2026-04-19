#include "contractor/contractor.hpp"
#include "contractor/contracted_edge_container.hpp"
#include "contractor/files.hpp"
#include "contractor/graph_contractor.hpp"
#include "contractor/graph_contractor_adaptors.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/edge_based_graph_factory.hpp"
#include "extractor/files.hpp"
#include "extractor/node_based_edge.hpp"

#include "storage/io.hpp"

#include "updater/updater.hpp"

#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/exclude_flag.hpp"
#include "util/filtered_graph.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/static_graph.hpp"
#include "util/string_util.hpp"
#include "util/timing_util.hpp"
#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <vector>

#include <tbb/global_control.h>

namespace osrm::contractor
{

int Contractor::Run()
{
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config.requested_num_threads);

    TIMER_START(preparing);

    util::Log() << "Reading node weights.";
    std::vector<EdgeWeight> node_weights;
    extractor::files::readEdgeBasedNodeWeights(config.GetPath(".osrm.enw"), node_weights);
    util::Log() << "Done reading node weights.";

    util::Log() << "Loading edge-expanded graph representation";

    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;

    updater::Updater updater(config.updater_config);
    std::uint32_t connectivity_checksum = 0;
    EdgeID number_of_edge_based_nodes = updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, connectivity_checksum);

    // Convert node weights for oneway streets to INVALID_EDGE_WEIGHT
    for (auto &weight : node_weights)
    {
        weight = (from_alias<EdgeWeight::value_type>(weight) & 0x80000000) ? INVALID_EDGE_WEIGHT
                                                                           : weight;
    }

    // Contracting the edge-expanded graph

    TIMER_START(contraction);

    std::string metric_name;
    std::vector<std::vector<bool>> node_filters;
    {
        extractor::EdgeBasedNodeDataContainer node_data;
        extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data);

        extractor::ProfileProperties properties;
        extractor::files::readProfileProperties(config.GetPath(".osrm.properties"), properties);
        metric_name = properties.GetWeightName();

        node_filters =
            util::excludeFlagsToNodeFilter(number_of_edge_based_nodes, node_data, properties);
    }

    QueryGraph query_graph;
    std::vector<std::vector<bool>> edge_filters;
    PHASTOrdering ordering;
    const std::size_t phast_exclude_index = 0;
    std::tie(query_graph, edge_filters) =
        contractExcludableGraph(toContractorGraph(number_of_edge_based_nodes, edge_based_edge_list),
                                std::move(node_weights),
                                node_filters,
                                &ordering,
                                phast_exclude_index);
    TIMER_STOP(contraction);
    util::Log() << "Contracted graph has " << query_graph.GetNumberOfEdges() << " edges.";
    util::Log() << "Contraction took " << TIMER_SEC(contraction) << " sec";
    const auto node_count = query_graph.GetNumberOfNodes();

    std::unordered_map<std::string, ContractedMetric> metrics = {
        {metric_name, {std::move(query_graph), std::move(edge_filters)}}};

    files::writeGraph(config.GetPath(".osrm.hsgr"), metrics, connectivity_checksum);

    if (ordering.order.size() != node_count)
    {
        throw util::exception("Invalid PHAST ordering: order size mismatch.");
    }
    if (ordering.rank.size() != node_count)
    {
        throw util::exception("Invalid PHAST ordering: rank size mismatch.");
    }
    if (ordering.contraction_to_original.size() != node_count ||
        ordering.original_to_contraction.size() != node_count)
    {
        throw util::exception("Invalid PHAST ordering: permutation size mismatch.");
    }

    for (const auto i : util::irange<std::size_t>(0, node_count))
    {
        if (ordering.rank[ordering.order[i]] != i)
        {
            throw util::exception("Invalid PHAST ordering: rank/order inverse mismatch.");
        }
    }
    for (const auto original_id : util::irange<NodeID>(0, node_count))
    {
        const auto local_id = ordering.original_to_contraction[original_id];
        if (local_id >= node_count || ordering.contraction_to_original[local_id] != original_id)
        {
            throw util::exception("Invalid PHAST ordering: permutation inverse mismatch.");
        }
    }

    PhastData phast_data;
    phast_data.version = 3;
    phast_data.connectivity_checksum = connectivity_checksum;
    phast_data.node_count = node_count;
    phast_data.exclude_index = static_cast<std::uint32_t>(phast_exclude_index);
    phast_data.metric_name = metric_name;
    phast_data.orientation = PHASTOrientation::Reverse;
    phast_data.ordering = std::move(ordering);
    files::writePhast(config.GetPath(".osrm.phast"), phast_data);

    TIMER_STOP(preparing);

    util::Log() << "Preprocessing : " << TIMER_SEC(preparing) << " seconds";

    util::Log() << "finished preprocessing";

    return 0;
}

} // namespace osrm::contractor
