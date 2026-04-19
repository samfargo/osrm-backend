#ifndef OSRM_CONTRACTOR_GRAPH_CONTRACTOR_HPP
#define OSRM_CONTRACTOR_GRAPH_CONTRACTOR_HPP

#include "contractor/contractor_graph.hpp"
#include "contractor/phast_data.hpp"
#include "contractor/query_graph.hpp"

#include "util/filtered_graph.hpp"

#include <vector>

namespace osrm::contractor
{

using GraphAndFilter = std::tuple<QueryGraph, std::vector<std::vector<bool>>>;

GraphAndFilter contractFullGraph(ContractorGraph contractor_graph,
                                 std::vector<EdgeWeight> node_weights,
                                 PHASTOrdering *ordering_out = nullptr);

GraphAndFilter contractExcludableGraph(ContractorGraph contractor_graph_,
                                       std::vector<EdgeWeight> node_weights,
                                       const std::vector<std::vector<bool>> &filters,
                                       PHASTOrdering *ordering_out = nullptr,
                                       std::size_t ordering_filter_index = 0);

std::vector<bool> contractGraph(ContractorGraph &graph,
                                std::vector<bool> node_is_uncontracted,
                                std::vector<bool> node_is_contractable,
                                std::vector<EdgeWeight> node_weights,
                                double core_factor = 1.0,
                                PHASTOrdering *ordering_out = nullptr);

// Overload for contracting all nodes
inline auto contractGraph(ContractorGraph &graph,
                          std::vector<EdgeWeight> node_weights,
                          double core_factor = 1.0,
                          PHASTOrdering *ordering_out = nullptr)
{
    return contractGraph(graph, {}, {}, std::move(node_weights), core_factor, ordering_out);
}

// Overload no contracted nodes
inline auto contractGraph(ContractorGraph &graph,
                          std::vector<bool> node_is_contractable,
                          std::vector<EdgeWeight> node_weights,
                          double core_factor = 1.0,
                          PHASTOrdering *ordering_out = nullptr)
{
    return contractGraph(
        graph, {}, std::move(node_is_contractable), std::move(node_weights), core_factor, ordering_out);
}

} // namespace osrm::contractor

#endif // OSRM_CONTRACTOR_GRAPH_CONTRACTOR_HPP
