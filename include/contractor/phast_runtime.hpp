#ifndef OSRM_CONTRACTOR_PHAST_RUNTIME_HPP
#define OSRM_CONTRACTOR_PHAST_RUNTIME_HPP

#include "contractor/contracted_metric.hpp"
#include "contractor/phast_common.hpp"
#include "contractor/phast_data.hpp"
#include "contractor/query_graph.hpp"

#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace osrm::contractor::phast
{

using CHDataFacade =
    engine::datafacade::ContiguousInternalMemoryDataFacade<engine::routing_algorithms::ch::Algorithm>;

struct DerivedAdjacency
{
    std::vector<std::uint32_t> up_offsets;
    std::vector<NodeID> up_targets;
    std::vector<EdgeWeight> up_costs;
    std::vector<std::uint32_t> down_offsets;
    std::vector<NodeID> down_targets;
    std::vector<EdgeWeight> down_costs;
    std::size_t oriented_arc_count = 0;
    std::size_t skipped_self_loops = 0;
    std::size_t pre_synthesis_upward_arc_count = 0;
    std::size_t pre_synthesis_downward_arc_count = 0;
    bool downward_built_from_transpose = false;
};

const char *OrientationToString(contractor::PHASTOrientation orientation);

bool ParseOrientationString(const std::string &value, contractor::PHASTOrientation &orientation);

bool ValidateOrdering(const contractor::PhastData &phast_data);

bool ValidateMetadata(const std::string &metric_name,
                      const contractor::ContractedMetric &metric,
                      std::uint32_t hsgr_checksum,
                      const contractor::PhastData &phast_data);

bool ResolveExcludeIndex(const RuntimeConfig &runtime_config,
                         const contractor::PhastData &phast_data,
                         std::size_t &exclude_index);

bool ResolveOrientation(const RuntimeConfig &runtime_config,
                        const contractor::PhastData &phast_data,
                        contractor::PHASTOrientation &orientation);

bool LoadCHFacade(const std::filesystem::path &base_path,
                  const std::string &metric_name,
                  std::size_t exclude_index,
                  std::shared_ptr<const CHDataFacade> &facade,
                  bool use_mmap = false);

bool ValidateFacadeMetadata(const CHDataFacade &facade,
                            const contractor::PhastData &phast_data,
                            const std::string &metric_name);

bool DeriveAdjacency(const contractor::QueryGraph &graph,
                     const std::vector<bool> &edge_filter,
                     const contractor::PhastData &phast_data,
                     contractor::PHASTOrientation orientation,
                     MetricKind metric_kind,
                     DerivedAdjacency &adjacency);

bool RunUpwardSearch(const DerivedAdjacency &adjacency,
                     const std::vector<PHASTSeed> &seeds,
                     const std::optional<EdgeWeight> &cap_weight,
                     std::vector<EdgeWeight> &distances,
                     std::size_t &settled_nodes);

bool RunDownwardSweep(const contractor::PhastData &phast_data,
                      const DerivedAdjacency &adjacency,
                      const std::optional<EdgeWeight> &cap_weight,
                      std::vector<EdgeWeight> &distances,
                      std::size_t *update_count = nullptr);

std::size_t CountReachable(const std::vector<EdgeWeight> &distances);

EdgeWeight MaxDistance(const std::vector<EdgeWeight> &distances);

} // namespace osrm::contractor::phast

#endif
