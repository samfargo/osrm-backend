#ifndef OSRM_CONTRACTOR_PHAST_SEEDS_HPP
#define OSRM_CONTRACTOR_PHAST_SEEDS_HPP

#include "contractor/phast_common.hpp"
#include "contractor/phast_runtime.hpp"

#include "contractor/phast_data.hpp"

#include <vector>

namespace osrm::contractor::phast
{

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

bool BuildSeeds(const RuntimeConfig &runtime_config,
                std::uint32_t node_count,
                MetricKind seed_metric_kind,
                contractor::PHASTOrientation orientation,
                const CHDataFacade &facade,
                std::vector<PHASTSeed> &seeds,
                SeedBuildStats &stats);

bool ValidateSeeds(std::uint32_t node_count, const std::vector<PHASTSeed> &seeds);

} // namespace osrm::contractor::phast

#endif
