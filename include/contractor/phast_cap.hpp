#ifndef OSRM_CONTRACTOR_PHAST_CAP_HPP
#define OSRM_CONTRACTOR_PHAST_CAP_HPP

#include "contractor/phast_common.hpp"

#include "extractor/profile_properties.hpp"

#include <optional>
#include <string>

namespace osrm::contractor::phast
{

bool ParseMetricKind(const std::string &metric_name,
                     const std::string &weight_name,
                     const char *option_name,
                     MetricKind &metric_kind);

bool ParseTraversalCap(const RuntimeConfig &runtime_config,
                       const extractor::ProfileProperties &properties,
                       MetricKind metric_kind,
                       std::optional<EdgeWeight> &cap_metric);

} // namespace osrm::contractor::phast

#endif
