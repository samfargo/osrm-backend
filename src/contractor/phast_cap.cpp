#include "contractor/phast_cap.hpp"

#include "util/log.hpp"

#include <cmath>

namespace osrm::contractor::phast
{

namespace
{

bool ConvertCapTicks(const std::int64_t ticks, const char *option_name, EdgeWeight &cap_metric)
{
    const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
    if (ticks < 0 || ticks > max_valid)
    {
        util::Log(logERROR) << option_name << " is out of range.";
        return false;
    }
    cap_metric = to_alias<EdgeWeight>(ticks);
    return true;
}

} // namespace

bool ParseMetricKind(const std::string &metric_name,
                     const std::string &weight_name,
                     const char *option_name,
                     MetricKind &metric_kind)
{
    if (metric_name == "weight")
    {
        metric_kind = MetricKind::Weight;
        return true;
    }
    if (metric_name == "duration")
    {
        if (weight_name != "duration")
        {
            util::Log(logERROR) << option_name
                                << "=duration requires a duration-weighted dataset.";
            return false;
        }
        metric_kind = MetricKind::Duration;
        return true;
    }

    util::Log(logERROR) << option_name << " must be 'weight' or 'duration'.";
    return false;
}

bool ParseTraversalCap(const RuntimeConfig &runtime_config,
                       const extractor::ProfileProperties &properties,
                       MetricKind metric_kind,
                       std::optional<EdgeWeight> &cap_metric)
{
    cap_metric = std::nullopt;

    if (metric_kind == MetricKind::Duration)
    {
        if (!runtime_config.has_cap_seconds)
        {
            util::Log(logERROR) << "--metric=duration requires --cap-seconds.";
            return false;
        }

        const auto ticks = static_cast<std::int64_t>(std::ceil(runtime_config.cap_seconds * 10.));
        EdgeWeight converted = INVALID_EDGE_WEIGHT;
        if (!ConvertCapTicks(ticks, "--cap-seconds", converted))
        {
            return false;
        }
        cap_metric = converted;
        return true;
    }

    if (runtime_config.has_cap_seconds)
    {
        if (properties.GetWeightName() != "duration")
        {
            util::Log(logERROR)
                << "--cap-seconds requires weight_name=duration when --metric=weight.";
            return false;
        }

        const auto ticks = static_cast<std::int64_t>(
            std::ceil(runtime_config.cap_seconds * properties.GetWeightMultiplier()));
        EdgeWeight converted = INVALID_EDGE_WEIGHT;
        if (!ConvertCapTicks(ticks, "--cap-seconds", converted))
        {
            return false;
        }
        cap_metric = converted;
        return true;
    }

    if (runtime_config.has_cap_weight)
    {
        const auto ticks = static_cast<std::int64_t>(runtime_config.cap_weight);
        EdgeWeight converted = INVALID_EDGE_WEIGHT;
        if (!ConvertCapTicks(ticks, "--cap-weight", converted))
        {
            return false;
        }
        cap_metric = converted;
        return true;
    }

    if (properties.GetWeightName() != "duration")
    {
        util::Log(logERROR)
            << "--metric=weight requires --cap-weight for non-duration-weighted datasets.";
        return false;
    }

    return true;
}

} // namespace osrm::contractor::phast
