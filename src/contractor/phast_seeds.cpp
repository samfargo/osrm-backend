#include "contractor/phast_seeds.hpp"

#include "engine/approach.hpp"
#include "engine/hint.hpp"

#include "util/integer_range.hpp"
#include "util/log.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <unordered_map>

namespace osrm::contractor::phast
{

namespace
{

struct POIRecord
{
    util::Coordinate coordinate;
    std::uint64_t line_number = 0;
};

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

bool LoadManualSeedFile(const std::filesystem::path &seed_file, std::vector<NodeID> &seed_nodes)
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
            util::Log(logERROR) << "Invalid seed value in " << seed_file.string() << ":" << line_number;
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
            util::Log(logERROR) << "Seed id out of range in " << seed_file.string() << ":" << line_number;
            return false;
        }
        seed_nodes.push_back(static_cast<NodeID>(node_id));
    }

    return true;
}

bool BuildManualSeeds(const RuntimeConfig &runtime_config,
                      std::uint32_t node_count,
                      std::vector<PHASTSeed> &seeds)
{
    std::vector<NodeID> manual_seed_nodes = runtime_config.seed_nodes;
    if (runtime_config.has_seed_file && !LoadManualSeedFile(runtime_config.seed_file, manual_seed_nodes))
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
            util::Log(logERROR) << "Manual seed node id out of range: " << seed_node << " >= " << node_count;
            return false;
        }
        seeds.push_back({seed_node, EdgeWeight{0}, SeedClass::Manual});
    }

    return true;
}

bool ParsePOILine(const std::string &line,
                  const std::filesystem::path &poi_file,
                  std::uint64_t line_number,
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
        util::Log(logERROR) << "Invalid POI coordinate in " << poi_file.string() << ":" << line_number;
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
        util::Log(logERROR) << "Non-finite POI coordinate in " << poi_file.string() << ":" << line_number;
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
        util::Log(logERROR) << "POI coordinate invalid in " << poi_file.string() << ":" << line_number;
        return false;
    }

    record = POIRecord{coordinate, line_number};
    return true;
}

bool LoadPOIFile(const std::filesystem::path &poi_file, std::vector<POIRecord> &pois)
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
        if (!ParsePOILine(line, poi_file, line_number, record))
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

bool ConvertSeedCost(const engine::PhantomNode &phantom,
                     bool forward_state,
                     MetricKind seed_metric_kind,
                     EdgeWeight &seed_cost)
{
    if (seed_metric_kind == MetricKind::Duration)
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

SeedClass ClassifyTargetSeed(bool forward_valid, bool reverse_valid)
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

bool GetOrientationSeedValidity(const engine::PhantomNode &phantom,
                                contractor::PHASTOrientation orientation,
                                bool &forward_valid,
                                bool &reverse_valid)
{
    switch (orientation)
    {
    case contractor::PHASTOrientation::Forward:
        forward_valid = phantom.IsValidForwardSource();
        reverse_valid = phantom.IsValidReverseSource();
        return true;
    case contractor::PHASTOrientation::Reverse:
        forward_valid = phantom.IsValidForwardTarget();
        reverse_valid = phantom.IsValidReverseTarget();
        return true;
    }
    return false;
}

bool BuildPOISeeds(const RuntimeConfig &runtime_config,
                   std::uint32_t node_count,
                   MetricKind seed_metric_kind,
                   contractor::PHASTOrientation orientation,
                   const CHDataFacade &facade,
                   std::vector<PHASTSeed> &seeds,
                   SeedBuildStats &stats)
{
    std::vector<POIRecord> pois;
    if (!LoadPOIFile(runtime_config.poi_file, pois))
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
        if (cache_iterator != snap_cache.end() && cache_iterator->second.IsValid(poi.coordinate, facade))
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
                const auto first_non_tiny = std::find_if(
                    alternatives.first.begin(),
                    alternatives.first.end(),
                    [](const engine::PhantomNode &candidate) {
                        return !candidate.component.is_tiny;
                    });
                if (first_non_tiny != alternatives.first.end())
                {
                    snapped_phantom = *first_non_tiny;
                }
                else if (!alternatives.second.empty())
                {
                    snapped_phantom = alternatives.second.front();
                    ++stats.big_component_fallbacks;
                }
                else
                {
                    snapped_phantom = alternatives.first.front();
                }
            }
            else if (!alternatives.second.empty())
            {
                snapped_phantom = alternatives.second.front();
                ++stats.big_component_fallbacks;
            }
            else
            {
                util::Log(logERROR) << "No phantom candidate found for POI at line " << poi.line_number;
                return false;
            }

            snap_cache[key] = engine::SegmentHint{snapped_phantom, facade.GetCheckSum()};
        }

        if (!snapped_phantom.IsValid(node_count, poi.coordinate))
        {
            util::Log(logERROR) << "Invalid snapped phantom for POI at line " << poi.line_number;
            return false;
        }

        bool forward_valid = false;
        bool reverse_valid = false;
        if (!GetOrientationSeedValidity(snapped_phantom, orientation, forward_valid, reverse_valid))
        {
            util::Log(logERROR) << "Unsupported orientation while building POI seeds.";
            return false;
        }
        if (!forward_valid && !reverse_valid)
        {
            util::Log(logERROR)
                << "Snapped phantom has no valid states for selected orientation at POI line "
                << poi.line_number;
            return false;
        }

        const auto seed_class = ClassifyTargetSeed(forward_valid, reverse_valid);
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

        const auto push_seed = [&](NodeID node, EdgeWeight cost)
        {
            if (node >= node_count)
            {
                util::Log(logERROR) << "Snapped seed node out of range: " << node << " >= " << node_count;
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
            if (!ConvertSeedCost(snapped_phantom, true, seed_metric_kind, forward_cost))
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
            if (!ConvertSeedCost(snapped_phantom, false, seed_metric_kind, reverse_cost))
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

} // namespace

bool BuildSeeds(const RuntimeConfig &runtime_config,
                std::uint32_t node_count,
                MetricKind seed_metric_kind,
                contractor::PHASTOrientation orientation,
                const CHDataFacade &facade,
                std::vector<PHASTSeed> &seeds,
                SeedBuildStats &stats)
{
    stats = {};
    if (runtime_config.has_poi_file)
    {
        return BuildPOISeeds(
            runtime_config, node_count, seed_metric_kind, orientation, facade, seeds, stats);
    }

    return BuildManualSeeds(runtime_config, node_count, seeds);
}

bool ValidateSeeds(std::uint32_t node_count, const std::vector<PHASTSeed> &seeds)
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

} // namespace osrm::contractor::phast
