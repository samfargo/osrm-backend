#ifndef OSRM_CONTRACTOR_PHAST_COMMON_HPP
#define OSRM_CONTRACTOR_PHAST_COMMON_HPP

#include "storage/io_config.hpp"
#include "util/typedefs.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace osrm::contractor::phast
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
    std::string metric = "weight";
    std::string seed_metric;
    bool has_seed_metric = false;
    double cap_seconds = 0.;
    bool has_cap_seconds = false;
    std::uint64_t cap_weight = 0;
    bool has_cap_weight = false;
    std::size_t exclude_index = std::numeric_limits<std::size_t>::max();
    bool has_exclude_index = false;
    std::string orientation;
    bool has_orientation = false;
    std::string output_format = "phastfield-v1";
    std::filesystem::path output_path;
    bool has_output_path = false;
    std::filesystem::path sample_file;
    bool has_sample_file = false;
    std::filesystem::path sample_snap_cache;
    bool has_sample_snap_cache = false;
    std::filesystem::path raster_output_path;
    bool has_raster_output_path = false;
    std::uint32_t expected_resolution = 9;
};

enum class ReturnCode : unsigned
{
    Ok,
    Fail,
    Exit
};

enum class MetricKind : std::uint8_t
{
    Weight = 0,
    Duration = 1
};

enum class OutputFormatKind : std::uint8_t
{
    PhastFieldV1 = 0,
    RawU32 = 1
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

} // namespace osrm::contractor::phast

#endif
