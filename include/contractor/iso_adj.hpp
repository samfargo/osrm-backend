#ifndef OSRM_CONTRACTOR_ISO_ADJ_HPP
#define OSRM_CONTRACTOR_ISO_ADJ_HPP

#include "contractor/phast_common.hpp"
#include "contractor/phast_data.hpp"

#include <boost/iostreams/device/mapped_file.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace osrm::contractor::phast
{

inline constexpr std::array<char, 8> ISO_ADJ_MAGIC = {'I', 'S', 'O', 'A', 'D', 'J', '2', '\0'};
inline constexpr std::uint32_t ISO_ADJ_VERSION = 2;
inline constexpr std::size_t ISO_ADJ_BUILD_ID_BYTES = 20;
inline constexpr std::size_t ISO_ADJ_METRIC_NAME_BYTES = 256;

struct IsoAdjFileHeader
{
    std::array<char, 8> magic = ISO_ADJ_MAGIC;
    std::uint32_t format_version = ISO_ADJ_VERSION;
    std::uint32_t header_bytes = 0;
    std::uint32_t phast_schema_version = 0;
    std::uint32_t metric_kind = 0;
    std::uint32_t orientation = 0;
    std::uint32_t exclude_index = 0;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t reserved = 0;
    std::uint64_t node_count = 0;
    std::uint64_t edge_count = 0;
    std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES> build_id{};
    std::uint32_t metric_name_size = 0;
    std::array<char, ISO_ADJ_METRIC_NAME_BYTES> metric_name{};
};
inline constexpr std::size_t ISO_ADJ_HEADER_BYTES = sizeof(IsoAdjFileHeader);
static_assert(ISO_ADJ_HEADER_BYTES == 336);

struct IsoAdjMetadata
{
    std::uint32_t phast_schema_version = PHAST_SCHEMA_VERSION;
    MetricKind metric_kind = MetricKind::Weight;
    contractor::PHASTOrientation orientation = contractor::PHASTOrientation::Forward;
    std::uint32_t exclude_index = 0;
    std::uint32_t connectivity_checksum = 0;
    std::uint64_t node_count = 0;
    std::uint64_t edge_count = 0;
    std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES> build_id{};
    std::string metric_name;
};

struct IsoAdj
{
    boost::iostreams::mapped_file_source file;
    IsoAdjMetadata metadata;
    const std::uint64_t *offsets = nullptr;
    const std::uint32_t *targets = nullptr;
    const std::uint32_t *costs = nullptr;
};

std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES>
BuildIsoAdjBuildID(const std::string &dataset_timestamp, const IsoAdjMetadata &metadata);

bool WriteIsoAdj(const std::filesystem::path &path,
                 const IsoAdjMetadata &metadata,
                 const std::vector<std::uint64_t> &offsets,
                 const std::vector<std::uint32_t> &targets,
                 const std::vector<std::uint32_t> &costs);

bool LoadIsoAdj(const std::filesystem::path &path, IsoAdj &adjacency);

void RunCappedDijkstraIsoAdj(const IsoAdj &adjacency,
                             const std::vector<PHASTSeed> &seeds,
                             const std::optional<EdgeWeight> &cap_metric,
                             std::vector<EdgeWeight> &distances);

} // namespace osrm::contractor::phast

#endif
