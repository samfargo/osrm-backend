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

inline constexpr std::array<char, 8> ISO_ADJ_MAGIC = {'I', 'S', 'O', 'A', 'D', 'J', '4', '\0'};
inline constexpr std::uint32_t ISO_ADJ_VERSION = 4;
inline constexpr std::uint32_t ISO_ADJ_NODE_ORDER_MORTON = 1;
inline constexpr std::size_t ISO_ADJ_BUILD_ID_BYTES = 20;
inline constexpr std::size_t ISO_ADJ_METRIC_NAME_BYTES = 256;
inline constexpr std::size_t ISO_ADJ_ARTIFACT_VERSION_BYTES = 64;

#pragma pack(push, 1)

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
    std::uint32_t resolution = 0;
    std::uint32_t node_order = ISO_ADJ_NODE_ORDER_MORTON;
    std::uint64_t node_count = 0;
    std::uint64_t edge_count = 0;
    std::uint64_t candidate_count = 0;
    std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES> build_id{};
    std::uint32_t metric_name_size = 0;
    std::array<char, ISO_ADJ_METRIC_NAME_BYTES> metric_name{};
    std::uint32_t artifact_version_size = 0;
    std::array<char, ISO_ADJ_ARTIFACT_VERSION_BYTES> artifact_version{};
    std::uint64_t ordinal_count = 0;
    std::uint64_t ordinals_file_size = 0;
    std::uint64_t ordinals_hash_hi = 0;
    std::uint64_t ordinals_hash_lo = 0;
};
inline constexpr std::size_t ISO_ADJ_HEADER_BYTES = sizeof(IsoAdjFileHeader);
static_assert(ISO_ADJ_HEADER_BYTES == 448);

inline constexpr std::array<char, 8> SPARSE_RESULT_MAGIC = {
    'P', 'H', 'S', 'P', 'A', 'R', '1', '\0'};
inline constexpr std::uint32_t SPARSE_RESULT_VERSION = 1;

struct SparseResultFileHeader
{
    std::array<char, 8> magic = SPARSE_RESULT_MAGIC;
    std::uint32_t format_version = SPARSE_RESULT_VERSION;
    std::uint32_t header_bytes = 0;
    std::uint32_t phast_schema_version = 0;
    std::uint32_t metric_kind = 0;
    std::uint32_t orientation = 0;
    std::uint32_t exclude_index = 0;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t resolution = 0;
    std::uint32_t cap_seconds = 0;
    std::uint64_t node_count = 0;
    std::uint64_t edge_count = 0;
    std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES> build_id{};
    std::uint64_t ordinal_count = 0;
    std::uint64_t ordinals_file_size = 0;
    std::uint64_t ordinals_hash_hi = 0;
    std::uint64_t ordinals_hash_lo = 0;
    std::array<char, ISO_ADJ_ARTIFACT_VERSION_BYTES> artifact_version{};
    std::uint64_t record_count = 0;
};
static_assert(sizeof(SparseResultFileHeader) == 184);

struct SparseResultRecord
{
    std::uint32_t ordinal = 0;
    std::uint16_t seconds = 0;
    std::uint16_t reserved = 0;
};
static_assert(sizeof(SparseResultRecord) == 8);

#pragma pack(pop)

struct IsoAdjMetadata
{
    std::uint32_t phast_schema_version = PHAST_SCHEMA_VERSION;
    MetricKind metric_kind = MetricKind::Weight;
    contractor::PHASTOrientation orientation = contractor::PHASTOrientation::Forward;
    std::uint32_t exclude_index = 0;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t node_order = ISO_ADJ_NODE_ORDER_MORTON;
    std::uint64_t node_count = 0;
    std::uint64_t edge_count = 0;
    std::uint64_t candidate_count = 0;
    std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES> build_id{};
    std::string metric_name;
    std::uint32_t resolution = 0;
    std::string artifact_version;
    std::uint64_t ordinal_count = 0;
    std::uint64_t ordinals_file_size = 0;
    std::uint64_t ordinals_hash_hi = 0;
    std::uint64_t ordinals_hash_lo = 0;
};

struct IsoAdj
{
    boost::iostreams::mapped_file_source file;
    IsoAdjMetadata metadata;
    const std::uint64_t *offsets = nullptr;
    const std::uint32_t *targets = nullptr;
    const std::uint32_t *costs = nullptr;
    const std::uint64_t *cell_offsets = nullptr;
    const std::uint32_t *candidate_ordinals = nullptr;
    const std::int32_t *candidate_costs = nullptr;
    const std::uint32_t *original_to_spatial = nullptr;
};

struct ReachedState
{
    std::uint32_t node = 0;
    EdgeWeight cost = INVALID_EDGE_WEIGHT;
};

std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES>
BuildIsoAdjBuildID(const std::string &dataset_timestamp, const IsoAdjMetadata &metadata);

bool WriteIsoAdj(const std::filesystem::path &path,
                 const IsoAdjMetadata &metadata,
                 const std::vector<std::uint64_t> &offsets,
                 const std::vector<std::uint32_t> &targets,
                 const std::vector<std::uint32_t> &costs,
                 const std::vector<std::uint64_t> &cell_offsets,
                 const std::vector<std::uint32_t> &candidate_ordinals,
                 const std::vector<std::int32_t> &candidate_costs,
                 const std::vector<std::uint32_t> &original_to_spatial);

bool LoadIsoAdj(const std::filesystem::path &path, IsoAdj &adjacency);

bool RunCappedDijkstraIsoAdj(const IsoAdj &adjacency,
                            const std::vector<PHASTSeed> &seeds,
                            const std::optional<EdgeWeight> &cap_metric,
                            std::vector<ReachedState> &reached);

bool ReduceReachedCells(const IsoAdj &adjacency,
                        const std::vector<ReachedState> &reached,
                        const std::optional<EdgeWeight> &cap_metric,
                        std::vector<SparseResultRecord> &records);

bool WriteSparseResult(const std::filesystem::path &path,
                       const IsoAdjMetadata &metadata,
                       std::uint32_t cap_seconds,
                       const std::vector<SparseResultRecord> &records);

} // namespace osrm::contractor::phast

#endif
