#include "engine/approach.hpp"
#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/process_memory_allocator.hpp"
#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/exception.hpp"
#include "storage/io_config.hpp"
#include "storage/serialization.hpp"
#include "storage/storage_config.hpp"
#include "storage/tar.hpp"
#include "util/coordinate.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/version.hpp"

#include <boost/program_options.hpp>
#include <boost/uuid/detail/sha1.hpp>
#include <h3api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

using namespace osrm;

namespace
{
struct RasterizeConfig final : storage::IOConfig
{
    RasterizeConfig() : IOConfig({".osrm.hsgr", ".osrm.properties"}, {}, {}) {}

    void UseDefaultOutputNames(const std::filesystem::path &base) { IOConfig::UseDefaultOutputNames(base); }

    bool IsValid() const { return IOConfig::IsValid(); }
};

struct RuntimeConfig final
{
    std::filesystem::path phastfield_path;
    std::filesystem::path sample_ordinals_path;
    std::filesystem::path sample_snap_cache_path;
    std::filesystem::path output_path;
    std::uint32_t expected_resolution = 9;
    bool prepare_snap_cache_only = false;
};

enum class return_code : unsigned
{
    ok,
    fail,
    exit
};

enum class MetricKind : std::uint8_t
{
    Weight = 0,
    Duration = 1
};

enum class PHASTOrientation : std::uint8_t
{
    Forward = 0,
    Reverse = 1
};

struct PhastFieldData
{
    std::uint32_t schema_version = 0;
    std::uint32_t connectivity_checksum = 0;
    std::string metric_name;
    MetricKind metric_kind = MetricKind::Weight;
    PHASTOrientation orientation = PHASTOrientation::Forward;
    std::uint32_t exclude_index = 0;
    std::uint32_t cap_metric = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t node_count = 0;
    std::uint32_t poi_count = 0;
    bool encoded_u16 = false;
    std::vector<EdgeWeight> costs;
};

struct RasterizationStats
{
    std::size_t mapped_samples = 0;
    std::size_t fallback_candidate_used_samples = 0;
    std::size_t no_candidate_samples = 0;
    std::size_t no_valid_state_samples = 0;
    std::size_t unreachable_state_samples = 0;
    std::size_t capped_samples = 0;
};

using CHDataFacade =
    engine::datafacade::ContiguousInternalMemoryDataFacade<engine::routing_algorithms::ch::Algorithm>;

constexpr std::uint32_t PHASTFIELD_SCHEMA_VERSION = 1;
constexpr std::uint32_t SAMPLE_SNAP_CACHE_VERSION = 2;
constexpr std::array<char, 8> SAMPLE_SNAP_CACHE_MAGIC = {'P', 'H', 'S', 'N', 'A', 'P', '2', '\0'};

struct SamplePrimaryHint
{
    util::Coordinate coordinate;
    bool has_forward_state = false;
    bool forward_offset_valid = false;
    std::uint32_t forward_node_id = 0;
    EdgeWeight forward_offset = INVALID_EDGE_WEIGHT;

    bool has_reverse_state = false;
    bool reverse_offset_valid = false;
    std::uint32_t reverse_node_id = 0;
    EdgeWeight reverse_offset = INVALID_EDGE_WEIGHT;
};

struct SampleSnapCacheHeader
{
    std::uint32_t version = SAMPLE_SNAP_CACHE_VERSION;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t expected_resolution = 0;
    std::uint32_t reserved = 0;
    std::uint64_t sample_count = 0;
    std::uint64_t ordinals_file_size = 0;
    std::uint64_t ordinals_hash_hi = 0;
    std::uint64_t ordinals_hash_lo = 0;
};

struct OrdinalsIdentity
{
    std::uint64_t sample_count = 0;
    std::uint64_t ordinals_file_size = 0;
    std::uint64_t ordinals_hash_hi = 0;
    std::uint64_t ordinals_hash_lo = 0;
};

template <typename T> bool writeBinary(std::ostream &output, const T &value)
{
    output.write(reinterpret_cast<const char *>(&value), sizeof(T));
    return output.good();
}

template <typename T> bool readBinary(std::istream &input, T &value)
{
    input.read(reinterpret_cast<char *>(&value), sizeof(T));
    return input.good();
}

bool writeSnapCacheHeader(std::ostream &output, const SampleSnapCacheHeader &header)
{
    if (!output.good())
    {
        return false;
    }
    if (!output.write(SAMPLE_SNAP_CACHE_MAGIC.data(), SAMPLE_SNAP_CACHE_MAGIC.size()))
    {
        return false;
    }
    return writeBinary(output, header.version) && writeBinary(output, header.connectivity_checksum) &&
           writeBinary(output, header.expected_resolution) && writeBinary(output, header.reserved) &&
           writeBinary(output, header.sample_count) && writeBinary(output, header.ordinals_file_size) &&
           writeBinary(output, header.ordinals_hash_hi) && writeBinary(output, header.ordinals_hash_lo);
}

bool readSnapCacheHeader(std::istream &input, SampleSnapCacheHeader &header)
{
    std::array<char, SAMPLE_SNAP_CACHE_MAGIC.size()> magic{};
    if (!input.read(magic.data(), magic.size()))
    {
        return false;
    }
    if (magic != SAMPLE_SNAP_CACHE_MAGIC)
    {
        return false;
    }
    return readBinary(input, header.version) && readBinary(input, header.connectivity_checksum) &&
           readBinary(input, header.expected_resolution) && readBinary(input, header.reserved) &&
           readBinary(input, header.sample_count) && readBinary(input, header.ordinals_file_size) &&
           readBinary(input, header.ordinals_hash_hi) && readBinary(input, header.ordinals_hash_lo);
}

std::uint64_t decodeLittleEndianU64(const unsigned char *bytes)
{
    std::uint64_t value = 0;
    for (const auto index : util::irange<std::size_t>(0, static_cast<std::size_t>(8)))
    {
        value |= static_cast<std::uint64_t>(bytes[index]) << (index * 8);
    }
    return value;
}

std::uint64_t loadBigEndianU64(const unsigned char *bytes)
{
    std::uint64_t value = 0;
    for (const auto index : util::irange<std::size_t>(0, static_cast<std::size_t>(8)))
    {
        value = (value << 8) | static_cast<std::uint64_t>(bytes[index]);
    }
    return value;
}

std::array<unsigned char, 20>
sha1DigestToBytes(const boost::uuids::detail::sha1::digest_type &digest)
{
    std::array<unsigned char, 20> bytes{};
    for (const auto index : util::irange<std::size_t>(0, static_cast<std::size_t>(5)))
    {
        const auto word = static_cast<std::uint32_t>(digest[index]);
        bytes[index * 4 + 0] = static_cast<unsigned char>((word >> 24) & 0xFF);
        bytes[index * 4 + 1] = static_cast<unsigned char>((word >> 16) & 0xFF);
        bytes[index * 4 + 2] = static_cast<unsigned char>((word >> 8) & 0xFF);
        bytes[index * 4 + 3] = static_cast<unsigned char>(word & 0xFF);
    }
    return bytes;
}

bool loadOrdinalsIdentityAndCoordinates(const std::filesystem::path &sample_ordinals_path,
                                        const std::uint32_t expected_resolution,
                                        OrdinalsIdentity &identity,
                                        std::vector<util::Coordinate> &coordinates)
{
    constexpr double RAD_TO_DEG = 57.295779513082320876798154814105;

    identity = {};
    coordinates.clear();

    std::error_code size_error;
    const auto ordinals_file_size = std::filesystem::file_size(sample_ordinals_path, size_error);
    if (size_error)
    {
        util::Log(logERROR) << "Failed to read ordinals file size: "
                            << sample_ordinals_path.string() << " (" << size_error.message() << ")";
        return false;
    }
    if (ordinals_file_size == 0 || ordinals_file_size % 8 != 0)
    {
        util::Log(logERROR) << "Invalid ordinals file byte size (must be >0 and divisible by 8): "
                            << sample_ordinals_path.string() << " (" << ordinals_file_size
                            << " bytes)";
        return false;
    }

    std::ifstream input(sample_ordinals_path, std::ios::binary);
    if (!input)
    {
        util::Log(logERROR) << "Could not open ordinals file: " << sample_ordinals_path.string();
        return false;
    }

    boost::uuids::detail::sha1 hasher;
    std::array<unsigned char, 8> raw{};
    std::uint64_t previous_h3_id = 0;
    bool saw_any_h3 = false;
    std::uint64_t sample_count = 0;

    const auto expected_rows =
        static_cast<std::size_t>(ordinals_file_size / static_cast<std::uint64_t>(8));
    coordinates.reserve(expected_rows);

    while (input.read(reinterpret_cast<char *>(raw.data()), raw.size()))
    {
        hasher.process_bytes(raw.data(), raw.size());
        const auto h3_id = decodeLittleEndianU64(raw.data());
        if (!isValidCell(static_cast<H3Index>(h3_id)))
        {
            util::Log(logERROR) << "Invalid H3 cell id in ordinals file at index " << sample_count
                                << ": " << h3_id;
            return false;
        }
        const auto resolution = getResolution(static_cast<H3Index>(h3_id));
        if (resolution != static_cast<int>(expected_resolution))
        {
            util::Log(logERROR) << "Unexpected H3 resolution in ordinals file at index "
                                << sample_count << ": found " << resolution << ", expected "
                                << expected_resolution;
            return false;
        }
        if (saw_any_h3 && h3_id <= previous_h3_id)
        {
            util::Log(logERROR) << "Ordinals must be strictly increasing at index " << sample_count
                                << " in " << sample_ordinals_path.string();
            return false;
        }
        saw_any_h3 = true;
        previous_h3_id = h3_id;

        LatLng lat_lng{};
        if (cellToLatLng(static_cast<H3Index>(h3_id), &lat_lng) != E_SUCCESS)
        {
            util::Log(logERROR) << "Failed to derive centroid for H3 cell " << h3_id
                                << " at index " << sample_count;
            return false;
        }
        const auto lon_deg = lat_lng.lng * RAD_TO_DEG;
        const auto lat_deg = lat_lng.lat * RAD_TO_DEG;
        try
        {
            coordinates.push_back(util::Coordinate{
                util::UnsafeFloatLongitude{lon_deg},
                util::UnsafeFloatLatitude{lat_deg},
            });
        }
        catch (const std::exception &)
        {
            util::Log(logERROR) << "Invalid centroid coordinate for H3 cell " << h3_id
                                << " at index " << sample_count;
            return false;
        }
        ++sample_count;
    }
    if (!input.eof())
    {
        util::Log(logERROR) << "Failed reading ordinals file: " << sample_ordinals_path.string();
        return false;
    }
    if (sample_count == 0)
    {
        util::Log(logERROR) << "No ordinals loaded from: " << sample_ordinals_path.string();
        return false;
    }

    boost::uuids::detail::sha1::digest_type digest{};
    hasher.get_digest(digest);
    const auto digest_bytes = sha1DigestToBytes(digest);

    identity.sample_count = sample_count;
    identity.ordinals_file_size = ordinals_file_size;
    identity.ordinals_hash_hi = loadBigEndianU64(digest_bytes.data());
    identity.ordinals_hash_lo = loadBigEndianU64(digest_bytes.data() + 8);
    return true;
}

bool sampleSnapCacheHeaderMatches(const SampleSnapCacheHeader &header,
                                  const std::uint32_t expected_resolution,
                                  const std::uint32_t connectivity_checksum,
                                  const OrdinalsIdentity &identity)
{
    return header.version == SAMPLE_SNAP_CACHE_VERSION &&
           header.expected_resolution == expected_resolution &&
           header.connectivity_checksum == connectivity_checksum &&
           header.sample_count == identity.sample_count &&
           header.ordinals_file_size == identity.ordinals_file_size &&
           header.ordinals_hash_hi == identity.ordinals_hash_hi &&
           header.ordinals_hash_lo == identity.ordinals_hash_lo;
}

bool readSampleSnapCacheMetadata(const std::filesystem::path &cache_path, SampleSnapCacheHeader &header)
{
    std::ifstream input(cache_path, std::ios::binary);
    if (!input)
    {
        return false;
    }
    return readSnapCacheHeader(input, header);
}

return_code
parseArguments(int argc,
               char *argv[],
               std::string &verbosity,
               RasterizeConfig &rasterize_config,
               RuntimeConfig &runtime_config)
{
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
        "list-inputs", "List required and optional input file extensions")(
        "verbosity,l",
        boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
        std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

    boost::program_options::options_description execution_options("Execution");
    execution_options.add_options()(
        "phastfield",
        boost::program_options::value<std::filesystem::path>(&runtime_config.phastfield_path),
        "Path to .phastfield artifact emitted by osrm-phast")(
        "sample-ordinals",
        boost::program_options::value<std::filesystem::path>(&runtime_config.sample_ordinals_path),
        "Little-endian uint64 ordinal IDs file (h3_r<res>_ordinals.u64)")(
        "sample-snap-cache",
        boost::program_options::value<std::filesystem::path>(&runtime_config.sample_snap_cache_path),
        "Primary snap cache file for sample points")(
        "prepare-snap-cache-only",
        boost::program_options::bool_switch(&runtime_config.prepare_snap_cache_only),
        "Build/refresh --sample-snap-cache then exit without rasterizing")(
        "output",
        boost::program_options::value<std::filesystem::path>(&runtime_config.output_path),
        "Output dense little-endian uint16 artifact path (.u16)")(
        "resolution",
        boost::program_options::value<std::uint32_t>(&runtime_config.expected_resolution)
            ->default_value(9),
        "Expected H3 resolution in ordinals file (default 9)");

    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i",
        boost::program_options::value<std::filesystem::path>(&rasterize_config.base_path),
        "Input base file path");

    boost::program_options::positional_options_description positional_options;
    positional_options.add("input", 1);

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(execution_options).add(hidden_options);

    const auto *executable = argv[0];
    boost::program_options::options_description visible_options(
        "Usage: " + std::filesystem::path(executable).filename().string() + " <input.osrm> [options]");
    visible_options.add(generic_options).add(execution_options);

    boost::program_options::variables_map option_variables;
    try
    {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                          .options(cmdline_options)
                                          .positional(positional_options)
                                          .run(),
                                      option_variables);
    }
    catch (const boost::program_options::error &e)
    {
        util::Log(logERROR) << e.what();
        return return_code::fail;
    }

    if (option_variables.contains("version"))
    {
        std::cout << OSRM_VERSION << std::endl;
        return return_code::exit;
    }

    if (option_variables.contains("help"))
    {
        std::cout << visible_options;
        return return_code::exit;
    }

    if (option_variables.contains("list-inputs"))
    {
        RasterizeConfig config;
        std::set<std::string> seen;
        config.ListInputFiles(std::cout, seen);
        return return_code::exit;
    }

    boost::program_options::notify(option_variables);

    if (!option_variables.contains("input"))
    {
        std::cout << visible_options;
        return return_code::fail;
    }
    if (!option_variables.contains("sample-ordinals") || runtime_config.sample_ordinals_path.empty())
    {
        util::Log(logERROR) << "--sample-ordinals is required.";
        return return_code::fail;
    }
    if (runtime_config.sample_snap_cache_path.empty())
    {
        util::Log(logERROR) << "--sample-snap-cache is required.";
        return return_code::fail;
    }
    if (!runtime_config.prepare_snap_cache_only &&
        (!option_variables.contains("phastfield") || runtime_config.phastfield_path.empty()))
    {
        util::Log(logERROR) << "--phastfield is required.";
        return return_code::fail;
    }
    if (!runtime_config.prepare_snap_cache_only &&
        (!option_variables.contains("output") || runtime_config.output_path.empty()))
    {
        util::Log(logERROR) << "--output is required.";
        return return_code::fail;
    }
    if (runtime_config.expected_resolution == 0)
    {
        util::Log(logERROR) << "--resolution must be > 0.";
        return return_code::fail;
    }
    return return_code::ok;
}

bool loadCHFacade(const std::filesystem::path &base_path,
                  const std::string &metric_name,
                  const std::size_t exclude_index,
                  std::shared_ptr<const CHDataFacade> &facade)
{
    const storage::StorageConfig storage_config(base_path);
    if (!storage_config.IsValid())
    {
        util::Log(logERROR) << "Dataset files required for PHAST rasterization are missing.";
        return false;
    }

    try
    {
        auto allocator =
            std::make_shared<engine::datafacade::ProcessMemoryAllocator>(storage_config);
        facade = std::make_shared<const CHDataFacade>(allocator, metric_name, exclude_index);
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to initialize CH facade: " << e.what();
        return false;
    }
    return true;
}

bool readPhastField(const std::filesystem::path &path, PhastFieldData &data)
{
    try
    {
        const auto fingerprint = storage::tar::FileReader::VerifyFingerprint;
        storage::tar::FileReader reader{path, fingerprint};

        std::uint32_t metric_kind = 0;
        std::uint32_t orientation = 0;
        reader.ReadInto("/phast_result/meta/schema_version", data.schema_version);
        reader.ReadInto("/phast_result/meta/connectivity_checksum", data.connectivity_checksum);
        storage::serialization::read(reader, "/phast_result/meta/metric_name", data.metric_name);
        reader.ReadInto("/phast_result/meta/metric_kind", metric_kind);
        reader.ReadInto("/phast_result/meta/orientation", orientation);
        reader.ReadInto("/phast_result/meta/exclude_index", data.exclude_index);
        reader.ReadInto("/phast_result/meta/cap_metric", data.cap_metric);
        reader.ReadInto("/phast_result/meta/node_count", data.node_count);
        reader.ReadInto("/phast_result/meta/poi_count", data.poi_count);
        data.metric_kind = static_cast<MetricKind>(metric_kind);
        data.orientation = static_cast<PHASTOrientation>(orientation);

        std::vector<storage::tar::FileReader::FileEntry> entries;
        reader.List(std::back_inserter(entries));
        auto has_path = [&](const std::string &target)
        {
            return std::find_if(entries.begin(), entries.end(), [&](const auto &entry)
                                { return entry.name == target; }) != entries.end();
        };

        if (has_path("/phast_result/costs_u16"))
        {
            std::vector<std::uint16_t> raw;
            storage::serialization::read(reader, "/phast_result/costs_u16", raw);
            data.encoded_u16 = true;
            data.costs.resize(raw.size(), INVALID_EDGE_WEIGHT);
            for (const auto i : util::irange<std::size_t>(0, raw.size()))
            {
                if (raw[i] == std::numeric_limits<std::uint16_t>::max())
                {
                    continue;
                }
                data.costs[i] = to_alias<EdgeWeight>(raw[i]);
            }
            return true;
        }
        if (has_path("/phast_result/costs_u32"))
        {
            std::vector<std::uint32_t> raw;
            storage::serialization::read(reader, "/phast_result/costs_u32", raw);
            data.encoded_u16 = false;
            data.costs.resize(raw.size(), INVALID_EDGE_WEIGHT);
            const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
            for (const auto i : util::irange<std::size_t>(0, raw.size()))
            {
                if (raw[i] == std::numeric_limits<std::uint32_t>::max())
                {
                    continue;
                }
                if (raw[i] > static_cast<std::uint64_t>(max_valid))
                {
                    util::Log(logERROR) << "Invalid costs_u32 value at index " << i;
                    return false;
                }
                data.costs[i] = to_alias<EdgeWeight>(static_cast<std::int64_t>(raw[i]));
            }
            return true;
        }

        util::Log(logERROR) << "No costs_u16 or costs_u32 payload found in phastfield artifact.";
        return false;
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to read phastfield: " << e.what();
        return false;
    }
}

bool validatePhastFieldMetadata(const PhastFieldData &data,
                                const extractor::ProfileProperties &properties,
                                const CHDataFacade &facade)
{
    if (data.schema_version != PHASTFIELD_SCHEMA_VERSION)
    {
        util::Log(logERROR) << "Unsupported phastfield schema version " << data.schema_version
                            << ". Expected " << PHASTFIELD_SCHEMA_VERSION << ".";
        return false;
    }
    if (data.metric_name != properties.GetWeightName())
    {
        util::Log(logERROR) << "Metric mismatch between phastfield and .osrm.properties.";
        return false;
    }
    if (data.metric_name != facade.GetWeightName())
    {
        util::Log(logERROR) << "Metric mismatch between phastfield and CH facade.";
        return false;
    }
    if (facade.GetCheckSum() != data.connectivity_checksum)
    {
        util::Log(logERROR) << "Checksum mismatch between phastfield and CH facade.";
        return false;
    }
    if (facade.GetNumberOfNodes() != data.node_count)
    {
        util::Log(logERROR) << "Node count mismatch between phastfield and CH facade.";
        return false;
    }
    if (data.costs.size() != data.node_count)
    {
        util::Log(logERROR) << "Cost vector size mismatch in phastfield.";
        return false;
    }
    return true;
}

bool tryAddCost(const EdgeWeight lhs, const EdgeWeight rhs, EdgeWeight &sum)
{
    if (lhs == INVALID_EDGE_WEIGHT || rhs == INVALID_EDGE_WEIGHT)
    {
        return false;
    }
    const auto total = from_alias<std::int64_t>(lhs) + from_alias<std::int64_t>(rhs);
    const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
    if (total < 0 || total > max_valid)
    {
        return false;
    }
    sum = to_alias<EdgeWeight>(total);
    return true;
}

bool buildPrimaryHintForSample(const CHDataFacade &facade, const util::Coordinate &coordinate, SamplePrimaryHint &hint)
{
    hint = {};
    hint.coordinate = coordinate;

    const auto nearest_candidates = facade.NearestPhantomNodes(coordinate,
                                                               1,
                                                               std::nullopt,
                                                               std::nullopt,
                                                               engine::Approach::UNRESTRICTED);
    if (nearest_candidates.empty())
    {
        return true;
    }

    const auto &phantom = nearest_candidates.front().phantom_node;
    const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;

    if (phantom.forward_segment_id.enabled && phantom.IsValidForwardSource())
    {
        hint.has_forward_state = true;
        hint.forward_node_id = phantom.forward_segment_id.id;
        const auto duration = phantom.GetForwardDuration();
        if (duration != MAXIMAL_EDGE_DURATION)
        {
            const auto duration_value = from_alias<std::int64_t>(duration);
            const auto signed_offset = -duration_value;
            if (duration_value >= 0 && duration_value <= max_valid && signed_offset >= -max_valid)
            {
                hint.forward_offset = to_alias<EdgeWeight>(signed_offset);
                hint.forward_offset_valid = true;
            }
        }
    }

    if (phantom.reverse_segment_id.enabled && phantom.IsValidReverseSource())
    {
        hint.has_reverse_state = true;
        hint.reverse_node_id = phantom.reverse_segment_id.id;
        const auto duration = phantom.GetReverseDuration();
        if (duration != MAXIMAL_EDGE_DURATION)
        {
            const auto duration_value = from_alias<std::int64_t>(duration);
            const auto signed_offset = -duration_value;
            if (duration_value >= 0 && duration_value <= max_valid && signed_offset >= -max_valid)
            {
                hint.reverse_offset = to_alias<EdgeWeight>(signed_offset);
                hint.reverse_offset_valid = true;
            }
        }
    }

    return true;
}

constexpr std::uint8_t SAMPLE_HINT_HAS_FORWARD = 1U << 0;
constexpr std::uint8_t SAMPLE_HINT_FORWARD_OFFSET_VALID = 1U << 1;
constexpr std::uint8_t SAMPLE_HINT_HAS_REVERSE = 1U << 2;
constexpr std::uint8_t SAMPLE_HINT_REVERSE_OFFSET_VALID = 1U << 3;
constexpr std::size_t SAMPLE_HINT_RECORD_BYTES = 28;

bool writePrimaryHintRecord(std::ostream &output, const SamplePrimaryHint &hint)
{
    const auto lon_fixed = static_cast<std::int32_t>(hint.coordinate.lon);
    const auto lat_fixed = static_cast<std::int32_t>(hint.coordinate.lat);
    const auto forward_offset_ticks = static_cast<std::int32_t>(from_alias<std::int64_t>(hint.forward_offset));
    const auto reverse_offset_ticks = static_cast<std::int32_t>(from_alias<std::int64_t>(hint.reverse_offset));

    std::uint8_t flags = 0;
    if (hint.has_forward_state)
    {
        flags |= SAMPLE_HINT_HAS_FORWARD;
    }
    if (hint.forward_offset_valid)
    {
        flags |= SAMPLE_HINT_FORWARD_OFFSET_VALID;
    }
    if (hint.has_reverse_state)
    {
        flags |= SAMPLE_HINT_HAS_REVERSE;
    }
    if (hint.reverse_offset_valid)
    {
        flags |= SAMPLE_HINT_REVERSE_OFFSET_VALID;
    }

    const std::uint8_t reserved0 = 0;
    const std::uint16_t reserved1 = 0;
    return writeBinary(output, lon_fixed) && writeBinary(output, lat_fixed) &&
           writeBinary(output, hint.forward_node_id) && writeBinary(output, hint.reverse_node_id) &&
           writeBinary(output, forward_offset_ticks) && writeBinary(output, reverse_offset_ticks) &&
           writeBinary(output, flags) && writeBinary(output, reserved0) && writeBinary(output, reserved1);
}

bool readPrimaryHintRecord(std::istream &input, SamplePrimaryHint &hint)
{
    std::int32_t lon_fixed = 0;
    std::int32_t lat_fixed = 0;
    std::int32_t forward_offset_ticks = 0;
    std::int32_t reverse_offset_ticks = 0;
    std::uint8_t flags = 0;
    std::uint8_t reserved0 = 0;
    std::uint16_t reserved1 = 0;

    if (!readBinary(input, lon_fixed) || !readBinary(input, lat_fixed) ||
        !readBinary(input, hint.forward_node_id) || !readBinary(input, hint.reverse_node_id) ||
        !readBinary(input, forward_offset_ticks) || !readBinary(input, reverse_offset_ticks) ||
        !readBinary(input, flags) || !readBinary(input, reserved0) || !readBinary(input, reserved1))
    {
        return false;
    }

    hint.coordinate = util::Coordinate{
        util::FixedLongitude{lon_fixed},
        util::FixedLatitude{lat_fixed},
    };
    hint.has_forward_state = (flags & SAMPLE_HINT_HAS_FORWARD) != 0;
    hint.forward_offset_valid = (flags & SAMPLE_HINT_FORWARD_OFFSET_VALID) != 0;
    hint.has_reverse_state = (flags & SAMPLE_HINT_HAS_REVERSE) != 0;
    hint.reverse_offset_valid = (flags & SAMPLE_HINT_REVERSE_OFFSET_VALID) != 0;
    hint.forward_offset = to_alias<EdgeWeight>(static_cast<std::int64_t>(forward_offset_ticks));
    hint.reverse_offset = to_alias<EdgeWeight>(static_cast<std::int64_t>(reverse_offset_ticks));
    return true;
}

bool ensureSampleSnapCache(const CHDataFacade &facade,
                           const std::filesystem::path &sample_ordinals_path,
                           const std::uint32_t expected_resolution,
                           const std::filesystem::path &cache_path,
                           std::uint64_t &sample_count,
                           bool &reused_existing_cache)
{
    sample_count = 0;
    reused_existing_cache = false;

    OrdinalsIdentity ordinals_identity;
    std::vector<util::Coordinate> sample_coordinates;
    if (!loadOrdinalsIdentityAndCoordinates(
            sample_ordinals_path, expected_resolution, ordinals_identity, sample_coordinates))
    {
        return false;
    }
    sample_count = ordinals_identity.sample_count;

    const auto connectivity_checksum = facade.GetCheckSum();
    SampleSnapCacheHeader cached_header;
    if (readSampleSnapCacheMetadata(cache_path, cached_header) &&
        sampleSnapCacheHeaderMatches(cached_header,
                                     expected_resolution,
                                     connectivity_checksum,
                                     ordinals_identity))
    {
        const auto expected_size = SAMPLE_SNAP_CACHE_MAGIC.size() + sizeof(std::uint32_t) * 4 +
                                   sizeof(std::uint64_t) * 4 +
                                   cached_header.sample_count * SAMPLE_HINT_RECORD_BYTES;
        std::error_code file_size_error;
        const auto actual_size = std::filesystem::file_size(cache_path, file_size_error);
        if (!file_size_error && actual_size == expected_size)
        {
            sample_count = cached_header.sample_count;
            reused_existing_cache = true;
            return true;
        }
    }

    std::error_code cache_parent_error;
    const auto cache_parent = cache_path.parent_path();
    if (!cache_parent.empty())
    {
        std::filesystem::create_directories(cache_parent, cache_parent_error);
        if (cache_parent_error)
        {
            util::Log(logERROR) << "Failed to create cache directory " << cache_parent.string()
                                << ": " << cache_parent_error.message();
            return false;
        }
    }

    auto cache_tmp = cache_path;
    cache_tmp += ".tmp";
    std::fstream cache_output(cache_tmp, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!cache_output)
    {
        util::Log(logERROR) << "Could not open snap cache output path: " << cache_tmp.string();
        return false;
    }

    SampleSnapCacheHeader header;
    header.connectivity_checksum = connectivity_checksum;
    header.expected_resolution = expected_resolution;
    header.sample_count = ordinals_identity.sample_count;
    header.ordinals_file_size = ordinals_identity.ordinals_file_size;
    header.ordinals_hash_hi = ordinals_identity.ordinals_hash_hi;
    header.ordinals_hash_lo = ordinals_identity.ordinals_hash_lo;
    if (!writeSnapCacheHeader(cache_output, header))
    {
        util::Log(logERROR) << "Failed writing snap cache header: " << cache_tmp.string();
        return false;
    }

    sample_count = 0;
    for (const auto &coordinate : sample_coordinates)
    {
        SamplePrimaryHint hint;
        if (!buildPrimaryHintForSample(facade, coordinate, hint))
        {
            return false;
        }
        if (!writePrimaryHintRecord(cache_output, hint))
        {
            util::Log(logERROR) << "Failed writing snap cache record at sample #" << (sample_count + 1);
            return false;
        }
        ++sample_count;
    }

    if (sample_count == 0)
    {
        util::Log(logERROR) << "No sample points loaded from ordinals: "
                            << sample_ordinals_path.string();
        return false;
    }
    if (sample_count != header.sample_count)
    {
        util::Log(logERROR) << "Sample count mismatch while building snap cache from ordinals: "
                            << sample_count << " vs header " << header.sample_count;
        return false;
    }
    cache_output.seekp(0, std::ios::beg);
    if (!writeSnapCacheHeader(cache_output, header))
    {
        util::Log(logERROR) << "Failed finalizing snap cache header: " << cache_tmp.string();
        return false;
    }
    cache_output.close();
    if (!cache_output)
    {
        util::Log(logERROR) << "Failed flushing snap cache: " << cache_tmp.string();
        return false;
    }

    std::error_code remove_error;
    std::filesystem::remove(cache_path, remove_error);
    (void)remove_error;
    std::error_code rename_error;
    std::filesystem::rename(cache_tmp, cache_path, rename_error);
    if (rename_error)
    {
        util::Log(logERROR) << "Failed to finalize snap cache " << cache_path.string() << ": "
                            << rename_error.message();
        std::error_code cleanup_error;
        std::filesystem::remove(cache_tmp, cleanup_error);
        (void)cleanup_error;
        return false;
    }

    reused_existing_cache = false;
    return true;
}

bool mapSampleFromPrimaryHint(const SamplePrimaryHint &hint,
                              const std::vector<EdgeWeight> &costs,
                              EdgeWeight &mapped_cost)
{
    mapped_cost = INVALID_EDGE_WEIGHT;
    EdgeWeight best_candidate_cost = INVALID_EDGE_WEIGHT;

    auto evaluate_hint_state = [&](const bool has_state,
                                   const std::uint32_t node_id,
                                   const bool offset_valid,
                                   const EdgeWeight offset) -> bool
    {
        if (!has_state)
        {
            return true;
        }
        if (node_id >= costs.size())
        {
            util::Log(logERROR) << "Primary snap node id out of bounds: " << node_id;
            return false;
        }
        const auto state_cost = costs[node_id];
        if (state_cost == INVALID_EDGE_WEIGHT || !offset_valid)
        {
            return true;
        }
        EdgeWeight total_cost = INVALID_EDGE_WEIGHT;
        if (!tryAddCost(state_cost, offset, total_cost))
        {
            return true;
        }
        if (best_candidate_cost == INVALID_EDGE_WEIGHT || total_cost < best_candidate_cost)
        {
            best_candidate_cost = total_cost;
        }
        return true;
    };

    if (!evaluate_hint_state(
            hint.has_forward_state, hint.forward_node_id, hint.forward_offset_valid, hint.forward_offset) ||
        !evaluate_hint_state(
            hint.has_reverse_state, hint.reverse_node_id, hint.reverse_offset_valid, hint.reverse_offset))
    {
        return false;
    }

    mapped_cost = best_candidate_cost;
    return true;
}

bool writeDenseU16ArtifactFromSnapCache(const std::filesystem::path &cache_path,
                                        const std::string &metric_name,
                                        const MetricKind metric_kind,
                                        const PHASTOrientation orientation,
                                        const std::optional<EdgeWeight> cap_metric,
                                        const std::vector<EdgeWeight> &costs,
                                        const std::filesystem::path &output_path,
                                        RasterizationStats &stats,
                                        std::uint64_t &cell_count,
                                        std::size_t &reachable_cells,
                                        std::size_t &unreachable_cells,
                                        std::size_t &bytes_written)
{
    if (metric_kind != MetricKind::Duration || orientation != PHASTOrientation::Reverse)
    {
        util::Log(logERROR) << "Sample snap cache currently supports only duration+reverse PHAST fields.";
        return false;
    }

    std::ifstream cache_input(cache_path, std::ios::binary);
    if (!cache_input)
    {
        util::Log(logERROR) << "Could not open sample snap cache: " << cache_path.string();
        return false;
    }

    SampleSnapCacheHeader header;
    if (!readSnapCacheHeader(cache_input, header))
    {
        util::Log(logERROR) << "Invalid sample snap cache header: " << cache_path.string();
        return false;
    }

    const auto expected_size = SAMPLE_SNAP_CACHE_MAGIC.size() + sizeof(std::uint32_t) * 4 +
                               sizeof(std::uint64_t) * 4 +
                               header.sample_count * SAMPLE_HINT_RECORD_BYTES;
    std::error_code cache_size_error;
    const auto actual_size = std::filesystem::file_size(cache_path, cache_size_error);
    if (cache_size_error || actual_size != expected_size)
    {
        util::Log(logERROR) << "Sample snap cache size mismatch: " << cache_path.string();
        return false;
    }

    constexpr std::uint16_t MAX_REACHABLE_U16_INT = 65534;
    constexpr std::uint16_t UNREACHABLE_U16_INT = 65535;

    auto output_tmp = output_path;
    output_tmp += ".tmp";
    std::ofstream output(output_tmp, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        util::Log(logERROR) << "Could not open output artifact path: " << output_tmp.string();
        return false;
    }

    stats = {};
    cell_count = header.sample_count;
    reachable_cells = 0;
    unreachable_cells = 0;
    bytes_written = 0;
    std::size_t primary_hint_miss_count = 0;

    for (const auto index : util::irange<std::uint64_t>(0, header.sample_count))
    {
        SamplePrimaryHint hint;
        if (!readPrimaryHintRecord(cache_input, hint))
        {
            util::Log(logERROR) << "Failed reading sample snap cache record at index " << index;
            return false;
        }

        EdgeWeight mapped_cost = INVALID_EDGE_WEIGHT;
        if (!mapSampleFromPrimaryHint(hint, costs, mapped_cost))
        {
            return false;
        }

        if (mapped_cost != INVALID_EDGE_WEIGHT)
        {
            ++stats.mapped_samples;
        }
        else
        {
            ++primary_hint_miss_count;
            ++stats.no_candidate_samples;
        }

        std::uint16_t encoded = UNREACHABLE_U16_INT;
        if (mapped_cost == INVALID_EDGE_WEIGHT)
        {
            ++unreachable_cells;
        }
        else if (cap_metric.has_value() && mapped_cost > cap_metric.value())
        {
            ++stats.capped_samples;
            ++unreachable_cells;
        }
        else
        {
            const auto ticks = from_alias<std::int64_t>(mapped_cost);
            double seconds = static_cast<double>(ticks);
            if (metric_name == "duration")
            {
                seconds = ticks / 10.0;
            }
            const auto rounded = std::nearbyint(seconds);
            if (!std::isfinite(rounded) || rounded < 0.0)
            {
                ++unreachable_cells;
            }
            else
            {
                const auto clipped =
                    std::min<double>(rounded, static_cast<double>(MAX_REACHABLE_U16_INT));
                encoded = static_cast<std::uint16_t>(clipped);
                ++reachable_cells;
            }
        }

        const char bytes[2] = {
            static_cast<char>(encoded & 0xFF),
            static_cast<char>((encoded >> 8) & 0xFF),
        };
        output.write(bytes, sizeof(bytes));
        if (!output.good())
        {
            util::Log(logERROR) << "Failed writing raster artifact bytes: " << output_tmp.string();
            return false;
        }
    }

    output.close();
    if (!output)
    {
        util::Log(logERROR) << "Failed flushing raster artifact: " << output_tmp.string();
        return false;
    }

    if (primary_hint_miss_count > 0)
    {
        util::Log(logWARNING) << "Primary snap hint could not map " << primary_hint_miss_count
                              << " samples; they were marked unreachable.";
    }

    bytes_written = static_cast<std::size_t>(header.sample_count) * sizeof(std::uint16_t);
    std::error_code remove_error;
    std::filesystem::remove(output_path, remove_error);
    (void)remove_error;
    std::error_code rename_error;
    std::filesystem::rename(output_tmp, output_path, rename_error);
    if (rename_error)
    {
        util::Log(logERROR) << "Failed to finalize raster artifact " << output_path.string()
                            << ": " << rename_error.message();
        std::error_code cleanup_error;
        std::filesystem::remove(output_tmp, cleanup_error);
        (void)cleanup_error;
        return false;
    }

    return true;
}

bool writeStatsMetadata(const std::filesystem::path &output_path,
                        const std::size_t cell_count,
                        const std::size_t reachable_cells,
                        const std::size_t unreachable_cells,
                        const std::size_t bytes_written,
                        const RasterizationStats &rasterization_stats)
{
    auto metadata_path = output_path;
    metadata_path += ".meta.json";
    auto metadata_tmp = metadata_path;
    metadata_tmp += ".tmp";

    std::ofstream metadata(metadata_tmp, std::ios::trunc);
    if (!metadata)
    {
        util::Log(logERROR) << "Could not open raster metadata path: " << metadata_tmp.string();
        return false;
    }
    metadata << "{"
             << "\"cells_total\":" << cell_count << ","
             << "\"reachable_cells\":" << reachable_cells << ","
             << "\"unreachable_cells\":" << unreachable_cells << ","
             << "\"bytes\":" << bytes_written << ","
             << "\"mapped_samples\":" << rasterization_stats.mapped_samples << ","
             << "\"fallback_candidate_used_samples\":"
             << rasterization_stats.fallback_candidate_used_samples << ","
             << "\"no_candidate_samples\":" << rasterization_stats.no_candidate_samples << ","
             << "\"no_valid_state_samples\":" << rasterization_stats.no_valid_state_samples << ","
             << "\"unreachable_state_samples\":" << rasterization_stats.unreachable_state_samples << ","
             << "\"capped_samples\":" << rasterization_stats.capped_samples
             << "}\n";
    metadata.close();
    if (!metadata)
    {
        util::Log(logERROR) << "Failed flushing raster metadata: " << metadata_tmp.string();
        return false;
    }

    std::error_code remove_error;
    std::filesystem::remove(metadata_path, remove_error);
    (void)remove_error;
    std::error_code rename_error;
    std::filesystem::rename(metadata_tmp, metadata_path, rename_error);
    if (rename_error)
    {
        util::Log(logERROR) << "Failed to finalize raster metadata " << metadata_path.string()
                            << ": " << rename_error.message();
        std::error_code cleanup_error;
        std::filesystem::remove(metadata_tmp, cleanup_error);
        (void)cleanup_error;
        return false;
    }

    return true;
}
} // namespace

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();

    std::string verbosity;
    RasterizeConfig rasterize_config;
    RuntimeConfig runtime_config;

    const auto result = parseArguments(argc, argv, verbosity, rasterize_config, runtime_config);
    if (return_code::fail == result)
    {
        return EXIT_FAILURE;
    }
    if (return_code::exit == result)
    {
        return EXIT_SUCCESS;
    }

    util::LogPolicy::GetInstance().SetLevel(verbosity);
    rasterize_config.UseDefaultOutputNames(rasterize_config.base_path);
    if (!rasterize_config.IsValid())
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Input file: " << rasterize_config.base_path.string() << ".osrm";
    util::Log() << "Sample ordinals: " << runtime_config.sample_ordinals_path.string();
    util::Log() << "Expected H3 resolution: " << runtime_config.expected_resolution;
    if (!runtime_config.sample_snap_cache_path.empty())
    {
        util::Log() << "Sample snap cache: " << runtime_config.sample_snap_cache_path.string();
    }

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(rasterize_config.GetPath(".osrm.properties"), properties);

    if (runtime_config.prepare_snap_cache_only)
    {
        std::shared_ptr<const CHDataFacade> facade;
        if (!loadCHFacade(rasterize_config.base_path, properties.GetWeightName(), 0, facade))
        {
            return EXIT_FAILURE;
        }
        std::uint64_t cache_samples = 0;
        bool reused_existing_cache = false;
        if (!ensureSampleSnapCache(*facade,
                                   runtime_config.sample_ordinals_path,
                                   runtime_config.expected_resolution,
                                   runtime_config.sample_snap_cache_path,
                                   cache_samples,
                                   reused_existing_cache))
        {
            return EXIT_FAILURE;
        }
        util::Log() << (reused_existing_cache ? "Reused sample snap cache: "
                                              : "Built sample snap cache: ")
                    << runtime_config.sample_snap_cache_path.string();
        util::Log() << "Sample snap cache rows: " << cache_samples;
        util::DumpMemoryStats();
        return EXIT_SUCCESS;
    }

    util::Log() << "Phastfield: " << runtime_config.phastfield_path.string();
    util::Log() << "Output artifact (.u16): " << runtime_config.output_path.string();

    PhastFieldData field_data;
    if (!readPhastField(runtime_config.phastfield_path, field_data))
    {
        return EXIT_FAILURE;
    }
    util::Log() << "Phastfield schema: " << field_data.schema_version;
    util::Log() << "Phastfield metric: " << field_data.metric_name;
    util::Log() << "Phastfield node count: " << field_data.node_count;
    util::Log() << "Phastfield exclude index: " << field_data.exclude_index;
    util::Log() << "Phastfield POI count: " << field_data.poi_count;
    util::Log() << "Phastfield orientation: "
                << (field_data.orientation == PHASTOrientation::Reverse ? "reverse" : "forward");
    std::optional<EdgeWeight> cap_metric = std::nullopt;
    if (field_data.cap_metric != std::numeric_limits<std::uint32_t>::max())
    {
        const auto cap_ticks = static_cast<std::int64_t>(field_data.cap_metric);
        const auto max_valid = from_alias<std::int64_t>(INVALID_EDGE_WEIGHT) - 1;
        if (cap_ticks < 0 || cap_ticks > max_valid)
        {
            util::Log(logERROR) << "Invalid cap_metric in phastfield metadata.";
            return EXIT_FAILURE;
        }
        cap_metric = to_alias<EdgeWeight>(cap_ticks);
        util::Log() << "Phastfield cap metric: " << cap_ticks << " ticks";
    }

    std::shared_ptr<const CHDataFacade> facade;
    if (!loadCHFacade(rasterize_config.base_path,
                      properties.GetWeightName(),
                      field_data.exclude_index,
                      facade))
    {
        return EXIT_FAILURE;
    }
    if (!validatePhastFieldMetadata(field_data, properties, *facade))
    {
        return EXIT_FAILURE;
    }

    RasterizationStats rasterization_stats;
    std::size_t cell_count = 0;
    std::size_t reachable_cells = 0;
    std::size_t unreachable_cells = 0;
    std::size_t bytes_written = 0;

    if (field_data.metric_kind != MetricKind::Duration ||
        field_data.orientation != PHASTOrientation::Reverse)
    {
        util::Log(logERROR)
            << "Sample snap cache rasterization requires duration+reverse phastfield metadata.";
        return EXIT_FAILURE;
    }

    std::uint64_t cache_samples = 0;
    bool reused_existing_cache = false;
    if (!ensureSampleSnapCache(*facade,
                               runtime_config.sample_ordinals_path,
                               runtime_config.expected_resolution,
                               runtime_config.sample_snap_cache_path,
                               cache_samples,
                               reused_existing_cache))
    {
        return EXIT_FAILURE;
    }
    util::Log() << (reused_existing_cache ? "Reused sample snap cache: "
                                          : "Built sample snap cache: ")
                << runtime_config.sample_snap_cache_path.string();
    util::Log() << "Sample snap cache rows: " << cache_samples;

    std::uint64_t cell_count_u64 = 0;
    if (!writeDenseU16ArtifactFromSnapCache(runtime_config.sample_snap_cache_path,
                                            field_data.metric_name,
                                            field_data.metric_kind,
                                            field_data.orientation,
                                            cap_metric,
                                            field_data.costs,
                                            runtime_config.output_path,
                                            rasterization_stats,
                                            cell_count_u64,
                                            reachable_cells,
                                            unreachable_cells,
                                            bytes_written))
    {
        return EXIT_FAILURE;
    }
    if (cell_count_u64 > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
    {
        util::Log(logERROR) << "Sample count does not fit into size_t.";
        return EXIT_FAILURE;
    }
    cell_count = static_cast<std::size_t>(cell_count_u64);

    if (!writeStatsMetadata(runtime_config.output_path,
                            cell_count,
                            reachable_cells,
                            unreachable_cells,
                            bytes_written,
                            rasterization_stats))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Rasterized cells: " << cell_count;
    util::Log() << "Reachable cells: " << reachable_cells;
    util::Log() << "Unreachable cells: " << unreachable_cells;
    util::Log() << "Mapped samples: " << rasterization_stats.mapped_samples;
    util::Log() << "Capped samples: " << rasterization_stats.capped_samples;
    util::Log() << "Samples with no snap candidates: " << rasterization_stats.no_candidate_samples;
    util::Log() << "Output bytes: " << bytes_written;
    util::Log() << "PHAST rasterization complete.";

    util::DumpMemoryStats();
    return EXIT_SUCCESS;
}
catch (const osrm::RuntimeError &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return e.GetCode();
}
catch (const util::exception &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return EXIT_FAILURE;
}
catch (const std::bad_alloc &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    util::Log(logERROR) << "Please provide more memory or consider using a larger swapfile";
    return EXIT_FAILURE;
}
#ifdef _WIN32
catch (const std::exception &e)
{
    util::Log(logERROR) << "[exception] " << e.what();
    return EXIT_FAILURE;
}
#endif
