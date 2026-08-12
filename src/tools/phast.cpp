#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
#include "contractor/iso_adj.hpp"
#include "contractor/ordinal_identity.hpp"
#include "contractor/phast_cap.hpp"
#include "contractor/phast_cli.hpp"
#include "contractor/phast_io.hpp"
#include "contractor/phast_runtime.hpp"
#include "contractor/phast_seeds.hpp"

#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"

#include "osrm/exception.hpp"

#include "util/exception.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"

#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/uuid/detail/sha1.hpp>
#include <h3api.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/mman.h>
#endif

using namespace osrm;

namespace
{
using OrdinalsIdentity = osrm::contractor::phast::OrdinalsIdentity;

constexpr std::uint32_t SAMPLE_SNAP_CACHE_VERSION = 2;
constexpr std::array<char, 8> SAMPLE_SNAP_CACHE_MAGIC = {'P', 'H', 'S', 'N', 'A', 'P', '2', '\0'};
constexpr std::uint16_t MAX_REACHABLE_U16_INT = 65534;
constexpr std::uint16_t UNREACHABLE_U16_INT = 65535;

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

struct SamplePrimaryHint
{
    bool has_forward_state = false;
    bool forward_offset_valid = false;
    std::uint32_t forward_node_id = 0;
    EdgeWeight forward_offset = INVALID_EDGE_WEIGHT;

    bool has_reverse_state = false;
    bool reverse_offset_valid = false;
    std::uint32_t reverse_node_id = 0;
    EdgeWeight reverse_offset = INVALID_EDGE_WEIGHT;
};

struct RasterizationStats
{
    std::size_t mapped_samples = 0;
    std::size_t no_candidate_samples = 0;
    std::size_t no_valid_state_samples = 0;
    std::size_t unreachable_state_samples = 0;
    std::size_t capped_samples = 0;
};

struct BatchRasterTask
{
    std::filesystem::path poi_file;
    double cap_seconds = 0.;
    std::filesystem::path raster_output_path;
};

struct LoadedSampleSnapCache
{
    boost::iostreams::mapped_file_source file;
    std::size_t sample_count = 0;
    const char *records = nullptr;
};

constexpr std::uint8_t SAMPLE_HINT_HAS_FORWARD = 1U << 0;
constexpr std::uint8_t SAMPLE_HINT_FORWARD_OFFSET_VALID = 1U << 1;
constexpr std::uint8_t SAMPLE_HINT_HAS_REVERSE = 1U << 2;
constexpr std::uint8_t SAMPLE_HINT_REVERSE_OFFSET_VALID = 1U << 3;
constexpr std::size_t SAMPLE_HINT_RECORD_BYTES = 28;
constexpr std::size_t SAMPLE_SNAP_CACHE_HEADER_BYTES =
    SAMPLE_SNAP_CACHE_MAGIC.size() + sizeof(std::uint32_t) * 4 + sizeof(std::uint64_t) * 4;
constexpr std::size_t RASTER_CHUNK_RECORDS = 1U << 18;
constexpr std::size_t RASTER_PARALLEL_MIN_RECORDS = RASTER_CHUNK_RECORDS * 8;
constexpr std::size_t MAX_RASTER_THREADS = 4;

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

template <typename T> T readUnaligned(const char *bytes)
{
    T value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

SamplePrimaryHint readPrimaryHintRecord(const char *record)
{
    SamplePrimaryHint hint;
    hint.forward_node_id = readUnaligned<std::uint32_t>(record + 8);
    hint.reverse_node_id = readUnaligned<std::uint32_t>(record + 12);
    const auto forward_offset_ticks = readUnaligned<std::int32_t>(record + 16);
    const auto reverse_offset_ticks = readUnaligned<std::int32_t>(record + 20);
    const auto flags = readUnaligned<std::uint8_t>(record + 24);

    hint.has_forward_state = (flags & SAMPLE_HINT_HAS_FORWARD) != 0;
    hint.forward_offset_valid = (flags & SAMPLE_HINT_FORWARD_OFFSET_VALID) != 0;
    hint.has_reverse_state = (flags & SAMPLE_HINT_HAS_REVERSE) != 0;
    hint.reverse_offset_valid = (flags & SAMPLE_HINT_REVERSE_OFFSET_VALID) != 0;
    hint.forward_offset = to_alias<EdgeWeight>(static_cast<std::int64_t>(forward_offset_ticks));
    hint.reverse_offset = to_alias<EdgeWeight>(static_cast<std::int64_t>(reverse_offset_ticks));
    return hint;
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

bool scanOrdinalsIdentity(const std::filesystem::path &sample_ordinals_path,
                          const std::uint32_t expected_resolution,
                          OrdinalsIdentity &identity)
{
    identity = {};

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

bool loadOrdinalsIdentity(const std::filesystem::path &sample_ordinals_path,
                          const std::uint32_t expected_resolution,
                          const bool validate_contents,
                          OrdinalsIdentity &identity)
{
    std::string error;
    if (!osrm::contractor::phast::ReadOrdinalsIdentity(
            sample_ordinals_path, expected_resolution, identity, error))
    {
        util::Log(logERROR) << error
                            << ". Rebuild the sample snap cache to create a trusted identity.";
        return false;
    }
    if (!validate_contents)
    {
        return true;
    }

    OrdinalsIdentity scanned_identity;
    if (!scanOrdinalsIdentity(sample_ordinals_path, expected_resolution, scanned_identity))
    {
        return false;
    }
    if (!(scanned_identity == identity))
    {
        util::Log(logERROR) << "Ordinal contents do not match identity: "
                            << sample_ordinals_path.string();
        return false;
    }
    return true;
}

std::string trim(const std::string &value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool loadBatchRasterTasks(const std::filesystem::path &task_file, std::vector<BatchRasterTask> &tasks)
{
    std::ifstream input(task_file);
    if (!input)
    {
        util::Log(logERROR) << "Could not open task file: " << task_file.string();
        return false;
    }

    tasks.clear();
    std::string line;
    std::uint64_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const auto row = trim(line);
        if (row.empty() || row[0] == '#')
        {
            continue;
        }

        std::vector<std::string> fields;
        std::string token;
        std::istringstream row_stream(row);
        while (std::getline(row_stream, token, '\t'))
        {
            fields.push_back(token);
        }
        if (fields.size() != 3)
        {
            util::Log(logERROR) << "Invalid task row at " << task_file.string() << ":" << line_number
                                << " expected 3 tab-separated fields: poi_file,cap_seconds,raster_output";
            return false;
        }

        double cap_seconds = 0.;
        std::istringstream cap_parser(trim(fields[1]));
        if (!(cap_parser >> cap_seconds) || !cap_parser.eof() || !std::isfinite(cap_seconds) ||
            cap_seconds < 0.)
        {
            util::Log(logERROR) << "Invalid cap_seconds at " << task_file.string() << ":" << line_number;
            return false;
        }

        const auto poi_file = std::filesystem::path(trim(fields[0]));
        const auto raster_output_path = std::filesystem::path(trim(fields[2]));
        if (poi_file.empty() || raster_output_path.empty())
        {
            util::Log(logERROR) << "Empty path field at " << task_file.string() << ":" << line_number;
            return false;
        }
        tasks.push_back(BatchRasterTask{poi_file, cap_seconds, raster_output_path});
    }

    if (tasks.empty())
    {
        util::Log(logERROR) << "No tasks loaded from task file: " << task_file.string();
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

bool mapSampleFromPrimaryHint(const SamplePrimaryHint &hint,
                              const std::vector<EdgeWeight> &costs,
                              EdgeWeight &mapped_cost,
                              RasterizationStats &stats)
{
    mapped_cost = INVALID_EDGE_WEIGHT;
    EdgeWeight best_candidate_cost = INVALID_EDGE_WEIGHT;

    bool any_valid_state = false;
    bool any_reachable_state = false;
    auto evaluate_hint_state = [&](const bool has_state,
                                   const std::uint32_t node_id,
                                   const bool offset_valid,
                                   const EdgeWeight offset) -> bool
    {
        if (!has_state)
        {
            return true;
        }
        any_valid_state = true;
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
        any_reachable_state = true;
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
    if (mapped_cost != INVALID_EDGE_WEIGHT)
    {
        ++stats.mapped_samples;
    }
    else if (!any_valid_state)
    {
        ++stats.no_valid_state_samples;
    }
    else if (!any_reachable_state)
    {
        ++stats.unreachable_state_samples;
    }

    return true;
}

bool writeStatsMetadata(const std::filesystem::path &output_path,
                        const std::size_t cell_count,
                        const std::size_t reachable_cells,
                        const std::size_t unreachable_cells,
                        const std::size_t bytes_written,
                        const std::int64_t traversal_ms,
                        const std::int64_t rasterization_ms,
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
             << "\"traversal_ms\":" << traversal_ms << ","
             << "\"rasterization_ms\":" << rasterization_ms << ","
             << "\"mapped_samples\":" << rasterization_stats.mapped_samples << ","
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

void adviseSequential(const char *data, const std::size_t size)
{
#if defined(MADV_SEQUENTIAL)
    if (size > 0)
    {
        (void)::madvise(const_cast<char *>(data), size, MADV_SEQUENTIAL);
    }
#else
    (void)data;
    (void)size;
#endif
}

bool loadSampleSnapCache(const std::filesystem::path &sample_cache_path,
                         const std::uint32_t expected_resolution,
                         const std::uint32_t connectivity_checksum,
                         const OrdinalsIdentity &ordinals_identity,
                         LoadedSampleSnapCache &loaded_cache)
{
    const auto load_started = std::chrono::steady_clock::now();
    std::ifstream cache_input(sample_cache_path, std::ios::binary);
    if (!cache_input)
    {
        util::Log(logERROR) << "Could not open sample snap cache: " << sample_cache_path.string();
        return false;
    }

    SampleSnapCacheHeader header;
    if (!readSnapCacheHeader(cache_input, header))
    {
        util::Log(logERROR) << "Invalid sample snap cache header: " << sample_cache_path.string();
        return false;
    }
    if (header.version != SAMPLE_SNAP_CACHE_VERSION ||
        header.expected_resolution != expected_resolution ||
        header.connectivity_checksum != connectivity_checksum ||
        header.sample_count != ordinals_identity.sample_count ||
        header.ordinals_file_size != ordinals_identity.ordinals_file_size ||
        header.ordinals_hash_hi != ordinals_identity.ordinals_hash_hi ||
        header.ordinals_hash_lo != ordinals_identity.ordinals_hash_lo)
    {
        util::Log(logERROR) << "Sample snap cache metadata mismatch: " << sample_cache_path.string();
        return false;
    }

    if (header.sample_count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
        static_cast<std::size_t>(header.sample_count) >
            (std::numeric_limits<std::size_t>::max() - SAMPLE_SNAP_CACHE_HEADER_BYTES) /
                SAMPLE_HINT_RECORD_BYTES)
    {
        util::Log(logERROR) << "Sample snap cache sample_count does not fit into size_t.";
        return false;
    }

    const auto sample_count = static_cast<std::size_t>(header.sample_count);
    const auto expected_size =
        SAMPLE_SNAP_CACHE_HEADER_BYTES + sample_count * SAMPLE_HINT_RECORD_BYTES;
    std::error_code cache_size_error;
    const auto actual_size = std::filesystem::file_size(sample_cache_path, cache_size_error);
    if (cache_size_error || actual_size != expected_size)
    {
        util::Log(logERROR) << "Sample snap cache size mismatch: " << sample_cache_path.string();
        return false;
    }

    cache_input.close();
    loaded_cache.file.close();
    try
    {
        loaded_cache.file.open(sample_cache_path.string());
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to mmap sample snap cache " << sample_cache_path.string()
                            << ": " << e.what();
        return false;
    }
    if (!loaded_cache.file.is_open() || loaded_cache.file.size() != expected_size)
    {
        util::Log(logERROR) << "Could not mmap complete sample snap cache: "
                            << sample_cache_path.string();
        return false;
    }

    adviseSequential(loaded_cache.file.data(), loaded_cache.file.size());
    loaded_cache.sample_count = sample_count;
    loaded_cache.records = loaded_cache.file.data() + SAMPLE_SNAP_CACHE_HEADER_BYTES;

    const auto load_done = std::chrono::steady_clock::now();
    util::Log() << "Sample snap cache mmap load: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(load_done - load_started)
                       .count()
                << " ms, records=" << loaded_cache.sample_count;

    return true;
}

bool writeDenseU16ArtifactFromHints(const LoadedSampleSnapCache &loaded_cache,
                                    const std::string &metric_name,
                                    const std::optional<EdgeWeight> cap_metric,
                                    const std::vector<EdgeWeight> &costs,
                                    const std::int64_t traversal_ms,
                                    const std::filesystem::path &raster_output_path,
                                    RasterizationStats &stats,
                                    std::size_t &cell_count,
                                    std::size_t &reachable_cells,
                                    std::size_t &unreachable_cells,
                                    std::size_t &bytes_written)
{
    const auto raster_started = std::chrono::steady_clock::now();
    auto output_tmp = raster_output_path;
    output_tmp += ".tmp";

    stats = {};
    cell_count = loaded_cache.sample_count;
    reachable_cells = 0;
    unreachable_cells = 0;
    bytes_written = cell_count * sizeof(std::uint16_t);

    std::error_code remove_tmp_error;
    std::filesystem::remove(output_tmp, remove_tmp_error);
    if (remove_tmp_error)
    {
        util::Log(logERROR) << "Could not remove stale raster temp file " << output_tmp.string()
                            << ": " << remove_tmp_error.message();
        return false;
    }

    boost::iostreams::mapped_file mapped_output;
    try
    {
        boost::iostreams::mapped_file_params params;
        params.path = output_tmp.string();
        params.flags = boost::iostreams::mapped_file::readwrite;
        params.new_file_size = bytes_written;
        mapped_output.open(params);
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to mmap raster temp file " << output_tmp.string() << ": "
                            << e.what();
        std::error_code cleanup_error;
        std::filesystem::remove(output_tmp, cleanup_error);
        return false;
    }
    if (!mapped_output.is_open() || mapped_output.size() != bytes_written)
    {
        util::Log(logERROR) << "Could not mmap complete raster temp file: " << output_tmp.string();
        mapped_output.close();
        std::error_code cleanup_error;
        std::filesystem::remove(output_tmp, cleanup_error);
        return false;
    }
    adviseSequential(mapped_output.data(), mapped_output.size());
    auto *const output_data = mapped_output.data();

    struct ThreadResult
    {
        RasterizationStats stats;
        std::size_t reachable_cells = 0;
        std::size_t unreachable_cells = 0;
    };

    const auto hardware_threads =
        static_cast<std::size_t>(std::max(1U, std::thread::hardware_concurrency()));
    const auto available_chunks =
        std::max<std::size_t>(1, (cell_count + RASTER_CHUNK_RECORDS - 1) / RASTER_CHUNK_RECORDS);
    const auto worker_count = cell_count < RASTER_PARALLEL_MIN_RECORDS
                                  ? std::size_t{1}
                                  : std::min({MAX_RASTER_THREADS, hardware_threads, available_chunks});
    const auto records_per_worker = (cell_count + worker_count - 1) / worker_count;
    const bool duration_metric = metric_name == "duration";
    std::vector<ThreadResult> thread_results(worker_count);
    std::atomic<bool> failed{false};

    auto rasterize_range = [&](const std::size_t worker_index)
    {
        ThreadResult result;
        const auto begin = worker_index * records_per_worker;
        const auto end = std::min(cell_count, begin + records_per_worker);
        for (auto chunk_begin = begin; chunk_begin < end && !failed.load(std::memory_order_relaxed);
             chunk_begin += RASTER_CHUNK_RECORDS)
        {
            const auto chunk_end = std::min(end, chunk_begin + RASTER_CHUNK_RECORDS);
            for (auto index = chunk_begin; index < chunk_end; ++index)
            {
                const auto hint = readPrimaryHintRecord(
                    loaded_cache.records + index * SAMPLE_HINT_RECORD_BYTES);
                EdgeWeight mapped_cost = INVALID_EDGE_WEIGHT;
                if (!mapSampleFromPrimaryHint(hint, costs, mapped_cost, result.stats))
                {
                    failed.store(true, std::memory_order_relaxed);
                    break;
                }
                if (mapped_cost == INVALID_EDGE_WEIGHT)
                {
                    ++result.stats.no_candidate_samples;
                }

                std::uint16_t encoded = UNREACHABLE_U16_INT;
                if (mapped_cost == INVALID_EDGE_WEIGHT)
                {
                    ++result.unreachable_cells;
                }
                else if (cap_metric.has_value() && mapped_cost > cap_metric.value())
                {
                    ++result.stats.capped_samples;
                    ++result.unreachable_cells;
                }
                else
                {
                    const auto ticks = from_alias<std::int64_t>(mapped_cost);
                    const auto seconds = duration_metric ? static_cast<double>(ticks) / 10.0
                                                         : static_cast<double>(ticks);
                    const auto rounded = std::nearbyint(seconds);
                    if (!std::isfinite(rounded) || rounded < 0.0)
                    {
                        ++result.unreachable_cells;
                    }
                    else
                    {
                        const auto clipped =
                            std::min<double>(rounded, static_cast<double>(MAX_REACHABLE_U16_INT));
                        encoded = static_cast<std::uint16_t>(clipped);
                        ++result.reachable_cells;
                    }
                }

                auto *output = output_data + index * sizeof(std::uint16_t);
                output[0] = static_cast<char>(encoded & 0xFF);
                output[1] = static_cast<char>((encoded >> 8) & 0xFF);
            }
        }
        thread_results[worker_index] = result;
    };

    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    try
    {
        for (const auto worker_index : util::irange<std::size_t>(1, worker_count))
        {
            workers.emplace_back(rasterize_range, worker_index);
        }
    }
    catch (const std::exception &e)
    {
        failed.store(true, std::memory_order_relaxed);
        for (auto &worker : workers)
        {
            worker.join();
        }
        util::Log(logERROR) << "Failed to start raster worker: " << e.what();
        mapped_output.close();
        std::error_code cleanup_error;
        std::filesystem::remove(output_tmp, cleanup_error);
        return false;
    }
    rasterize_range(0);
    for (auto &worker : workers)
    {
        worker.join();
    }

    for (const auto &result : thread_results)
    {
        stats.mapped_samples += result.stats.mapped_samples;
        stats.no_candidate_samples += result.stats.no_candidate_samples;
        stats.no_valid_state_samples += result.stats.no_valid_state_samples;
        stats.unreachable_state_samples += result.stats.unreachable_state_samples;
        stats.capped_samples += result.stats.capped_samples;
        reachable_cells += result.reachable_cells;
        unreachable_cells += result.unreachable_cells;
    }

    if (failed.load(std::memory_order_relaxed))
    {
        util::Log(logERROR) << "Failed rasterizing raster artifact: " << output_tmp.string();
        mapped_output.close();
        std::error_code cleanup_error;
        std::filesystem::remove(output_tmp, cleanup_error);
        return false;
    }
    mapped_output.close();

    if (stats.no_candidate_samples > 0)
    {
        util::Log(logWARNING) << "Primary snap hint could not map " << stats.no_candidate_samples
                              << " samples; they were marked unreachable.";
    }

    std::error_code rename_error;
    std::filesystem::rename(output_tmp, raster_output_path, rename_error);
    if (rename_error)
    {
        util::Log(logERROR) << "Failed to finalize raster artifact " << raster_output_path.string()
                            << ": " << rename_error.message();
        std::error_code cleanup_error;
        std::filesystem::remove(output_tmp, cleanup_error);
        (void)cleanup_error;
        return false;
    }

    const auto raster_done = std::chrono::steady_clock::now();
    const auto rasterization_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(raster_done - raster_started).count();
    util::Log() << "Snap cache rasterization: "
                << rasterization_ms
                << " ms, workers=" << worker_count << ", chunk_records=" << RASTER_CHUNK_RECORDS;

    if (!writeStatsMetadata(raster_output_path,
                            cell_count,
                            reachable_cells,
                            unreachable_cells,
                            bytes_written,
                            traversal_ms,
                            rasterization_ms,
                            stats))
    {
        return false;
    }

    return true;
}

bool writeDenseU16ArtifactFromSnapCache(const std::filesystem::path &sample_cache_path,
                                        const std::uint32_t expected_resolution,
                                        const std::uint32_t connectivity_checksum,
                                        const OrdinalsIdentity &ordinals_identity,
                                        const std::string &metric_name,
                                        const std::optional<EdgeWeight> cap_metric,
                                        const std::vector<EdgeWeight> &costs,
                                        const std::int64_t traversal_ms,
                                        const std::filesystem::path &raster_output_path,
                                        RasterizationStats &stats,
                                        std::size_t &cell_count,
                                        std::size_t &reachable_cells,
                                        std::size_t &unreachable_cells,
                                        std::size_t &bytes_written)
{
    LoadedSampleSnapCache loaded_cache;
    if (!loadSampleSnapCache(sample_cache_path,
                             expected_resolution,
                             connectivity_checksum,
                             ordinals_identity,
                             loaded_cache))
    {
        return false;
    }
    return writeDenseU16ArtifactFromHints(loaded_cache,
                                          metric_name,
                                          cap_metric,
                                          costs,
                                          traversal_ms,
                                          raster_output_path,
                                          stats,
                                          cell_count,
                                          reachable_cells,
                                          unreachable_cells,
                                          bytes_written);
}

// ---------------------------------------------------------------------------
// Lean custom-isochrone path (--iso-adj): capped Dijkstra over a memory-mapped
// edge-based adjacency. Reuses the proven source snapping (BuildSeeds via an
// mmap'd facade) and the dense-u16 rasterizer, replacing only the whole-graph
// PHAST sweep with a region-bounded Dijkstra. See src/tools/iso_adj.cpp for the
// exporter and the on-disk format.
// ---------------------------------------------------------------------------

bool RunIsoAdjMode(const osrm::contractor::phast::PhastConfig &phast_config,
                   const osrm::contractor::phast::RuntimeConfig &runtime_config)
{
    namespace phast = osrm::contractor::phast;

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(phast_config.GetPath(".osrm.properties"), properties);
    const std::string metric_name = properties.GetWeightName();

    phast::MetricKind runtime_metric_kind = phast::MetricKind::Weight;
    if (!phast::ParseMetricKind(runtime_config.metric, metric_name, "--metric", runtime_metric_kind))
    {
        return false;
    }
    phast::MetricKind seed_metric_kind = phast::MetricKind::Weight;
    if (!phast::ParseMetricKind(
            runtime_config.seed_metric, metric_name, "--seed-metric", seed_metric_kind))
    {
        return false;
    }
    if (seed_metric_kind != runtime_metric_kind && metric_name != "duration")
    {
        util::Log(logERROR) << "--seed-metric must match --metric unless weight_name=duration.";
        return false;
    }

    phast::IsoAdj adj;
    if (!phast::LoadIsoAdj(runtime_config.iso_adj_path, adj))
    {
        return false;
    }
    const auto &metadata = adj.metadata;
    if (metadata.metric_name != metric_name)
    {
        util::Log(logERROR) << "Metric name mismatch between .osrm.properties and iso-adj.";
        return false;
    }
    if (metadata.metric_kind != runtime_metric_kind)
    {
        util::Log(logERROR) << "iso-adj metric kind differs from --metric.";
        return false;
    }

    const std::size_t selected_exclude_index = metadata.exclude_index;
    if (runtime_config.has_exclude_index &&
        runtime_config.exclude_index != selected_exclude_index)
    {
        util::Log(logERROR) << "--exclude=" << runtime_config.exclude_index
                            << " does not match iso-adj exclude index "
                            << selected_exclude_index << ".";
        return false;
    }
    if (selected_exclude_index >= properties.excludable_classes.size())
    {
        util::Log(logERROR) << "iso-adj exclude index " << selected_exclude_index
                            << " is out of range for this profile.";
        return false;
    }

    const auto selected_orientation = metadata.orientation;
    if (runtime_config.has_orientation &&
        runtime_config.orientation != phast::OrientationToString(selected_orientation))
    {
        util::Log(logERROR) << "--orientation=" << runtime_config.orientation
                            << " does not match iso-adj orientation "
                            << phast::OrientationToString(selected_orientation) << ".";
        return false;
    }

    // Snapping only: mmap the facade so the CH graph (.hsgr) is mapped but never
    // faulted in; the rtree + geometry pages touched by BuildSeeds stay tiny.
    std::shared_ptr<const phast::CHDataFacade> facade;
    if (!phast::LoadCHFacade(phast_config.base_path,
                             metric_name,
                             selected_exclude_index,
                             facade,
                             /*use_mmap=*/true))
    {
        return false;
    }
    if (facade->GetCheckSum() != metadata.connectivity_checksum)
    {
        util::Log(logERROR) << "Facade connectivity checksum (" << facade->GetCheckSum()
                            << ") differs from iso-adj (" << metadata.connectivity_checksum
                            << "). Re-export osrm-iso-adj for this dataset.";
        return false;
    }
    if (facade->GetNumberOfNodes() != metadata.node_count)
    {
        util::Log(logERROR) << "Facade node count (" << facade->GetNumberOfNodes()
                            << ") differs from iso-adj (" << metadata.node_count << ").";
        return false;
    }
    if (std::string{facade->GetWeightName()} != metadata.metric_name)
    {
        util::Log(logERROR) << "Facade weight name differs from iso-adj.";
        return false;
    }
    if (phast::BuildIsoAdjBuildID(facade->GetTimestamp(), metadata) != metadata.build_id)
    {
        util::Log(logERROR)
            << "iso-adj build identity differs from the loaded OSRM base. Re-export it.";
        return false;
    }

    OrdinalsIdentity ordinals_identity;
    if (!loadOrdinalsIdentity(
            runtime_config.sample_ordinals,
            runtime_config.expected_resolution,
            runtime_config.validate_sample_ordinals,
            ordinals_identity))
    {
        return false;
    }

    std::vector<phast::PHASTSeed> seeds;
    phast::SeedBuildStats seed_stats;
    if (!phast::BuildSeeds(runtime_config,
                           static_cast<std::uint32_t>(metadata.node_count),
                           seed_metric_kind,
                           selected_orientation,
                           *facade,
                           seeds,
                           seed_stats) ||
        !phast::ValidateSeeds(static_cast<std::uint32_t>(metadata.node_count), seeds))
    {
        return false;
    }

    std::optional<EdgeWeight> cap_metric;
    if (!phast::ParseTraversalCap(runtime_config, properties, runtime_metric_kind, cap_metric))
    {
        return false;
    }

    const auto t_search_start = std::chrono::steady_clock::now();
    std::vector<EdgeWeight> distances;
    phast::RunCappedDijkstraIsoAdj(adj, seeds, cap_metric, distances);
    const auto t_search_done = std::chrono::steady_clock::now();
    const auto search_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t_search_done - t_search_start)
            .count();
    util::Log() << "[iso-adj] capped Dijkstra: " << search_ms
                << " ms, reachable nodes: "
                << osrm::contractor::phast::CountReachable(distances);

    RasterizationStats stats;
    std::size_t cell_count = 0;
    std::size_t reachable_cells = 0;
    std::size_t unreachable_cells = 0;
    std::size_t bytes_written = 0;
    if (!writeDenseU16ArtifactFromSnapCache(runtime_config.sample_snap_cache,
                                            runtime_config.expected_resolution,
                                            metadata.connectivity_checksum,
                                            ordinals_identity,
                                            metric_name,
                                            cap_metric,
                                            distances,
                                            search_ms,
                                            runtime_config.raster_output_path,
                                            stats,
                                            cell_count,
                                            reachable_cells,
                                            unreachable_cells,
                                            bytes_written))
    {
        return false;
    }

    util::Log() << "[iso-adj] raster cells=" << cell_count << " reachable=" << reachable_cells
                << " unreachable=" << unreachable_cells << " bytes=" << bytes_written;
    return true;
}
} // namespace

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();

    std::string verbosity;
    osrm::contractor::phast::PhastConfig phast_config;
    osrm::contractor::phast::RuntimeConfig runtime_config;

    const auto parse_result =
        osrm::contractor::phast::ParseArguments(argc, argv, verbosity, phast_config, runtime_config);
    if (parse_result == osrm::contractor::phast::ReturnCode::Fail)
    {
        return EXIT_FAILURE;
    }
    if (parse_result == osrm::contractor::phast::ReturnCode::Exit)
    {
        return EXIT_SUCCESS;
    }

    util::LogPolicy::GetInstance().SetLevel(verbosity);
    phast_config.UseDefaultOutputNames(phast_config.base_path);
    if (!runtime_config.has_iso_adj && !phast_config.IsValid())
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Input file: " << phast_config.base_path.string() << ".osrm";

    // Lean custom-isochrone path: capped Dijkstra over a memory-mapped edge-based
    // adjacency. Skips the whole-graph CH load + PHAST sweep entirely; the
    // existing batch/core path below is untouched.
    if (runtime_config.has_iso_adj)
    {
        return RunIsoAdjMode(phast_config, runtime_config) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(phast_config.GetPath(".osrm.properties"), properties);
    const std::string metric_name = properties.GetWeightName();

    std::unordered_map<std::string, osrm::contractor::ContractedMetric> metrics = {{metric_name, {}}};
    std::uint32_t hsgr_checksum = 0;
    osrm::contractor::files::readGraph(phast_config.GetPath(".osrm.hsgr"), metrics, hsgr_checksum);

    osrm::contractor::PhastData phast_data;
    osrm::contractor::files::readPhast(phast_config.GetPath(".osrm.phast"), phast_data);

    const auto &metric = metrics.at(metric_name);
    const auto &graph = metric.graph;

    if (!osrm::contractor::phast::ValidateMetadata(metric_name, metric, hsgr_checksum, phast_data) ||
        !osrm::contractor::phast::ValidateOrdering(phast_data))
    {
        return EXIT_FAILURE;
    }

    osrm::contractor::phast::MetricKind runtime_metric_kind = osrm::contractor::phast::MetricKind::Weight;
    if (!osrm::contractor::phast::ParseMetricKind(
            runtime_config.metric, metric_name, "--metric", runtime_metric_kind))
    {
        return EXIT_FAILURE;
    }

    osrm::contractor::phast::MetricKind seed_metric_kind = osrm::contractor::phast::MetricKind::Weight;
    if (!osrm::contractor::phast::ParseMetricKind(
            runtime_config.seed_metric, metric_name, "--seed-metric", seed_metric_kind))
    {
        return EXIT_FAILURE;
    }
    if (seed_metric_kind != runtime_metric_kind && metric_name != "duration")
    {
        util::Log(logERROR) << "--seed-metric must match --metric unless weight_name=duration.";
        return EXIT_FAILURE;
    }

    osrm::contractor::phast::OutputFormatKind output_format_kind =
        osrm::contractor::phast::OutputFormatKind::PhastFieldV1;
    if (!osrm::contractor::phast::ParseOutputFormatKind(runtime_config.output_format, output_format_kind))
    {
        util::Log(logERROR) << "Unsupported output format: " << runtime_config.output_format;
        return EXIT_FAILURE;
    }

    std::size_t selected_exclude_index = phast_data.exclude_index;
    if (!osrm::contractor::phast::ResolveExcludeIndex(
            runtime_config, phast_data, selected_exclude_index))
    {
        return EXIT_FAILURE;
    }

    osrm::contractor::PHASTOrientation selected_orientation = phast_data.orientation;
    if (!osrm::contractor::phast::ResolveOrientation(runtime_config, phast_data, selected_orientation))
    {
        return EXIT_FAILURE;
    }

    std::shared_ptr<const osrm::contractor::phast::CHDataFacade> facade;
    if (!osrm::contractor::phast::LoadCHFacade(
            phast_config.base_path, metric_name, selected_exclude_index, facade) ||
        !osrm::contractor::phast::ValidateFacadeMetadata(*facade, phast_data, metric_name))
    {
        return EXIT_FAILURE;
    }

    const auto &selected_edge_filter = metric.edge_filter[selected_exclude_index];
    osrm::contractor::phast::DerivedAdjacency adjacency;
    if (!osrm::contractor::phast::DeriveAdjacency(graph,
                                                  selected_edge_filter,
                                                  phast_data,
                                                  selected_orientation,
                                                  runtime_metric_kind,
                                                  adjacency))
    {
        return EXIT_FAILURE;
    }

    auto run_phast_task = [&](const osrm::contractor::phast::RuntimeConfig &task_runtime,
                              std::vector<EdgeWeight> &task_distances,
                              std::optional<EdgeWeight> &task_cap_metric,
                              osrm::contractor::phast::SeedBuildStats &task_seed_stats,
                              std::size_t &task_upward_settled_nodes,
                              std::size_t &task_downward_updates,
                              std::int64_t &task_phast_elapsed_ms) -> bool
    {
        std::vector<osrm::contractor::phast::PHASTSeed> task_seeds;
        if (!osrm::contractor::phast::BuildSeeds(task_runtime,
                                                 phast_data.node_count,
                                                 seed_metric_kind,
                                                 selected_orientation,
                                                 *facade,
                                                 task_seeds,
                                                 task_seed_stats) ||
            !osrm::contractor::phast::ValidateSeeds(phast_data.node_count, task_seeds))
        {
            return false;
        }

        if (!osrm::contractor::phast::ParseTraversalCap(
                task_runtime, properties, runtime_metric_kind, task_cap_metric))
        {
            return false;
        }

        const auto phast_started = std::chrono::steady_clock::now();
        if (!osrm::contractor::phast::RunUpwardSearch(
                adjacency, task_seeds, task_cap_metric, task_distances, task_upward_settled_nodes))
        {
            return false;
        }
        if (!osrm::contractor::phast::RunDownwardSweep(
                phast_data, adjacency, task_cap_metric, task_distances, &task_downward_updates))
        {
            return false;
        }
        const auto downward_done = std::chrono::steady_clock::now();
        task_phast_elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(downward_done - phast_started).count();
        return true;
    };

    util::Log() << "CH metric: " << metric_name;
    util::Log() << "PHAST nodes: " << phast_data.node_count;
    util::Log() << "Runtime metric: "
                << (runtime_metric_kind == osrm::contractor::phast::MetricKind::Weight ? "weight"
                                                                                        : "duration");
    util::Log() << "Runtime orientation: "
                << osrm::contractor::phast::OrientationToString(selected_orientation);
    util::Log() << "Runtime exclude index: " << selected_exclude_index;
    util::Log() << "Runtime upward arcs: " << adjacency.pre_synthesis_upward_arc_count;
    util::Log() << "Runtime downward arcs: " << adjacency.pre_synthesis_downward_arc_count;
    util::Log() << "Downward adjacency built from transpose: "
                << (adjacency.downward_built_from_transpose ? "yes" : "no");
    util::Log() << "Derived upward arcs: " << adjacency.up_targets.size();
    util::Log() << "Derived downward arcs: " << adjacency.down_targets.size();

    std::optional<OrdinalsIdentity> ordinals_identity;
    auto ensure_ordinals_identity = [&]() -> bool
    {
        if (ordinals_identity.has_value())
        {
            return true;
        }
        OrdinalsIdentity loaded_identity;
        if (!loadOrdinalsIdentity(runtime_config.sample_ordinals,
                                  runtime_config.expected_resolution,
                                  runtime_config.validate_sample_ordinals,
                                  loaded_identity))
        {
            return false;
        }
        util::Log() << "Sample ordinals rows: " << loaded_identity.sample_count;
        ordinals_identity = loaded_identity;
        return true;
    };

    if (runtime_config.has_task_file)
    {
        if (runtime_metric_kind != osrm::contractor::phast::MetricKind::Duration ||
            selected_orientation != osrm::contractor::PHASTOrientation::Reverse)
        {
            util::Log(logERROR) << "--task-file requires --metric duration and --orientation reverse.";
            return EXIT_FAILURE;
        }
        if (!ensure_ordinals_identity())
        {
            return EXIT_FAILURE;
        }

        std::vector<BatchRasterTask> batch_tasks;
        if (!loadBatchRasterTasks(runtime_config.task_file, batch_tasks))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "Batch tasks loaded: " << batch_tasks.size();

        LoadedSampleSnapCache loaded_snap_cache;
        if (!loadSampleSnapCache(runtime_config.sample_snap_cache,
                                 runtime_config.expected_resolution,
                                 phast_data.connectivity_checksum,
                                 ordinals_identity.value(),
                                 loaded_snap_cache))
        {
            return EXIT_FAILURE;
        }

        std::int64_t batch_phast_elapsed_ms = 0;
        for (const auto index : util::irange<std::size_t>(0, batch_tasks.size()))
        {
            const auto &batch_task = batch_tasks[index];
            osrm::contractor::phast::RuntimeConfig task_runtime = runtime_config;
            task_runtime.poi_file = batch_task.poi_file;
            task_runtime.has_poi_file = true;
            task_runtime.seed_nodes.clear();
            task_runtime.seed_file.clear();
            task_runtime.has_seed_file = false;
            task_runtime.has_cap_weight = false;
            task_runtime.cap_weight = 0;
            task_runtime.has_cap_seconds = true;
            task_runtime.cap_seconds = batch_task.cap_seconds;

            std::vector<EdgeWeight> task_distances;
            std::optional<EdgeWeight> task_cap_metric;
            osrm::contractor::phast::SeedBuildStats task_seed_stats;
            std::size_t task_upward_settled_nodes = 0;
            std::size_t task_downward_updates = 0;
            std::int64_t task_phast_elapsed_ms = 0;

            if (!run_phast_task(task_runtime,
                                task_distances,
                                task_cap_metric,
                                task_seed_stats,
                                task_upward_settled_nodes,
                                task_downward_updates,
                                task_phast_elapsed_ms))
            {
                return EXIT_FAILURE;
            }
            batch_phast_elapsed_ms += task_phast_elapsed_ms;

            RasterizationStats rasterization_stats;
            std::size_t cell_count = 0;
            std::size_t reachable_cells = 0;
            std::size_t unreachable_cells = 0;
            std::size_t bytes_written = 0;

            if (!writeDenseU16ArtifactFromHints(loaded_snap_cache,
                                                metric_name,
                                                task_cap_metric,
                                                task_distances,
                                                task_phast_elapsed_ms,
                                                batch_task.raster_output_path,
                                                rasterization_stats,
                                                cell_count,
                                                reachable_cells,
                                                unreachable_cells,
                                                bytes_written))
            {
                return EXIT_FAILURE;
            }

            util::Log() << "Task " << (index + 1) << "/" << batch_tasks.size()
                        << " POIs parsed: " << task_seed_stats.poi_count;
            util::Log() << "Task " << (index + 1) << "/" << batch_tasks.size()
                        << " PHAST runtime: " << task_phast_elapsed_ms << " ms";
            util::Log() << "Task " << (index + 1) << "/" << batch_tasks.size()
                        << " Wrote raster output: " << batch_task.raster_output_path.string();
        }

        util::Log() << "Batch PHAST runtime total: " << batch_phast_elapsed_ms << " ms";
        util::DumpMemoryStats();
        return EXIT_SUCCESS;
    }

    std::vector<EdgeWeight> phast_distances;
    std::optional<EdgeWeight> cap_metric;
    osrm::contractor::phast::SeedBuildStats seed_stats;
    std::size_t upward_settled_nodes = 0;
    std::size_t downward_updates = 0;
    std::int64_t phast_elapsed = 0;
    if (!run_phast_task(runtime_config,
                        phast_distances,
                        cap_metric,
                        seed_stats,
                        upward_settled_nodes,
                        downward_updates,
                        phast_elapsed))
    {
        return EXIT_FAILURE;
    }

    if (runtime_config.has_poi_file)
    {
        util::Log() << "POIs parsed: " << seed_stats.poi_count;
        util::Log() << "Snap cache hits: " << seed_stats.cache_hits;
        util::Log() << "Big-component snap fallbacks: " << seed_stats.big_component_fallbacks;
    }
    if (cap_metric.has_value())
    {
        util::Log() << "Traversal cap: " << cap_metric.value() << " ticks";
    }
    else
    {
        util::Log() << "Traversal cap: none";
    }
    util::Log() << "Upward search settled nodes: " << upward_settled_nodes;
    util::Log() << "Downward sweep updates: " << downward_updates;
    util::Log() << "PHAST reachable nodes: "
                << osrm::contractor::phast::CountReachable(phast_distances)
                << ", max distance: " << osrm::contractor::phast::MaxDistance(phast_distances)
                << " ticks";
    util::Log() << "PHAST runtime: " << phast_elapsed << " ms";

    if (runtime_config.has_raster_output_path)
    {
        if (runtime_metric_kind != osrm::contractor::phast::MetricKind::Duration ||
            selected_orientation != osrm::contractor::PHASTOrientation::Reverse)
        {
            util::Log(logERROR) << "--raster-output requires --metric duration and --orientation reverse.";
            return EXIT_FAILURE;
        }
        if (!ensure_ordinals_identity())
        {
            return EXIT_FAILURE;
        }

        RasterizationStats rasterization_stats;
        std::size_t cell_count = 0;
        std::size_t reachable_cells = 0;
        std::size_t unreachable_cells = 0;
        std::size_t bytes_written = 0;

        if (!writeDenseU16ArtifactFromSnapCache(runtime_config.sample_snap_cache,
                                                runtime_config.expected_resolution,
                                                phast_data.connectivity_checksum,
                                                ordinals_identity.value(),
                                                metric_name,
                                                cap_metric,
                                                phast_distances,
                                                phast_elapsed,
                                                runtime_config.raster_output_path,
                                                rasterization_stats,
                                                cell_count,
                                                reachable_cells,
                                                unreachable_cells,
                                                bytes_written))
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
        util::Log() << "Wrote raster output: " << runtime_config.raster_output_path.string();

        util::DumpMemoryStats();
        return EXIT_SUCCESS;
    }

    const auto output_path = runtime_config.has_output_path
                                 ? runtime_config.output_path
                                 : osrm::contractor::phast::DefaultOutputPath(
                                       phast_config.base_path, output_format_kind);

    const auto poi_count = runtime_config.has_poi_file ? seed_stats.poi_count : 0;
    const osrm::contractor::phast::OutputMetadata output_metadata{
        metric_name,
        runtime_metric_kind,
        selected_orientation,
        selected_exclude_index,
        phast_data.connectivity_checksum,
        poi_count,
    };

    if (!osrm::contractor::phast::WriteOutput(
            output_path, output_format_kind, phast_distances, cap_metric, output_metadata))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Output format: " << runtime_config.output_format;
    util::Log() << "Wrote PHAST field output: " << output_path.string();

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
