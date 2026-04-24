#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
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

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

using namespace osrm;

namespace
{
constexpr std::uint32_t SAMPLE_SNAP_CACHE_VERSION = 1;
constexpr std::array<char, 8> SAMPLE_SNAP_CACHE_MAGIC = {'P', 'H', 'S', 'N', 'A', 'P', '1', '\0'};
constexpr std::uint16_t MAX_REACHABLE_U16_INT = 65534;
constexpr std::uint16_t UNREACHABLE_U16_INT = 65535;

struct SampleSnapCacheHeader
{
    std::uint32_t version = SAMPLE_SNAP_CACHE_VERSION;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t expected_resolution = 0;
    std::uint32_t reserved = 0;
    std::uint64_t sample_count = 0;
    std::uint64_t sample_file_size = 0;
    std::int64_t sample_file_mtime_ticks = 0;
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

constexpr std::uint8_t SAMPLE_HINT_HAS_FORWARD = 1U << 0;
constexpr std::uint8_t SAMPLE_HINT_FORWARD_OFFSET_VALID = 1U << 1;
constexpr std::uint8_t SAMPLE_HINT_HAS_REVERSE = 1U << 2;
constexpr std::uint8_t SAMPLE_HINT_REVERSE_OFFSET_VALID = 1U << 3;
constexpr std::size_t SAMPLE_HINT_RECORD_BYTES = 28;

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
           readBinary(input, header.sample_count) && readBinary(input, header.sample_file_size) &&
           readBinary(input, header.sample_file_mtime_ticks);
}

bool readPrimaryHintRecord(std::istream &input, SamplePrimaryHint &hint)
{
    std::int32_t ignored_lon = 0;
    std::int32_t ignored_lat = 0;
    std::int32_t forward_offset_ticks = 0;
    std::int32_t reverse_offset_ticks = 0;
    std::uint8_t flags = 0;
    std::uint8_t reserved0 = 0;
    std::uint16_t reserved1 = 0;

    if (!readBinary(input, ignored_lon) || !readBinary(input, ignored_lat) ||
        !readBinary(input, hint.forward_node_id) || !readBinary(input, hint.reverse_node_id) ||
        !readBinary(input, forward_offset_ticks) || !readBinary(input, reverse_offset_ticks) ||
        !readBinary(input, flags) || !readBinary(input, reserved0) || !readBinary(input, reserved1))
    {
        return false;
    }

    hint.has_forward_state = (flags & SAMPLE_HINT_HAS_FORWARD) != 0;
    hint.forward_offset_valid = (flags & SAMPLE_HINT_FORWARD_OFFSET_VALID) != 0;
    hint.has_reverse_state = (flags & SAMPLE_HINT_HAS_REVERSE) != 0;
    hint.reverse_offset_valid = (flags & SAMPLE_HINT_REVERSE_OFFSET_VALID) != 0;
    hint.forward_offset = to_alias<EdgeWeight>(static_cast<std::int64_t>(forward_offset_ticks));
    hint.reverse_offset = to_alias<EdgeWeight>(static_cast<std::int64_t>(reverse_offset_ticks));
    return true;
}

bool readSampleFileFingerprint(const std::filesystem::path &sample_file_path,
                               std::uint64_t &sample_file_size,
                               std::int64_t &sample_file_mtime_ticks)
{
    std::error_code size_error;
    sample_file_size = std::filesystem::file_size(sample_file_path, size_error);
    if (size_error)
    {
        util::Log(logERROR) << "Failed to read sample file size: " << sample_file_path.string()
                            << " (" << size_error.message() << ")";
        return false;
    }

    std::error_code mtime_error;
    const auto mtime = std::filesystem::last_write_time(sample_file_path, mtime_error);
    if (mtime_error)
    {
        util::Log(logERROR) << "Failed to read sample file mtime: " << sample_file_path.string()
                            << " (" << mtime_error.message() << ")";
        return false;
    }
    sample_file_mtime_ticks = static_cast<std::int64_t>(mtime.time_since_epoch().count());
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

bool writeDenseU16ArtifactFromSnapCache(const std::filesystem::path &sample_cache_path,
                                        const std::filesystem::path &sample_file_path,
                                        const std::uint32_t expected_resolution,
                                        const std::uint32_t connectivity_checksum,
                                        const std::string &metric_name,
                                        const std::optional<EdgeWeight> cap_metric,
                                        const std::vector<EdgeWeight> &costs,
                                        const std::filesystem::path &raster_output_path,
                                        RasterizationStats &stats,
                                        std::size_t &cell_count,
                                        std::size_t &reachable_cells,
                                        std::size_t &unreachable_cells,
                                        std::size_t &bytes_written)
{
    std::uint64_t sample_file_size = 0;
    std::int64_t sample_file_mtime_ticks = 0;
    if (!readSampleFileFingerprint(sample_file_path, sample_file_size, sample_file_mtime_ticks))
    {
        return false;
    }

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
        header.sample_file_size != sample_file_size ||
        header.sample_file_mtime_ticks != sample_file_mtime_ticks)
    {
        util::Log(logERROR) << "Sample snap cache metadata mismatch: " << sample_cache_path.string();
        return false;
    }

    const auto expected_size = SAMPLE_SNAP_CACHE_MAGIC.size() + sizeof(std::uint32_t) * 4 +
                               sizeof(std::uint64_t) * 2 + sizeof(std::int64_t) +
                               header.sample_count * SAMPLE_HINT_RECORD_BYTES;
    std::error_code cache_size_error;
    const auto actual_size = std::filesystem::file_size(sample_cache_path, cache_size_error);
    if (cache_size_error || actual_size != expected_size)
    {
        util::Log(logERROR) << "Sample snap cache size mismatch: " << sample_cache_path.string();
        return false;
    }

    auto output_tmp = raster_output_path;
    output_tmp += ".tmp";
    std::ofstream output(output_tmp, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        util::Log(logERROR) << "Could not open output artifact path: " << output_tmp.string();
        return false;
    }

    stats = {};
    cell_count = static_cast<std::size_t>(header.sample_count);
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
        if (!mapSampleFromPrimaryHint(hint, costs, mapped_cost, stats))
        {
            return false;
        }
        if (mapped_cost == INVALID_EDGE_WEIGHT)
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
    std::filesystem::remove(raster_output_path, remove_error);
    (void)remove_error;
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

    if (!writeStatsMetadata(raster_output_path,
                            cell_count,
                            reachable_cells,
                            unreachable_cells,
                            bytes_written,
                            stats))
    {
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
    if (!phast_config.IsValid())
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Input file: " << phast_config.base_path.string() << ".osrm";

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
                adjacency, task_seeds, task_cap_metric, task_distances, task_upward_settled_nodes) ||
            !osrm::contractor::phast::RunDownwardSweep(
                phast_data, adjacency, task_cap_metric, task_distances, &task_downward_updates))
        {
            return false;
        }
        task_phast_elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                   phast_started)
                .count();
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
    util::Log() << "Pre-synthesis upward arcs: " << adjacency.pre_synthesis_upward_arc_count;
    util::Log() << "Pre-synthesis downward arcs: " << adjacency.pre_synthesis_downward_arc_count;
    util::Log() << "Downward adjacency built from transpose: "
                << (adjacency.downward_built_from_transpose ? "yes" : "no");
    util::Log() << "Derived upward arcs: " << adjacency.up_targets.size();
    util::Log() << "Derived downward arcs: " << adjacency.down_targets.size();

    if (runtime_config.has_task_file)
    {
        if (runtime_metric_kind != osrm::contractor::phast::MetricKind::Duration ||
            selected_orientation != osrm::contractor::PHASTOrientation::Reverse)
        {
            util::Log(logERROR) << "--task-file requires --metric duration and --orientation reverse.";
            return EXIT_FAILURE;
        }

        std::vector<BatchRasterTask> batch_tasks;
        if (!loadBatchRasterTasks(runtime_config.task_file, batch_tasks))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "Batch tasks loaded: " << batch_tasks.size();

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

            if (!writeDenseU16ArtifactFromSnapCache(task_runtime.sample_snap_cache,
                                                    task_runtime.sample_file,
                                                    task_runtime.expected_resolution,
                                                    phast_data.connectivity_checksum,
                                                    metric_name,
                                                    task_cap_metric,
                                                    task_distances,
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

        RasterizationStats rasterization_stats;
        std::size_t cell_count = 0;
        std::size_t reachable_cells = 0;
        std::size_t unreachable_cells = 0;
        std::size_t bytes_written = 0;

        if (!writeDenseU16ArtifactFromSnapCache(runtime_config.sample_snap_cache,
                                                runtime_config.sample_file,
                                                runtime_config.expected_resolution,
                                                phast_data.connectivity_checksum,
                                                metric_name,
                                                cap_metric,
                                                phast_distances,
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
