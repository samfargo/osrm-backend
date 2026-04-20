#include "engine/approach.hpp"
#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/process_memory_allocator.hpp"
#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/exception.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"
#include "osrm/table_parameters.hpp"
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
    std::filesystem::path sample_file_path;
    std::filesystem::path output_path;
    std::filesystem::path oracle_poi_file;
    std::filesystem::path oracle_mismatch_report_path;
    std::uint32_t expected_resolution = 9;
    std::uint32_t oracle_max_samples = 256;
    std::int32_t oracle_tolerance_ticks = 15;
    std::uint32_t oracle_trace_limit = 10;
    bool oracle_enabled = false;
    bool oracle_strict_endpoints = false;
    bool oracle_report_all = false;
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

struct SamplePoint
{
    std::string h3_id;
    std::uint32_t res = 0;
    std::uint32_t sample_index = 0;
    util::Coordinate coordinate;
};

struct CellAggregate
{
    std::uint32_t res = 0;
    std::size_t sample_count = 0;
    std::size_t reachable_count = 0;
    EdgeWeight min_cost = INVALID_EDGE_WEIGHT;
};

struct RasterizationStats
{
    std::size_t mapped_samples = 0;
    std::size_t fallback_candidate_used_samples = 0;
    std::size_t no_candidate_samples = 0;
    std::size_t no_valid_state_samples = 0;
    std::size_t unreachable_state_samples = 0;
};

struct OracleStats
{
    std::size_t compared_samples = 0;
    std::size_t reachable_pairs = 0;
    std::size_t unreachable_pairs = 0;
    std::size_t mismatches = 0;
    std::int64_t max_abs_error_ticks = 0;
};

struct OracleComparison
{
    std::size_t sample_position = 0;
    std::size_t sample_index = 0;
    bool raster_reachable = false;
    bool oracle_reachable = false;
    std::int64_t raster_ticks = 0;
    std::int64_t oracle_ticks = 0;
    std::int64_t delta_ticks = 0;
    std::int64_t abs_error_ticks = 0;
    bool mismatch = false;
    std::string mismatch_reason;
    std::vector<std::optional<std::int64_t>> oracle_ticks_by_poi;
};

struct StrictSeedEndpoint
{
    std::size_t poi_index = 0;
    bool forward_state = false;
    NodeID state_node_id = SPECIAL_SEGMENTID;
    EdgeWeight seed_offset = INVALID_EDGE_WEIGHT;
    util::Coordinate coordinate;
    engine::PhantomNode restricted_phantom;
};

struct StrictOracleStats
{
    std::size_t compared_samples = 0;
    std::size_t reachable_pairs = 0;
    std::size_t unreachable_pairs = 0;
    std::size_t mismatches = 0;
    std::size_t coordinate_mismatches_resolved = 0;
    std::int64_t max_abs_error_ticks = 0;
    std::size_t state_compared_samples = 0;
    std::size_t state_reachable_pairs = 0;
    std::size_t state_unreachable_pairs = 0;
    std::size_t state_mismatches = 0;
    std::int64_t state_max_abs_error_ticks = 0;
    std::size_t endpoint_semantics_mismatch_samples = 0;
};

struct SampleMappingTrace
{
    bool has_snap_candidates = false;
    std::size_t snap_candidate_count = 0;
    std::size_t chosen_candidate_index = std::numeric_limits<std::size_t>::max();
    NodeID chosen_forward_segment_id = SPECIAL_SEGMENTID;
    bool chosen_forward_segment_enabled = false;
    NodeID chosen_reverse_segment_id = SPECIAL_SEGMENTID;
    bool chosen_reverse_segment_enabled = false;
    bool has_chosen_state = false;
    NodeID chosen_state_node_id = SPECIAL_SEGMENTID;
    bool chosen_state_forward = false;
    EdgeWeight chosen_state_cost = INVALID_EDGE_WEIGHT;
    EdgeWeight chosen_source_offset = INVALID_EDGE_WEIGHT;
    EdgeWeight chosen_total_cost = INVALID_EDGE_WEIGHT;
};

using CHDataFacade =
    engine::datafacade::ContiguousInternalMemoryDataFacade<engine::routing_algorithms::ch::Algorithm>;

constexpr std::uint32_t PHASTFIELD_SCHEMA_VERSION = 1;
constexpr std::size_t MAX_SNAP_CANDIDATES = 8;

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

std::vector<std::string> splitCommaLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::string token;
    std::istringstream stream(line);
    while (std::getline(stream, token, ','))
    {
        fields.push_back(trim(token));
    }
    return fields;
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
        "sample-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.sample_file_path),
        "CSV file with rows: h3_id,res,sample_index,lon,lat")(
        "output",
        boost::program_options::value<std::filesystem::path>(&runtime_config.output_path),
        "Output CSV path for per-cell costs")(
        "resolution",
        boost::program_options::value<std::uint32_t>(&runtime_config.expected_resolution)
            ->default_value(9),
        "Expected H3 resolution in sample file (default 9)")(
        "oracle-poi-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.oracle_poi_file),
        "Optional POI coordinate file (lon,lat per line) to run per-sample oracle validation "
        "against OSRM Table durations")(
        "oracle-max-samples",
        boost::program_options::value<std::uint32_t>(&runtime_config.oracle_max_samples)
            ->default_value(256),
        "Maximum sample points to validate in oracle mode (0 = all)")(
        "oracle-tolerance-ticks",
        boost::program_options::value<std::int32_t>(&runtime_config.oracle_tolerance_ticks)
            ->default_value(15),
        "Allowed absolute error in decisecond ticks between rasterized and oracle durations")(
        "oracle-mismatch-report",
        boost::program_options::value<std::filesystem::path>(&runtime_config.oracle_mismatch_report_path),
        "Optional CSV path for oracle mismatches")(
        "oracle-trace-limit",
        boost::program_options::value<std::uint32_t>(&runtime_config.oracle_trace_limit)
            ->default_value(10),
        "Maximum number of mismatch/pass traces to log for oracle debugging")(
        "oracle-strict-endpoints",
        "Also run strict endpoint oracle pinned to the same snapped source/seed states")(
        "oracle-report-all",
        "Write all oracle comparisons (not only mismatches) to --oracle-mismatch-report");

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
    if (!option_variables.contains("phastfield") || runtime_config.phastfield_path.empty())
    {
        util::Log(logERROR) << "--phastfield is required.";
        return return_code::fail;
    }
    if (!option_variables.contains("sample-file") || runtime_config.sample_file_path.empty())
    {
        util::Log(logERROR) << "--sample-file is required.";
        return return_code::fail;
    }
    if (!option_variables.contains("output") || runtime_config.output_path.empty())
    {
        util::Log(logERROR) << "--output is required.";
        return return_code::fail;
    }
    if (runtime_config.expected_resolution == 0)
    {
        util::Log(logERROR) << "--resolution must be > 0.";
        return return_code::fail;
    }
    if (runtime_config.oracle_tolerance_ticks < 0)
    {
        util::Log(logERROR) << "--oracle-tolerance-ticks must be >= 0.";
        return return_code::fail;
    }
    runtime_config.oracle_enabled =
        option_variables.contains("oracle-poi-file") && !runtime_config.oracle_poi_file.empty();
    runtime_config.oracle_strict_endpoints = option_variables.contains("oracle-strict-endpoints");
    runtime_config.oracle_report_all = option_variables.contains("oracle-report-all");
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

bool loadSamplePoints(const std::filesystem::path &path,
                      const std::uint32_t expected_resolution,
                      std::vector<SamplePoint> &samples)
{
    std::ifstream input(path);
    if (!input)
    {
        util::Log(logERROR) << "Could not open sample file: " << path.string();
        return false;
    }

    std::string line;
    std::uint64_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const auto trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#')
        {
            continue;
        }

        const auto fields = splitCommaLine(trimmed);
        if (fields.size() < 5)
        {
            util::Log(logERROR) << "Invalid sample row at " << path.string() << ":" << line_number
                                << ". Expected: h3_id,res,sample_index,lon,lat";
            return false;
        }
        if (line_number == 1 && fields[0] == "h3_id")
        {
            continue;
        }

        std::uint64_t resolution = 0;
        std::uint64_t sample_index = 0;
        double lon = 0.;
        double lat = 0.;
        std::istringstream res_parser(fields[1]);
        if (!(res_parser >> resolution) || !res_parser.eof())
        {
            util::Log(logERROR) << "Invalid resolution at " << path.string() << ":" << line_number;
            return false;
        }
        std::istringstream idx_parser(fields[2]);
        if (!(idx_parser >> sample_index) || !idx_parser.eof())
        {
            util::Log(logERROR) << "Invalid sample_index at " << path.string() << ":" << line_number;
            return false;
        }
        std::istringstream lon_parser(fields[3]);
        if (!(lon_parser >> lon) || !lon_parser.eof())
        {
            util::Log(logERROR) << "Invalid lon at " << path.string() << ":" << line_number;
            return false;
        }
        std::istringstream lat_parser(fields[4]);
        if (!(lat_parser >> lat) || !lat_parser.eof())
        {
            util::Log(logERROR) << "Invalid lat at " << path.string() << ":" << line_number;
            return false;
        }
        if (resolution != expected_resolution)
        {
            util::Log(logERROR) << "Unexpected resolution at " << path.string() << ":" << line_number
                                << ". Found " << resolution << ", expected "
                                << expected_resolution << ".";
            return false;
        }
        if (sample_index > std::numeric_limits<std::uint32_t>::max())
        {
            util::Log(logERROR) << "sample_index out of range at " << path.string() << ":"
                                << line_number;
            return false;
        }
        try
        {
            samples.push_back(
                SamplePoint{fields[0],
                            static_cast<std::uint32_t>(resolution),
                            static_cast<std::uint32_t>(sample_index),
                            util::Coordinate{
                                util::UnsafeFloatLongitude{lon},
                                util::UnsafeFloatLatitude{lat},
                            }});
        }
        catch (const std::exception &)
        {
            util::Log(logERROR) << "Invalid coordinate at " << path.string() << ":" << line_number;
            return false;
        }
    }

    if (samples.empty())
    {
        util::Log(logERROR) << "No sample points loaded from: " << path.string();
        return false;
    }

    return true;
}

bool loadPOICoordinates(const std::filesystem::path &path, std::vector<util::Coordinate> &pois)
{
    std::ifstream input(path);
    if (!input)
    {
        util::Log(logERROR) << "Could not open oracle POI file: " << path.string();
        return false;
    }

    pois.clear();
    std::string line;
    std::uint64_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        std::string token = line;
        const auto comment_pos = token.find('#');
        if (comment_pos != std::string::npos)
        {
            token.erase(comment_pos);
        }
        token = trim(token);
        if (token.empty())
        {
            continue;
        }

        std::replace(token.begin(), token.end(), ',', ' ');
        std::istringstream parser(token);
        double lon = 0.;
        double lat = 0.;
        if (!(parser >> lon >> lat))
        {
            util::Log(logERROR) << "Invalid POI coordinate in " << path.string() << ":" << line_number;
            return false;
        }
        parser >> std::ws;
        if (!parser.eof())
        {
            util::Log(logERROR) << "Unexpected trailing token in " << path.string() << ":"
                                << line_number;
            return false;
        }
        if (!std::isfinite(lon) || !std::isfinite(lat))
        {
            util::Log(logERROR) << "Non-finite POI coordinate in " << path.string() << ":"
                                << line_number;
            return false;
        }

        try
        {
            pois.push_back(util::Coordinate{
                util::UnsafeFloatLongitude{lon},
                util::UnsafeFloatLatitude{lat},
            });
        }
        catch (const std::exception &)
        {
            util::Log(logERROR) << "POI coordinate out of range in " << path.string() << ":"
                                << line_number;
            return false;
        }
    }

    if (pois.empty())
    {
        util::Log(logERROR) << "No POI coordinates loaded from: " << path.string();
        return false;
    }

    return true;
}

std::vector<std::size_t> selectOracleSampleIndices(const std::size_t total_samples,
                                                   const std::uint32_t oracle_max_samples)
{
    std::vector<std::size_t> selected_indices;
    if (total_samples == 0)
    {
        return selected_indices;
    }

    const auto limit = oracle_max_samples == 0
                           ? total_samples
                           : std::min<std::size_t>(total_samples, oracle_max_samples);
    selected_indices.reserve(limit);
    for (const auto slot : util::irange<std::size_t>(0, limit))
    {
        const auto sample_index = (slot * total_samples) / limit;
        selected_indices.push_back(sample_index);
    }
    return selected_indices;
}

bool extractDurationTicks(const json::Value &value, bool &is_reachable, std::int64_t &ticks)
{
    if (const auto *number = std::get_if<json::Number>(&value))
    {
        if (!std::isfinite(number->value) || number->value < 0.0)
        {
            util::Log(logERROR) << "Invalid duration value returned by OSRM table oracle.";
            return false;
        }
        is_reachable = true;
        ticks = static_cast<std::int64_t>(std::llround(number->value * 10.0));
        return true;
    }
    if (std::holds_alternative<json::Null>(value))
    {
        is_reachable = false;
        ticks = 0;
        return true;
    }

    util::Log(logERROR) << "Unexpected duration entry type returned by OSRM table oracle.";
    return false;
}

std::string edgeWeightToString(const EdgeWeight value)
{
    if (value == INVALID_EDGE_WEIGHT)
    {
        return "invalid";
    }
    return std::to_string(from_alias<std::int64_t>(value));
}

std::string formatOraclePOITicks(const std::vector<std::optional<std::int64_t>> &ticks_by_poi,
                                 const std::vector<util::Coordinate> &pois)
{
    std::ostringstream stream;
    stream << "[";
    for (const auto poi_index : util::irange<std::size_t>(0, ticks_by_poi.size()))
    {
        if (poi_index > 0)
        {
            stream << ", ";
        }
        stream << "poi" << poi_index << "("
               << static_cast<double>(util::toFloating(pois[poi_index].lon)) << ","
               << static_cast<double>(util::toFloating(pois[poi_index].lat)) << ")=";
        if (!ticks_by_poi[poi_index].has_value())
        {
            stream << "null";
            continue;
        }
        stream << ticks_by_poi[poi_index].value();
    }
    stream << "]";
    return stream.str();
}

void logOracleMismatchTrace(const SamplePoint &sample,
                            const SampleMappingTrace &trace,
                            const std::vector<std::optional<std::int64_t>> &oracle_ticks_by_poi,
                            const std::vector<util::Coordinate> &pois,
                            const std::string &mismatch_reason)
{
    util::Log(logERROR) << "Oracle mismatch trace: sample_index=" << sample.sample_index
                        << ", h3_id=" << sample.h3_id
                        << ", lon=" << static_cast<double>(util::toFloating(sample.coordinate.lon))
                        << ", lat=" << static_cast<double>(util::toFloating(sample.coordinate.lat))
                        << ", reason=" << mismatch_reason;
    util::Log(logERROR) << "  snap: candidate_count=" << trace.snap_candidate_count
                        << ", chosen_candidate_index="
                        << (trace.chosen_candidate_index == std::numeric_limits<std::size_t>::max()
                                ? std::string("none")
                                : std::to_string(trace.chosen_candidate_index))
                        << ", chosen_forward_segment="
                        << (trace.chosen_forward_segment_enabled
                                ? std::to_string(trace.chosen_forward_segment_id)
                                : std::string("disabled"))
                        << ", chosen_reverse_segment="
                        << (trace.chosen_reverse_segment_enabled
                                ? std::to_string(trace.chosen_reverse_segment_id)
                                : std::string("disabled"));
    util::Log(logERROR) << "  raster: chosen_state_node="
                        << (trace.has_chosen_state ? std::to_string(trace.chosen_state_node_id)
                                                   : std::string("none"))
                        << ", state_direction="
                        << (trace.has_chosen_state
                                ? (trace.chosen_state_forward ? "forward" : "reverse")
                                : "none")
                        << ", source_offset=" << edgeWeightToString(trace.chosen_source_offset)
                        << ", phast_state_cost=" << edgeWeightToString(trace.chosen_state_cost)
                        << ", raster_total=" << edgeWeightToString(trace.chosen_total_cost);
    util::Log(logERROR) << "  oracle: poi_ticks="
                        << formatOraclePOITicks(oracle_ticks_by_poi, pois);
}

void logOraclePassTrace(const SamplePoint &sample,
                        const SampleMappingTrace &trace,
                        const std::vector<std::optional<std::int64_t>> &oracle_ticks_by_poi,
                        const std::vector<util::Coordinate> &pois,
                        const std::int64_t delta_ticks)
{
    util::Log() << "Oracle pass trace: sample_index=" << sample.sample_index
                << ", h3_id=" << sample.h3_id
                << ", lon=" << static_cast<double>(util::toFloating(sample.coordinate.lon))
                << ", lat=" << static_cast<double>(util::toFloating(sample.coordinate.lat))
                << ", delta_ticks(raster-table)=" << delta_ticks;
    util::Log() << "  snap: candidate_count=" << trace.snap_candidate_count
                << ", chosen_candidate_index="
                << (trace.chosen_candidate_index == std::numeric_limits<std::size_t>::max()
                        ? std::string("none")
                        : std::to_string(trace.chosen_candidate_index))
                << ", chosen_forward_segment="
                << (trace.chosen_forward_segment_enabled ? std::to_string(trace.chosen_forward_segment_id)
                                                         : std::string("disabled"))
                << ", chosen_reverse_segment="
                << (trace.chosen_reverse_segment_enabled ? std::to_string(trace.chosen_reverse_segment_id)
                                                         : std::string("disabled"));
    util::Log() << "  raster: chosen_state_node="
                << (trace.has_chosen_state ? std::to_string(trace.chosen_state_node_id)
                                           : std::string("none"))
                << ", state_direction="
                << (trace.has_chosen_state ? (trace.chosen_state_forward ? "forward" : "reverse")
                                           : "none")
                << ", source_offset=" << edgeWeightToString(trace.chosen_source_offset)
                << ", phast_state_cost=" << edgeWeightToString(trace.chosen_state_cost)
                << ", raster_total=" << edgeWeightToString(trace.chosen_total_cost);
    util::Log() << "  oracle: poi_ticks=" << formatOraclePOITicks(oracle_ticks_by_poi, pois);
}

bool runDurationOracle(const std::filesystem::path &base_path,
                       const PHASTOrientation orientation,
                       const std::vector<SamplePoint> &samples,
                       const std::vector<EdgeWeight> &sample_costs,
                       const std::vector<SampleMappingTrace> &sample_traces,
                       const std::vector<util::Coordinate> &pois,
                       const std::uint32_t oracle_max_samples,
                       const std::int64_t tolerance_ticks,
                       const std::filesystem::path &mismatch_report_path,
                       const bool report_all,
                       const std::size_t trace_limit,
                       OracleStats &stats,
                       std::vector<OracleComparison> &comparisons)
{
    stats = {};
    comparisons.clear();
    if (samples.size() != sample_costs.size())
    {
        util::Log(logERROR) << "Oracle input mismatch: samples and sample costs differ in size.";
        return false;
    }
    if (samples.size() != sample_traces.size())
    {
        util::Log(logERROR) << "Oracle input mismatch: samples and traces differ in size.";
        return false;
    }
    if (samples.empty())
    {
        util::Log(logERROR) << "Oracle input is empty.";
        return false;
    }
    if (pois.empty())
    {
        util::Log(logERROR) << "Oracle requires at least one POI coordinate.";
        return false;
    }

    const auto selected_sample_indices = selectOracleSampleIndices(samples.size(), oracle_max_samples);
    if (selected_sample_indices.empty())
    {
        util::Log(logERROR) << "Oracle selected zero samples.";
        return false;
    }
    comparisons.reserve(selected_sample_indices.size());

    engine::EngineConfig engine_config;
    engine_config.storage_config = storage::StorageConfig(base_path);
    engine_config.use_shared_memory = false;
    engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
    if (!engine_config.IsValid())
    {
        util::Log(logERROR) << "Could not initialize OSRM table oracle: invalid engine config.";
        return false;
    }

    OSRM osrm{engine_config};
    TableParameters params;
    params.annotations = TableParameters::AnnotationsType::Duration;
    params.skip_waypoints = true;

    if (orientation == PHASTOrientation::Reverse)
    {
        for (const auto sample_index : selected_sample_indices)
        {
            params.coordinates.push_back(samples[sample_index].coordinate);
            params.sources.push_back(params.sources.size());
        }
        const auto poi_offset = params.coordinates.size();
        for (const auto poi_index : util::irange<std::size_t>(0, pois.size()))
        {
            params.coordinates.push_back(pois[poi_index]);
            params.destinations.push_back(poi_offset + poi_index);
        }
    }
    else
    {
        for (const auto poi_index : util::irange<std::size_t>(0, pois.size()))
        {
            params.coordinates.push_back(pois[poi_index]);
            params.sources.push_back(params.sources.size());
        }
        const auto sample_offset = params.coordinates.size();
        for (const auto local_sample_index : util::irange<std::size_t>(0, selected_sample_indices.size()))
        {
            params.coordinates.push_back(samples[selected_sample_indices[local_sample_index]].coordinate);
            params.destinations.push_back(sample_offset + local_sample_index);
        }
    }
    if (!params.IsValid())
    {
        util::Log(logERROR) << "OSRM table oracle parameters are invalid.";
        return false;
    }

    json::Object table_json;
    const auto status = osrm.Table(params, table_json);
    if (status != Status::Ok)
    {
        util::Log(logERROR) << "OSRM table oracle query failed.";
        return false;
    }
    if (!table_json.values.contains("code") || !table_json.values.contains("durations"))
    {
        util::Log(logERROR) << "OSRM table oracle response is missing required fields.";
        return false;
    }
    const auto code = std::get<json::String>(table_json.values.at("code")).value;
    if (code != "Ok")
    {
        util::Log(logERROR) << "OSRM table oracle response code: " << code;
        return false;
    }

    const auto &durations_rows = std::get<json::Array>(table_json.values.at("durations")).values;
    const auto expected_row_count = orientation == PHASTOrientation::Reverse
                                        ? selected_sample_indices.size()
                                        : pois.size();
    if (durations_rows.size() != expected_row_count)
    {
        util::Log(logERROR) << "Unexpected OSRM table oracle row count. got=" << durations_rows.size()
                            << ", expected=" << expected_row_count;
        return false;
    }

    std::ofstream mismatch_report;
    if (!mismatch_report_path.empty())
    {
        mismatch_report.open(mismatch_report_path);
        if (!mismatch_report)
        {
            util::Log(logERROR) << "Could not open oracle mismatch report path: "
                                << mismatch_report_path.string();
            return false;
        }
        mismatch_report
            << "sample_index,h3_id,lon,lat,raster_ticks,oracle_ticks,oracle_seconds,abs_error_ticks,reason,"
            << "chosen_candidate_index,chosen_state_node,state_direction,source_offset,phast_state_cost,"
            << "raster_total,delta_ticks\n";
    }

    std::size_t pass_trace_count = 0;
    std::size_t mismatch_trace_count = 0;
    for (const auto local_sample_index : util::irange<std::size_t>(0, selected_sample_indices.size()))
    {
        const auto sample_index = selected_sample_indices[local_sample_index];
        const auto &sample = samples[sample_index];

        bool oracle_reachable = false;
        std::int64_t oracle_best_ticks = 0;
        std::vector<std::optional<std::int64_t>> oracle_ticks_by_poi(pois.size(), std::nullopt);

        if (orientation == PHASTOrientation::Reverse)
        {
            const auto &row = std::get<json::Array>(durations_rows[local_sample_index]).values;
            if (row.size() != pois.size())
            {
                util::Log(logERROR) << "Unexpected OSRM table oracle column count for sample row "
                                    << local_sample_index << ".";
                return false;
            }

            for (const auto poi_index : util::irange<std::size_t>(0, row.size()))
            {
                const auto &entry = row[poi_index];
                bool reachable = false;
                std::int64_t ticks = 0;
                if (!extractDurationTicks(entry, reachable, ticks))
                {
                    return false;
                }
                if (!reachable)
                {
                    continue;
                }
                oracle_ticks_by_poi[poi_index] = ticks;
                if (!oracle_reachable || ticks < oracle_best_ticks)
                {
                    oracle_reachable = true;
                    oracle_best_ticks = ticks;
                }
            }
        }
        else
        {
            for (const auto row_index : util::irange<std::size_t>(0, durations_rows.size()))
            {
                const auto &row = std::get<json::Array>(durations_rows[row_index]).values;
                if (row.size() != selected_sample_indices.size())
                {
                    util::Log(logERROR)
                        << "Unexpected OSRM table oracle column count for forward orientation row "
                        << row_index << ".";
                    return false;
                }
                bool reachable = false;
                std::int64_t ticks = 0;
                if (!extractDurationTicks(row[local_sample_index], reachable, ticks))
                {
                    return false;
                }
                if (!reachable)
                {
                    continue;
                }
                oracle_ticks_by_poi[row_index] = ticks;
                if (!oracle_reachable || ticks < oracle_best_ticks)
                {
                    oracle_reachable = true;
                    oracle_best_ticks = ticks;
                }
            }
        }

        ++stats.compared_samples;
        const auto raster_cost = sample_costs[sample_index];
        const auto raster_reachable = raster_cost != INVALID_EDGE_WEIGHT;
        const auto raster_ticks = raster_reachable ? from_alias<std::int64_t>(raster_cost) : 0;

        bool mismatch = false;
        std::int64_t delta_ticks = 0;
        std::int64_t abs_error_ticks = 0;
        std::string mismatch_reason;
        if (!raster_reachable && !oracle_reachable)
        {
            ++stats.unreachable_pairs;
        }
        else if (!raster_reachable && oracle_reachable)
        {
            mismatch = true;
            mismatch_reason = "raster_unreachable_oracle_reachable";
        }
        else if (raster_reachable && !oracle_reachable)
        {
            mismatch = true;
            mismatch_reason = "raster_reachable_oracle_unreachable";
        }
        else
        {
            ++stats.reachable_pairs;
            delta_ticks = raster_ticks - oracle_best_ticks;
            abs_error_ticks = std::llabs(delta_ticks);
            if (abs_error_ticks > tolerance_ticks)
            {
                mismatch = true;
                mismatch_reason = "abs_error_exceeds_tolerance";
            }
            stats.max_abs_error_ticks = std::max(stats.max_abs_error_ticks, abs_error_ticks);
        }

        comparisons.push_back(OracleComparison{
            local_sample_index,
            sample_index,
            raster_reachable,
            oracle_reachable,
            raster_ticks,
            oracle_reachable ? oracle_best_ticks : 0,
            delta_ticks,
            abs_error_ticks,
            mismatch,
            mismatch ? mismatch_reason : std::string("ok"),
            oracle_ticks_by_poi});

        if (mismatch)
        {
            ++stats.mismatches;
            if (mismatch_trace_count < trace_limit)
            {
                logOracleMismatchTrace(sample,
                                       sample_traces[sample_index],
                                       oracle_ticks_by_poi,
                                       pois,
                                       mismatch_reason);
                ++mismatch_trace_count;
            }
        }
        else if (pass_trace_count < std::min<std::size_t>(trace_limit, 2))
        {
            logOraclePassTrace(sample,
                               sample_traces[sample_index],
                               oracle_ticks_by_poi,
                               pois,
                               delta_ticks);
            ++pass_trace_count;
        }

        if (!mismatch_report_path.empty() && (report_all || mismatch))
        {
            const auto &trace = sample_traces[sample_index];
            mismatch_report << sample.sample_index << "," << sample.h3_id << ","
                            << static_cast<double>(util::toFloating(sample.coordinate.lon)) << ","
                            << static_cast<double>(util::toFloating(sample.coordinate.lat)) << ",";
            if (raster_reachable)
            {
                mismatch_report << from_alias<std::int64_t>(raster_cost);
            }
            mismatch_report << ",";
            if (oracle_reachable)
            {
                mismatch_report << oracle_best_ticks << "," << std::fixed << std::setprecision(1)
                                << (oracle_best_ticks / 10.0);
            }
            mismatch_report << "," << abs_error_ticks << "," << mismatch_reason << ",";
            if (trace.chosen_candidate_index == std::numeric_limits<std::size_t>::max())
            {
                mismatch_report << ",";
            }
            else
            {
                mismatch_report << trace.chosen_candidate_index << ",";
            }
            if (trace.has_chosen_state)
            {
                mismatch_report << trace.chosen_state_node_id << ","
                                << (trace.chosen_state_forward ? "forward" : "reverse") << ","
                                << edgeWeightToString(trace.chosen_source_offset) << ","
                                << edgeWeightToString(trace.chosen_state_cost) << ","
                                << edgeWeightToString(trace.chosen_total_cost);
            }
            else
            {
                mismatch_report << ",,,,"; // chosen_state_node,state_direction,source_offset,phast_state_cost,raster_total
            }
            mismatch_report << "," << delta_ticks << "\n";
        }
    }

    return true;
}

bool convertDurationOffset(const engine::PhantomNode &phantom,
                           const bool forward_state,
                           EdgeWeight &offset)
{
    const auto duration = forward_state ? phantom.GetForwardDuration() : phantom.GetReverseDuration();
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
    offset = to_alias<EdgeWeight>(duration_value);
    return true;
}

bool getOrientationSeedValidity(const engine::PhantomNode &phantom,
                                const PHASTOrientation orientation,
                                bool &forward_valid,
                                bool &reverse_valid)
{
    if (orientation == PHASTOrientation::Reverse)
    {
        forward_valid = phantom.IsValidForwardTarget();
        reverse_valid = phantom.IsValidReverseTarget();
        return true;
    }
    if (orientation == PHASTOrientation::Forward)
    {
        forward_valid = phantom.IsValidForwardSource();
        reverse_valid = phantom.IsValidReverseSource();
        return true;
    }
    return false;
}

engine::PhantomNode restrictPhantomToSingleState(const engine::PhantomNode &phantom,
                                                 const bool forward_state)
{
    auto restricted = phantom;
    if (forward_state)
    {
        restricted.reverse_segment_id = SegmentID{SPECIAL_SEGMENTID, false};
    }
    else
    {
        restricted.forward_segment_id = SegmentID{SPECIAL_SEGMENTID, false};
    }
    return restricted;
}

engine::Hint makeHintFromPhantom(const engine::PhantomNode &phantom, const std::uint32_t checksum)
{
    engine::SegmentHint segment_hint;
    segment_hint.phantom = phantom;
    segment_hint.data_checksum = checksum;
    engine::Hint hint;
    hint.segment_hints.push_back(segment_hint);
    return hint;
}

engine::PhantomNode makeZeroStateOffsetPhantom(const engine::PhantomNode &phantom, const bool forward_state)
{
    auto adjusted = phantom;
    if (forward_state)
    {
        adjusted.forward_weight = EdgeWeight{0};
        adjusted.forward_weight_offset = EdgeWeight{0};
        adjusted.forward_duration = EdgeDuration{0};
        adjusted.forward_duration_offset = EdgeDuration{0};
        adjusted.forward_distance = EdgeDistance{0};
        adjusted.forward_distance_offset = EdgeDistance{0};
    }
    else
    {
        adjusted.reverse_weight = EdgeWeight{0};
        adjusted.reverse_weight_offset = EdgeWeight{0};
        adjusted.reverse_duration = EdgeDuration{0};
        adjusted.reverse_duration_offset = EdgeDuration{0};
        adjusted.reverse_distance = EdgeDistance{0};
        adjusted.reverse_distance_offset = EdgeDistance{0};
    }
    return adjusted;
}

std::string formatStrictSeedTicks(const std::vector<std::optional<std::int64_t>> &ticks_by_seed,
                                  const std::vector<StrictSeedEndpoint> &seed_endpoints)
{
    std::ostringstream stream;
    stream << "[";
    for (const auto seed_index : util::irange<std::size_t>(0, seed_endpoints.size()))
    {
        if (seed_index > 0)
        {
            stream << ", ";
        }
        const auto &seed = seed_endpoints[seed_index];
        stream << "seed" << seed_index << "(poi=" << seed.poi_index
               << ",dir=" << (seed.forward_state ? "forward" : "reverse")
               << ",node=" << seed.state_node_id
               << ",seed_offset=" << edgeWeightToString(seed.seed_offset) << ")=";
        if (!ticks_by_seed[seed_index].has_value())
        {
            stream << "null";
        }
        else
        {
            stream << ticks_by_seed[seed_index].value();
        }
    }
    stream << "]";
    return stream.str();
}

bool buildStrictSeedEndpoints(const CHDataFacade &facade,
                              const PHASTOrientation orientation,
                              const std::vector<util::Coordinate> &pois,
                              std::vector<StrictSeedEndpoint> &seed_endpoints)
{
    seed_endpoints.clear();
    seed_endpoints.reserve(pois.size() * 2);

    for (const auto poi_index : util::irange<std::size_t>(0, pois.size()))
    {
        const auto &poi = pois[poi_index];
        auto alternatives = facade.NearestCandidatesWithAlternativeFromBigComponent(
            poi, std::nullopt, std::nullopt, engine::Approach::UNRESTRICTED, true);

        engine::PhantomNode snapped;
        if (!alternatives.first.empty())
        {
            snapped = alternatives.first.front();
        }
        else if (!alternatives.second.empty())
        {
            snapped = alternatives.second.front();
        }
        else
        {
            util::Log(logERROR) << "Strict endpoint oracle could not snap POI index " << poi_index;
            return false;
        }

        bool forward_valid = false;
        bool reverse_valid = false;
        if (!getOrientationSeedValidity(snapped, orientation, forward_valid, reverse_valid))
        {
            util::Log(logERROR) << "Unsupported orientation while building strict seed endpoints.";
            return false;
        }
        if (!forward_valid && !reverse_valid)
        {
            util::Log(logERROR) << "Strict endpoint oracle snapped POI index " << poi_index
                                << " without orientation-valid seed states.";
            return false;
        }

        auto push_seed = [&](const bool forward_state, const SegmentID segment_id) -> bool
        {
            if (!segment_id.enabled)
            {
                return false;
            }
            EdgeWeight seed_offset = INVALID_EDGE_WEIGHT;
            if (!convertDurationOffset(snapped, forward_state, seed_offset))
            {
                util::Log(logERROR) << "Could not convert strict seed offset for POI index "
                                    << poi_index;
                return false;
            }
            seed_endpoints.push_back(StrictSeedEndpoint{
                poi_index,
                forward_state,
                segment_id.id,
                seed_offset,
                poi,
                restrictPhantomToSingleState(snapped, forward_state)});
            return true;
        };

        if (forward_valid && !push_seed(true, snapped.forward_segment_id))
        {
            return false;
        }
        if (reverse_valid && !push_seed(false, snapped.reverse_segment_id))
        {
            return false;
        }
    }

    return !seed_endpoints.empty();
}

bool runStrictEndpointOracle(const std::filesystem::path &base_path,
                             const CHDataFacade &facade,
                             const PHASTOrientation orientation,
                             const std::vector<SamplePoint> &samples,
                             const std::vector<EdgeWeight> &sample_costs,
                             const std::vector<SampleMappingTrace> &sample_traces,
                             const std::vector<util::Coordinate> &pois,
                             const std::vector<OracleComparison> &coordinate_comparisons,
                             const std::int64_t tolerance_ticks,
                             const std::size_t trace_limit,
                             StrictOracleStats &stats,
                             std::vector<std::int64_t> &strict_deltas,
                             std::vector<std::int64_t> &state_deltas)
{
    stats = {};
    strict_deltas.clear();
    state_deltas.clear();
    if (samples.size() != sample_costs.size() || samples.size() != sample_traces.size())
    {
        util::Log(logERROR) << "Strict endpoint oracle input sizes are inconsistent.";
        return false;
    }
    if (coordinate_comparisons.empty())
    {
        util::Log(logERROR) << "Strict endpoint oracle has no coordinate oracle samples to compare.";
        return false;
    }

    std::vector<StrictSeedEndpoint> seed_endpoints;
    if (!buildStrictSeedEndpoints(facade, orientation, pois, seed_endpoints))
    {
        return false;
    }
    util::Log() << "Strict endpoint oracle seed states: " << seed_endpoints.size();

    std::unordered_set<std::size_t> coordinate_mismatch_samples;
    for (const auto &comparison : coordinate_comparisons)
    {
        if (comparison.mismatch)
        {
            coordinate_mismatch_samples.insert(comparison.sample_index);
        }
    }

    engine::EngineConfig engine_config;
    engine_config.storage_config = storage::StorageConfig(base_path);
    engine_config.use_shared_memory = false;
    engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
    if (!engine_config.IsValid())
    {
        util::Log(logERROR) << "Could not initialize strict endpoint oracle: invalid engine config.";
        return false;
    }
    OSRM osrm{engine_config};

    std::size_t logged_mismatch_traces = 0;
    std::size_t logged_pass_traces = 0;
    for (const auto &comparison : coordinate_comparisons)
    {
        const auto sample_index = comparison.sample_index;
        const auto &sample = samples[sample_index];
        const auto &trace = sample_traces[sample_index];
        if (!trace.has_chosen_state ||
            trace.chosen_candidate_index == std::numeric_limits<std::size_t>::max())
        {
            ++stats.compared_samples;
            ++stats.unreachable_pairs;
            continue;
        }

        auto source_candidates = facade.NearestPhantomNodes(sample.coordinate,
                                                            MAX_SNAP_CANDIDATES,
                                                            std::nullopt,
                                                            std::nullopt,
                                                            engine::Approach::UNRESTRICTED);
        const engine::PhantomNode *matched_source_phantom = nullptr;
        for (const auto &candidate : source_candidates)
        {
            const auto segment_id = trace.chosen_state_forward ? candidate.phantom_node.forward_segment_id
                                                               : candidate.phantom_node.reverse_segment_id;
            if (segment_id.enabled && segment_id.id == trace.chosen_state_node_id)
            {
                matched_source_phantom = &candidate.phantom_node;
                break;
            }
        }
        if (matched_source_phantom == nullptr)
        {
            util::Log(logERROR) << "Strict endpoint oracle could not recover chosen source state for sample "
                                << sample_index << " (state_node=" << trace.chosen_state_node_id << ").";
            return false;
        }
        const auto source_phantom =
            restrictPhantomToSingleState(*matched_source_phantom, trace.chosen_state_forward);

        auto run_table_for_source = [&](const engine::PhantomNode &source_hint_phantom,
                                        const bool zero_target_offsets,
                                        std::vector<std::optional<std::int64_t>> &ticks_by_seed,
                                        bool &reachable,
                                        std::int64_t &best_ticks) -> bool
        {
            TableParameters params;
            params.annotations = TableParameters::AnnotationsType::Duration;
            params.skip_waypoints = true;
            params.generate_hints = false;
            params.coordinates.reserve(seed_endpoints.size() + 1);
            params.hints.reserve(seed_endpoints.size() + 1);
            params.destinations.reserve(seed_endpoints.size());

            params.coordinates.push_back(sample.coordinate);
            params.hints.push_back(makeHintFromPhantom(source_hint_phantom, facade.GetCheckSum()));
            params.sources.push_back(0);
            for (const auto seed_index : util::irange<std::size_t>(0, seed_endpoints.size()))
            {
                const auto target_phantom = zero_target_offsets
                                                ? makeZeroStateOffsetPhantom(
                                                      seed_endpoints[seed_index].restricted_phantom,
                                                      seed_endpoints[seed_index].forward_state)
                                                : seed_endpoints[seed_index].restricted_phantom;
                params.coordinates.push_back(seed_endpoints[seed_index].coordinate);
                params.hints.push_back(makeHintFromPhantom(target_phantom, facade.GetCheckSum()));
                params.destinations.push_back(seed_index + 1);
            }
            if (!params.IsValid())
            {
                util::Log(logERROR) << "Strict endpoint oracle table parameters are invalid.";
                return false;
            }

            json::Object table_json;
            const auto status = osrm.Table(params, table_json);
            if (status != Status::Ok || !table_json.values.contains("durations"))
            {
                util::Log(logERROR) << "Strict endpoint oracle table query failed.";
                return false;
            }

            const auto &durations_rows = std::get<json::Array>(table_json.values.at("durations")).values;
            if (durations_rows.size() != 1)
            {
                util::Log(logERROR) << "Strict endpoint oracle expected one source row.";
                return false;
            }
            const auto &row = std::get<json::Array>(durations_rows.front()).values;
            if (row.size() != seed_endpoints.size())
            {
                util::Log(logERROR) << "Strict endpoint oracle column count mismatch.";
                return false;
            }

            ticks_by_seed.assign(seed_endpoints.size(), std::nullopt);
            reachable = false;
            best_ticks = 0;
            for (const auto seed_index : util::irange<std::size_t>(0, row.size()))
            {
                bool seed_reachable = false;
                std::int64_t ticks = 0;
                if (!extractDurationTicks(row[seed_index], seed_reachable, ticks))
                {
                    return false;
                }
                if (!seed_reachable)
                {
                    continue;
                }
                ticks_by_seed[seed_index] = ticks;
                if (!reachable || ticks < best_ticks)
                {
                    reachable = true;
                    best_ticks = ticks;
                }
            }
            return true;
        };

        std::vector<std::optional<std::int64_t>> strict_ticks_by_seed;
        bool strict_reachable = false;
        std::int64_t strict_best_ticks = 0;
        if (!run_table_for_source(
                source_phantom, false, strict_ticks_by_seed, strict_reachable, strict_best_ticks))
        {
            return false;
        }

        std::vector<std::optional<std::int64_t>> state_base_ticks_by_seed;
        bool state_base_reachable = false;
        std::int64_t state_base_best_ticks = 0;
        const auto state_source_phantom =
            makeZeroStateOffsetPhantom(source_phantom, trace.chosen_state_forward);
        if (!run_table_for_source(state_source_phantom,
                                  true,
                                  state_base_ticks_by_seed,
                                  state_base_reachable,
                                  state_base_best_ticks))
        {
            return false;
        }

        std::vector<std::optional<std::int64_t>> state_ticks_by_seed(seed_endpoints.size(), std::nullopt);
        bool state_oracle_reachable = false;
        std::int64_t state_oracle_best_ticks = 0;
        for (const auto seed_index : util::irange<std::size_t>(0, seed_endpoints.size()))
        {
            if (!state_base_ticks_by_seed[seed_index].has_value())
            {
                continue;
            }
            const auto seed_offset = from_alias<std::int64_t>(seed_endpoints[seed_index].seed_offset);
            const auto base_ticks = state_base_ticks_by_seed[seed_index].value();
            const auto combined_ticks = base_ticks + seed_offset;
            state_ticks_by_seed[seed_index] = combined_ticks;
            if (!state_oracle_reachable || combined_ticks < state_oracle_best_ticks)
            {
                state_oracle_reachable = true;
                state_oracle_best_ticks = combined_ticks;
            }
        }

        ++stats.compared_samples;
        const auto raster_reachable = sample_costs[sample_index] != INVALID_EDGE_WEIGHT;
        bool strict_mismatch = false;
        std::int64_t strict_delta_ticks = 0;
        std::int64_t strict_abs_ticks = 0;
        std::string reason = "ok";
        if (!raster_reachable && !strict_reachable)
        {
            ++stats.unreachable_pairs;
        }
        else if (!raster_reachable && strict_reachable)
        {
            strict_mismatch = true;
            reason = "raster_unreachable_strict_reachable";
        }
        else if (raster_reachable && !strict_reachable)
        {
            strict_mismatch = true;
            reason = "raster_reachable_strict_unreachable";
        }
        else
        {
            ++stats.reachable_pairs;
            strict_delta_ticks = from_alias<std::int64_t>(sample_costs[sample_index]) - strict_best_ticks;
            strict_abs_ticks = std::llabs(strict_delta_ticks);
            strict_deltas.push_back(strict_delta_ticks);
            if (strict_abs_ticks > tolerance_ticks)
            {
                strict_mismatch = true;
                reason = "abs_error_exceeds_tolerance";
            }
            stats.max_abs_error_ticks = std::max(stats.max_abs_error_ticks, strict_abs_ticks);
        }

        ++stats.state_compared_samples;
        bool state_reachable = state_oracle_reachable;
        bool state_mismatch = false;
        std::int64_t state_delta_ticks = 0;
        std::int64_t state_abs_ticks = 0;
        const auto phast_state_reachable =
            trace.chosen_state_cost != INVALID_EDGE_WEIGHT;

        if (!phast_state_reachable && !state_reachable)
        {
            ++stats.state_unreachable_pairs;
        }
        else if (!phast_state_reachable && state_reachable)
        {
            state_mismatch = true;
        }
        else if (phast_state_reachable && !state_reachable)
        {
            state_mismatch = true;
        }
        else
        {
            ++stats.state_reachable_pairs;
            const auto phast_state_ticks = from_alias<std::int64_t>(trace.chosen_state_cost);
            state_delta_ticks = phast_state_ticks - state_oracle_best_ticks;
            state_abs_ticks = std::llabs(state_delta_ticks);
            state_deltas.push_back(state_delta_ticks);
            if (state_abs_ticks > tolerance_ticks)
            {
                state_mismatch = true;
            }
            stats.state_max_abs_error_ticks = std::max(stats.state_max_abs_error_ticks, state_abs_ticks);
        }
        if (state_mismatch)
        {
            ++stats.state_mismatches;
        }
        if (strict_mismatch && !state_mismatch)
        {
            ++stats.endpoint_semantics_mismatch_samples;
        }

        if (coordinate_mismatch_samples.contains(sample_index) && !strict_mismatch)
        {
            ++stats.coordinate_mismatches_resolved;
        }
        if (strict_mismatch)
        {
            ++stats.mismatches;
            if (logged_mismatch_traces < trace_limit)
            {
                util::Log(logERROR)
                    << "Strict endpoint mismatch: sample_index=" << sample.sample_index
                    << ", h3_id=" << sample.h3_id << ", reason=" << reason
                    << ", strict_delta_ticks=" << strict_delta_ticks
                    << ", state_delta_ticks=" << state_delta_ticks;
                util::Log(logERROR) << "  strict_seed_ticks="
                                    << formatStrictSeedTicks(strict_ticks_by_seed, seed_endpoints);
                util::Log(logERROR) << "  state_seed_ticks="
                                    << formatStrictSeedTicks(state_ticks_by_seed, seed_endpoints);
                ++logged_mismatch_traces;
            }
        }
        else if (logged_pass_traces < std::min<std::size_t>(trace_limit, 2))
        {
            util::Log() << "Strict endpoint pass: sample_index=" << sample.sample_index
                        << ", h3_id=" << sample.h3_id
                        << ", strict_delta_ticks=" << strict_delta_ticks
                        << ", state_delta_ticks=" << state_delta_ticks;
            util::Log() << "  strict_seed_ticks="
                        << formatStrictSeedTicks(strict_ticks_by_seed, seed_endpoints);
            util::Log() << "  state_seed_ticks="
                        << formatStrictSeedTicks(state_ticks_by_seed, seed_endpoints);
            ++logged_pass_traces;
        }
    }

    return true;
}

void logDeltaBuckets(const std::string &label, const std::vector<std::int64_t> &deltas)
{
    if (deltas.empty())
    {
        util::Log() << label << " delta buckets: no reachable pairs";
        return;
    }

    std::size_t negative = 0;
    std::size_t zero = 0;
    std::size_t positive = 0;
    std::size_t bucket_0_15 = 0;
    std::size_t bucket_16_30 = 0;
    std::size_t bucket_31_60 = 0;
    std::size_t bucket_61_120 = 0;
    std::size_t bucket_121_300 = 0;
    std::size_t bucket_301_600 = 0;
    std::size_t bucket_601_plus = 0;
    for (const auto delta : deltas)
    {
        if (delta < 0)
        {
            ++negative;
        }
        else if (delta > 0)
        {
            ++positive;
        }
        else
        {
            ++zero;
        }

        const auto abs_delta = std::llabs(delta);
        if (abs_delta <= 15)
        {
            ++bucket_0_15;
        }
        else if (abs_delta <= 30)
        {
            ++bucket_16_30;
        }
        else if (abs_delta <= 60)
        {
            ++bucket_31_60;
        }
        else if (abs_delta <= 120)
        {
            ++bucket_61_120;
        }
        else if (abs_delta <= 300)
        {
            ++bucket_121_300;
        }
        else if (abs_delta <= 600)
        {
            ++bucket_301_600;
        }
        else
        {
            ++bucket_601_plus;
        }
    }

    util::Log() << label << " delta sign: negative=" << negative << ", zero=" << zero
                << ", positive=" << positive;
    util::Log() << label << " abs(delta) buckets: [0..15]=" << bucket_0_15
                << ", [16..30]=" << bucket_16_30 << ", [31..60]=" << bucket_31_60
                << ", [61..120]=" << bucket_61_120 << ", [121..300]=" << bucket_121_300
                << ", [301..600]=" << bucket_301_600 << ", [601+]=" << bucket_601_plus;
}

bool mapSampleToCost(const CHDataFacade &facade,
                     const SamplePoint &sample,
                     const MetricKind metric_kind,
                     const PHASTOrientation orientation,
                     const std::vector<EdgeWeight> &costs,
                     EdgeWeight &mapped_cost,
                     RasterizationStats &stats,
                     SampleMappingTrace &trace)
{
    trace = {};
    mapped_cost = INVALID_EDGE_WEIGHT;
    auto candidates = facade.NearestPhantomNodes(sample.coordinate,
                                                 MAX_SNAP_CANDIDATES,
                                                 std::nullopt,
                                                 std::nullopt,
                                                 engine::Approach::UNRESTRICTED);
    trace.has_snap_candidates = !candidates.empty();
    trace.snap_candidate_count = candidates.size();
    if (candidates.empty())
    {
        ++stats.no_candidate_samples;
        return true;
    }

    auto is_state_valid = [&](const engine::PhantomNode &phantom, const bool forward_state) -> bool
    {
        if (orientation == PHASTOrientation::Reverse)
        {
            return forward_state ? phantom.IsValidForwardSource() : phantom.IsValidReverseSource();
        }
        return forward_state ? phantom.IsValidForwardTarget() : phantom.IsValidReverseTarget();
    };

    auto convert_state_offset = [&](const engine::PhantomNode &phantom,
                                    const bool forward_state,
                                    EdgeWeight &offset) -> bool
    {
        if (metric_kind == MetricKind::Duration)
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
            offset = to_alias<EdgeWeight>(duration_value);
            return true;
        }

        offset = forward_state ? phantom.GetForwardWeightPlusOffset()
                               : phantom.GetReverseWeightPlusOffset();
        return offset != INVALID_EDGE_WEIGHT && offset >= EdgeWeight{0};
    };

    auto try_add_cost = [](const EdgeWeight lhs, const EdgeWeight rhs, EdgeWeight &sum) -> bool
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
    };

    bool any_valid_state = false;
    bool any_reachable_state = false;
    for (const auto candidate_index : util::irange<std::size_t>(0, candidates.size()))
    {
        const auto &phantom = candidates[candidate_index].phantom_node;
        EdgeWeight best_candidate_cost = INVALID_EDGE_WEIGHT;
        NodeID best_candidate_state_node = SPECIAL_SEGMENTID;
        bool best_candidate_state_forward = false;
        EdgeWeight best_candidate_state_cost = INVALID_EDGE_WEIGHT;
        EdgeWeight best_candidate_source_offset = INVALID_EDGE_WEIGHT;
        bool best_candidate_has_state = false;
        bool candidate_has_valid_state = false;
        bool candidate_has_reachable_state = false;

        auto evaluate_state = [&](const SegmentID segment_id, const bool forward_state) -> bool
        {
            if (!segment_id.enabled || !is_state_valid(phantom, forward_state))
            {
                return true;
            }

            candidate_has_valid_state = true;
            const auto node_id = segment_id.id;
            if (node_id >= costs.size())
            {
                util::Log(logERROR) << "Nearest phantom node id out of bounds: " << node_id;
                return false;
            }

            const auto state_cost = costs[node_id];
            if (state_cost == INVALID_EDGE_WEIGHT)
            {
                return true;
            }
            candidate_has_reachable_state = true;

            EdgeWeight source_offset = INVALID_EDGE_WEIGHT;
            if (!convert_state_offset(phantom, forward_state, source_offset))
            {
                return true;
            }

            EdgeWeight total_cost = INVALID_EDGE_WEIGHT;
            if (!try_add_cost(state_cost, source_offset, total_cost))
            {
                util::Log(logERROR) << "Overflow while adding source offset to state cost.";
                return false;
            }

            if (best_candidate_cost == INVALID_EDGE_WEIGHT || total_cost < best_candidate_cost)
            {
                best_candidate_cost = total_cost;
                best_candidate_state_node = node_id;
                best_candidate_state_forward = forward_state;
                best_candidate_state_cost = state_cost;
                best_candidate_source_offset = source_offset;
                best_candidate_has_state = true;
            }
            return true;
        };

        if (!evaluate_state(phantom.forward_segment_id, true) ||
            !evaluate_state(phantom.reverse_segment_id, false))
        {
            return false;
        }

        any_valid_state = any_valid_state || candidate_has_valid_state;
        any_reachable_state = any_reachable_state || candidate_has_reachable_state;

        if (best_candidate_cost == INVALID_EDGE_WEIGHT)
        {
            continue;
        }

        mapped_cost = best_candidate_cost;
        trace.chosen_candidate_index = candidate_index;
        trace.chosen_forward_segment_id = phantom.forward_segment_id.id;
        trace.chosen_forward_segment_enabled = phantom.forward_segment_id.enabled;
        trace.chosen_reverse_segment_id = phantom.reverse_segment_id.id;
        trace.chosen_reverse_segment_enabled = phantom.reverse_segment_id.enabled;
        if (best_candidate_has_state)
        {
            trace.has_chosen_state = true;
            trace.chosen_state_node_id = best_candidate_state_node;
            trace.chosen_state_forward = best_candidate_state_forward;
            trace.chosen_state_cost = best_candidate_state_cost;
            trace.chosen_source_offset = best_candidate_source_offset;
            trace.chosen_total_cost = best_candidate_cost;
        }
        if (candidate_index > 0)
        {
            ++stats.fallback_candidate_used_samples;
        }
        break;
    }

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

bool rasterizeToCells(const CHDataFacade &facade,
                      const std::vector<SamplePoint> &samples,
                      const MetricKind metric_kind,
                      const PHASTOrientation orientation,
                      const std::vector<EdgeWeight> &costs,
                      std::unordered_map<std::string, CellAggregate> &cells,
                      std::vector<EdgeWeight> &sample_costs,
                      std::vector<SampleMappingTrace> &sample_traces,
                      RasterizationStats &stats)
{
    cells.clear();
    sample_costs.assign(samples.size(), INVALID_EDGE_WEIGHT);
    sample_traces.assign(samples.size(), SampleMappingTrace{});
    stats = {};
    if (costs.empty())
    {
        util::Log(logERROR) << "Cost vector is empty.";
        return false;
    }

    for (const auto sample_index : util::irange<std::size_t>(0, samples.size()))
    {
        const auto &sample = samples[sample_index];
        EdgeWeight mapped_cost = INVALID_EDGE_WEIGHT;
        if (!mapSampleToCost(
                facade, sample, metric_kind, orientation, costs, mapped_cost, stats, sample_traces[sample_index]))
        {
            return false;
        }
        sample_costs[sample_index] = mapped_cost;

        auto &cell = cells[sample.h3_id];
        if (cell.sample_count == 0)
        {
            cell.res = sample.res;
        }
        else if (cell.res != sample.res)
        {
            util::Log(logERROR) << "Sample file mixes resolutions for h3_id " << sample.h3_id;
            return false;
        }
        ++cell.sample_count;

        if (mapped_cost == INVALID_EDGE_WEIGHT)
        {
            continue;
        }
        ++cell.reachable_count;
        if (cell.min_cost == INVALID_EDGE_WEIGHT || mapped_cost < cell.min_cost)
        {
            cell.min_cost = mapped_cost;
        }
    }

    return !cells.empty();
}

bool writeCellCostArtifact(const std::filesystem::path &output_path,
                           const std::string &metric_name,
                           const std::unordered_map<std::string, CellAggregate> &cells,
                           std::size_t &reachable_cells,
                           std::size_t &unreachable_cells)
{
    std::ofstream output(output_path);
    if (!output)
    {
        util::Log(logERROR) << "Could not open output artifact path: " << output_path.string();
        return false;
    }

    std::vector<std::string> ids;
    ids.reserve(cells.size());
    for (const auto &pair : cells)
    {
        ids.push_back(pair.first);
    }
    std::sort(ids.begin(), ids.end());

    output << "h3_id,res,sample_count,reachable_samples,cost_ticks,cost_seconds\n";

    reachable_cells = 0;
    unreachable_cells = 0;
    for (const auto &h3_id : ids)
    {
        const auto &cell = cells.at(h3_id);
        output << h3_id << "," << cell.res << "," << cell.sample_count << "," << cell.reachable_count
               << ",";
        if (cell.min_cost == INVALID_EDGE_WEIGHT)
        {
            ++unreachable_cells;
            output << ",\n";
            continue;
        }

        ++reachable_cells;
        const auto ticks = from_alias<std::int64_t>(cell.min_cost);
        output << ticks << ",";
        if (metric_name == "duration")
        {
            output << std::fixed << std::setprecision(1) << (ticks / 10.0);
        }
        output << "\n";
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
    util::Log() << "Phastfield: " << runtime_config.phastfield_path.string();
    util::Log() << "Sample file: " << runtime_config.sample_file_path.string();
    util::Log() << "Output artifact: " << runtime_config.output_path.string();
    util::Log() << "Expected H3 resolution: " << runtime_config.expected_resolution;
    if (runtime_config.oracle_enabled)
    {
        util::Log() << "Oracle POI file: " << runtime_config.oracle_poi_file.string();
        util::Log() << "Oracle max samples: "
                    << (runtime_config.oracle_max_samples == 0
                            ? std::string("all")
                            : std::to_string(runtime_config.oracle_max_samples));
        util::Log() << "Oracle tolerance (ticks): " << runtime_config.oracle_tolerance_ticks;
        util::Log() << "Oracle trace limit: " << runtime_config.oracle_trace_limit;
        util::Log() << "Oracle strict endpoints: "
                    << (runtime_config.oracle_strict_endpoints ? "yes" : "no");
        util::Log() << "Oracle report all comparisons: "
                    << (runtime_config.oracle_report_all ? "yes" : "no");
        if (!runtime_config.oracle_mismatch_report_path.empty())
        {
            util::Log() << "Oracle mismatch report: "
                        << runtime_config.oracle_mismatch_report_path.string();
        }
    }

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(rasterize_config.GetPath(".osrm.properties"), properties);

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

    std::vector<SamplePoint> samples;
    if (!loadSamplePoints(runtime_config.sample_file_path, runtime_config.expected_resolution, samples))
    {
        return EXIT_FAILURE;
    }
    util::Log() << "Loaded sample points: " << samples.size();

    std::unordered_map<std::string, CellAggregate> cells;
    std::vector<EdgeWeight> sample_costs;
    std::vector<SampleMappingTrace> sample_traces;
    RasterizationStats rasterization_stats;
    if (!rasterizeToCells(*facade,
                          samples,
                          field_data.metric_kind,
                          field_data.orientation,
                          field_data.costs,
                          cells,
                          sample_costs,
                          sample_traces,
                          rasterization_stats))
    {
        return EXIT_FAILURE;
    }

    bool oracle_failed = false;
    if (runtime_config.oracle_enabled)
    {
        if (field_data.metric_kind != MetricKind::Duration)
        {
            util::Log(logERROR)
                << "Oracle mode is only supported for duration metric phastfield artifacts.";
            return EXIT_FAILURE;
        }
        if (runtime_config.oracle_report_all && runtime_config.oracle_mismatch_report_path.empty())
        {
            util::Log(logERROR) << "--oracle-report-all requires --oracle-mismatch-report.";
            return EXIT_FAILURE;
        }

        std::vector<util::Coordinate> oracle_pois;
        if (!loadPOICoordinates(runtime_config.oracle_poi_file, oracle_pois))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "Loaded oracle POIs: " << oracle_pois.size();

        OracleStats oracle_stats;
        std::vector<OracleComparison> oracle_comparisons;
        if (!runDurationOracle(rasterize_config.base_path,
                               field_data.orientation,
                               samples,
                               sample_costs,
                               sample_traces,
                               oracle_pois,
                               runtime_config.oracle_max_samples,
                               runtime_config.oracle_tolerance_ticks,
                               runtime_config.oracle_mismatch_report_path,
                               runtime_config.oracle_report_all,
                               runtime_config.oracle_trace_limit,
                               oracle_stats,
                               oracle_comparisons))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "Oracle compared samples: " << oracle_stats.compared_samples;
        util::Log() << "Oracle reachable sample pairs: " << oracle_stats.reachable_pairs;
        util::Log() << "Oracle unreachable sample pairs: " << oracle_stats.unreachable_pairs;
        util::Log() << "Oracle max abs error (ticks): " << oracle_stats.max_abs_error_ticks;

        std::vector<std::int64_t> coordinate_deltas;
        coordinate_deltas.reserve(oracle_comparisons.size());
        for (const auto &comparison : oracle_comparisons)
        {
            if (comparison.raster_reachable && comparison.oracle_reachable)
            {
                coordinate_deltas.push_back(comparison.delta_ticks);
            }
        }
        logDeltaBuckets("Coordinate oracle", coordinate_deltas);

        if (runtime_config.oracle_strict_endpoints)
        {
            StrictOracleStats strict_stats;
            std::vector<std::int64_t> strict_deltas;
            std::vector<std::int64_t> state_deltas;
            if (!runStrictEndpointOracle(rasterize_config.base_path,
                                         *facade,
                                         field_data.orientation,
                                         samples,
                                         sample_costs,
                                         sample_traces,
                                         oracle_pois,
                                         oracle_comparisons,
                                         runtime_config.oracle_tolerance_ticks,
                                         runtime_config.oracle_trace_limit,
                                         strict_stats,
                                         strict_deltas,
                                         state_deltas))
            {
                return EXIT_FAILURE;
            }
            util::Log() << "Strict endpoint compared samples: " << strict_stats.compared_samples;
            util::Log() << "Strict endpoint reachable sample pairs: " << strict_stats.reachable_pairs;
            util::Log() << "Strict endpoint unreachable sample pairs: " << strict_stats.unreachable_pairs;
            util::Log() << "Strict endpoint mismatches: " << strict_stats.mismatches;
            util::Log() << "Strict endpoint max abs error (ticks): " << strict_stats.max_abs_error_ticks;
            util::Log() << "Coordinate mismatches resolved by strict endpoint oracle: "
                        << strict_stats.coordinate_mismatches_resolved;
            logDeltaBuckets("Strict endpoint oracle", strict_deltas);
            util::Log() << "Graph-state oracle compared samples: " << strict_stats.state_compared_samples;
            util::Log() << "Graph-state oracle reachable sample pairs: "
                        << strict_stats.state_reachable_pairs;
            util::Log() << "Graph-state oracle unreachable sample pairs: "
                        << strict_stats.state_unreachable_pairs;
            util::Log() << "Graph-state oracle mismatches: " << strict_stats.state_mismatches;
            util::Log() << "Graph-state oracle max abs error (ticks): "
                        << strict_stats.state_max_abs_error_ticks;
            util::Log() << "Endpoint-semantics-only mismatches (strict mismatch, graph-state pass): "
                        << strict_stats.endpoint_semantics_mismatch_samples;
            logDeltaBuckets("Graph-state oracle", state_deltas);
        }

        if (oracle_stats.mismatches > 0)
        {
            util::Log(logERROR) << "Coordinate oracle mismatches: " << oracle_stats.mismatches
                                << " of " << oracle_stats.compared_samples
                                << " compared samples (tolerance "
                                << runtime_config.oracle_tolerance_ticks << " ticks).";
            oracle_failed = true;
        }
    }

    std::size_t reachable_cells = 0;
    std::size_t unreachable_cells = 0;
    if (!writeCellCostArtifact(runtime_config.output_path,
                               field_data.metric_name,
                               cells,
                               reachable_cells,
                               unreachable_cells))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Rasterized cells: " << cells.size();
    util::Log() << "Reachable cells: " << reachable_cells;
    util::Log() << "Unreachable cells: " << unreachable_cells;
    util::Log() << "Mapped samples: " << rasterization_stats.mapped_samples;
    util::Log() << "Fallback snap candidate used: "
                << rasterization_stats.fallback_candidate_used_samples;
    util::Log() << "Samples with no snap candidates: " << rasterization_stats.no_candidate_samples;
    util::Log() << "Samples with no valid start states: " << rasterization_stats.no_valid_state_samples;
    util::Log() << "Samples with valid states but unreachable field values: "
                << rasterization_stats.unreachable_state_samples;
    util::Log() << "PHAST rasterization prototype complete.";

    util::DumpMemoryStats();
    if (oracle_failed)
    {
        return EXIT_FAILURE;
    }
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
