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
    bool oracle_enabled = false;
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
        "Optional CSV path for oracle mismatches");

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

bool runDurationOracle(const std::filesystem::path &base_path,
                       const PHASTOrientation orientation,
                       const std::vector<SamplePoint> &samples,
                       const std::vector<EdgeWeight> &sample_costs,
                       const std::vector<util::Coordinate> &pois,
                       const std::uint32_t oracle_max_samples,
                       const std::int64_t tolerance_ticks,
                       const std::filesystem::path &mismatch_report_path,
                       OracleStats &stats)
{
    stats = {};
    if (samples.size() != sample_costs.size())
    {
        util::Log(logERROR) << "Oracle input mismatch: samples and sample costs differ in size.";
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
            << "sample_index,h3_id,lon,lat,raster_ticks,oracle_ticks,oracle_seconds,abs_error_ticks,reason\n";
    }

    for (const auto local_sample_index : util::irange<std::size_t>(0, selected_sample_indices.size()))
    {
        const auto sample_index = selected_sample_indices[local_sample_index];
        const auto &sample = samples[sample_index];

        bool oracle_reachable = false;
        std::int64_t oracle_best_ticks = 0;

        if (orientation == PHASTOrientation::Reverse)
        {
            const auto &row = std::get<json::Array>(durations_rows[local_sample_index]).values;
            if (row.size() != pois.size())
            {
                util::Log(logERROR) << "Unexpected OSRM table oracle column count for sample row "
                                    << local_sample_index << ".";
                return false;
            }

            for (const auto &entry : row)
            {
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

        bool mismatch = false;
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
            const auto raster_ticks = from_alias<std::int64_t>(raster_cost);
            abs_error_ticks = std::llabs(raster_ticks - oracle_best_ticks);
            if (abs_error_ticks > tolerance_ticks)
            {
                mismatch = true;
                mismatch_reason = "abs_error_exceeds_tolerance";
            }
            stats.max_abs_error_ticks = std::max(stats.max_abs_error_ticks, abs_error_ticks);
        }

        if (!mismatch)
        {
            continue;
        }

        ++stats.mismatches;
        if (stats.mismatches <= 10)
        {
            util::Log(logERROR) << "Oracle mismatch at sample_index=" << sample.sample_index
                                << " h3_id=" << sample.h3_id
                                << " reason=" << mismatch_reason;
        }
        if (!mismatch_report_path.empty())
        {
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
            mismatch_report << "," << abs_error_ticks << "," << mismatch_reason << "\n";
        }
    }

    if (stats.mismatches > 0)
    {
        util::Log(logERROR) << "Oracle validation failed with " << stats.mismatches
                            << " mismatches out of " << stats.compared_samples << " compared samples.";
        return false;
    }

    return true;
}

bool mapSampleToCost(const CHDataFacade &facade,
                     const SamplePoint &sample,
                     const MetricKind metric_kind,
                     const PHASTOrientation orientation,
                     const std::vector<EdgeWeight> &costs,
                     EdgeWeight &mapped_cost,
                     RasterizationStats &stats)
{
    mapped_cost = INVALID_EDGE_WEIGHT;
    auto candidates = facade.NearestPhantomNodes(sample.coordinate,
                                                 MAX_SNAP_CANDIDATES,
                                                 std::nullopt,
                                                 std::nullopt,
                                                 engine::Approach::UNRESTRICTED);
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
                      RasterizationStats &stats)
{
    cells.clear();
    sample_costs.assign(samples.size(), INVALID_EDGE_WEIGHT);
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
        if (!mapSampleToCost(facade, sample, metric_kind, orientation, costs, mapped_cost, stats))
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
    RasterizationStats rasterization_stats;
    if (!rasterizeToCells(*facade,
                          samples,
                          field_data.metric_kind,
                          field_data.orientation,
                          field_data.costs,
                          cells,
                          sample_costs,
                          rasterization_stats))
    {
        return EXIT_FAILURE;
    }

    if (runtime_config.oracle_enabled)
    {
        if (field_data.metric_kind != MetricKind::Duration)
        {
            util::Log(logERROR)
                << "Oracle mode is only supported for duration metric phastfield artifacts.";
            return EXIT_FAILURE;
        }

        std::vector<util::Coordinate> oracle_pois;
        if (!loadPOICoordinates(runtime_config.oracle_poi_file, oracle_pois))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "Loaded oracle POIs: " << oracle_pois.size();

        OracleStats oracle_stats;
        if (!runDurationOracle(rasterize_config.base_path,
                               field_data.orientation,
                               samples,
                               sample_costs,
                               oracle_pois,
                               runtime_config.oracle_max_samples,
                               runtime_config.oracle_tolerance_ticks,
                               runtime_config.oracle_mismatch_report_path,
                               oracle_stats))
        {
            return EXIT_FAILURE;
        }
        util::Log() << "Oracle compared samples: " << oracle_stats.compared_samples;
        util::Log() << "Oracle reachable sample pairs: " << oracle_stats.reachable_pairs;
        util::Log() << "Oracle unreachable sample pairs: " << oracle_stats.unreachable_pairs;
        util::Log() << "Oracle max abs error (ticks): " << oracle_stats.max_abs_error_ticks;
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
