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

#include <algorithm>
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
    std::uint32_t expected_resolution = 9;
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

using CHDataFacade =
    engine::datafacade::ContiguousInternalMemoryDataFacade<engine::routing_algorithms::ch::Algorithm>;

constexpr std::uint32_t PHASTFIELD_SCHEMA_VERSION = 1;

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
        "Expected H3 resolution in sample file (default 9)");

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

bool mapSampleToCost(const CHDataFacade &facade,
                     const SamplePoint &sample,
                     const std::vector<EdgeWeight> &costs,
                     EdgeWeight &mapped_cost)
{
    mapped_cost = INVALID_EDGE_WEIGHT;
    auto candidates = facade.NearestPhantomNodes(sample.coordinate,
                                                 1,
                                                 std::nullopt,
                                                 std::nullopt,
                                                 engine::Approach::UNRESTRICTED);
    if (candidates.empty())
    {
        return false;
    }

    const auto &phantom = candidates.front().phantom_node;
    auto update_cost = [&](const SegmentID segment_id) -> bool
    {
        if (!segment_id.enabled)
        {
            return true;
        }

        const auto node_id = segment_id.id;
        if (node_id >= costs.size())
        {
            util::Log(logERROR) << "Nearest phantom node id out of bounds: " << node_id;
            return false;
        }

        const auto candidate_cost = costs[node_id];
        if (candidate_cost == INVALID_EDGE_WEIGHT)
        {
            return true;
        }
        if (mapped_cost == INVALID_EDGE_WEIGHT || candidate_cost < mapped_cost)
        {
            mapped_cost = candidate_cost;
        }
        return true;
    };

    if (!update_cost(phantom.forward_segment_id) || !update_cost(phantom.reverse_segment_id))
    {
        return false;
    }
    return true;
}

bool rasterizeToCells(const CHDataFacade &facade,
                      const std::vector<SamplePoint> &samples,
                      const std::vector<EdgeWeight> &costs,
                      std::unordered_map<std::string, CellAggregate> &cells)
{
    cells.clear();
    if (costs.empty())
    {
        util::Log(logERROR) << "Cost vector is empty.";
        return false;
    }

    for (const auto &sample : samples)
    {
        EdgeWeight mapped_cost = INVALID_EDGE_WEIGHT;
        if (!mapSampleToCost(facade, sample, costs, mapped_cost))
        {
            return false;
        }

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
    if (!rasterizeToCells(*facade, samples, field_data.costs, cells))
    {
        return EXIT_FAILURE;
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
