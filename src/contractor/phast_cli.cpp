#include "contractor/phast_cli.hpp"

#include "util/log.hpp"
#include "util/version.hpp"

#include <boost/program_options.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>

namespace osrm::contractor::phast
{

ReturnCode ParseArguments(int argc,
                          char *argv[],
                          std::string &verbosity,
                          PhastConfig &phast_config,
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
        "seed-node",
        boost::program_options::value<std::vector<NodeID>>(&runtime_config.seed_nodes)
            ->multitoken()
            ->composing(),
        "Seed node id(s) in CH node space")(
        "seed-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.seed_file),
        "Path to a text file containing one seed node id per line")(
        "poi-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.poi_file),
        "Path to a text file containing one POI coordinate per line as 'lon,lat'")(
        "metric",
        boost::program_options::value<std::string>(&runtime_config.metric)->default_value("weight"),
        "Traversal metric: weight|duration")(
        "seed-metric",
        boost::program_options::value<std::string>(&runtime_config.seed_metric),
        "Seed offset metric override: weight|duration (default: --metric)")(
        "cap-seconds",
        boost::program_options::value<double>(&runtime_config.cap_seconds),
        "Traversal cap in seconds")(
        "cap-weight",
        boost::program_options::value<std::uint64_t>(&runtime_config.cap_weight),
        "Traversal cap in metric ticks (required for non-duration weight datasets)")(
        "exclude",
        boost::program_options::value<std::size_t>(&runtime_config.exclude_index),
        "Expected exclude index (must match .osrm.phast)")(
        "orientation",
        boost::program_options::value<std::string>(&runtime_config.orientation),
        "Expected orientation: forward|reverse (must match .osrm.phast)")(
        "output-format",
        boost::program_options::value<std::string>(&runtime_config.output_format)
            ->default_value("phastfield-v1"),
        "Output format: phastfield-v1|raw-u32")(
        "output",
        boost::program_options::value<std::filesystem::path>(&runtime_config.output_path),
        "Output path (default: <input>.phastfield or <input>.phastfield.raw_u32)")(
        "sample-file",
        boost::program_options::value<std::filesystem::path>(&runtime_config.sample_file),
        "CSV file with rows: h3_id,res,sample_index,lon,lat")(
        "sample-snap-cache",
        boost::program_options::value<std::filesystem::path>(&runtime_config.sample_snap_cache),
        "Primary snap cache file for sample points")(
        "raster-output",
        boost::program_options::value<std::filesystem::path>(&runtime_config.raster_output_path),
        "Output dense little-endian uint16 artifact path (.u16)")(
        "resolution",
        boost::program_options::value<std::uint32_t>(&runtime_config.expected_resolution)
            ->default_value(9),
        "Expected H3 resolution in sample file/cache (default 9)");

    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i",
        boost::program_options::value<std::filesystem::path>(&phast_config.base_path),
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
        return ReturnCode::Fail;
    }

    if (option_variables.contains("version"))
    {
        std::cout << OSRM_VERSION << std::endl;
        return ReturnCode::Exit;
    }

    if (option_variables.contains("help"))
    {
        std::cout << visible_options;
        return ReturnCode::Exit;
    }

    if (option_variables.contains("list-inputs"))
    {
        PhastConfig config;
        std::set<std::string> seen;
        config.ListInputFiles(std::cout, seen);
        return ReturnCode::Exit;
    }

    boost::program_options::notify(option_variables);

    if (!option_variables.contains("input"))
    {
        std::cout << visible_options;
        return ReturnCode::Fail;
    }
    if (option_variables.contains("seed-file"))
    {
        runtime_config.has_seed_file = true;
    }
    if (option_variables.contains("poi-file"))
    {
        runtime_config.has_poi_file = true;
    }
    if (option_variables.contains("seed-metric"))
    {
        runtime_config.has_seed_metric = true;
    }
    else
    {
        runtime_config.seed_metric = runtime_config.metric;
    }
    if (option_variables.contains("cap-seconds"))
    {
        runtime_config.has_cap_seconds = true;
    }
    if (option_variables.contains("cap-weight"))
    {
        runtime_config.has_cap_weight = true;
    }
    if (option_variables.contains("exclude"))
    {
        runtime_config.has_exclude_index = true;
    }
    if (option_variables.contains("orientation"))
    {
        runtime_config.has_orientation = true;
    }
    if (option_variables.contains("output"))
    {
        runtime_config.has_output_path = true;
    }
    if (option_variables.contains("sample-file"))
    {
        runtime_config.has_sample_file = true;
    }
    if (option_variables.contains("sample-snap-cache"))
    {
        runtime_config.has_sample_snap_cache = true;
    }
    if (option_variables.contains("raster-output"))
    {
        runtime_config.has_raster_output_path = true;
    }

    const bool has_manual_seeds = !runtime_config.seed_nodes.empty() || runtime_config.has_seed_file;
    if (has_manual_seeds == runtime_config.has_poi_file)
    {
        util::Log(logERROR)
            << "Provide exactly one seed source: --poi-file or --seed-node/--seed-file.";
        return ReturnCode::Fail;
    }
    if ((runtime_config.metric != "weight") && (runtime_config.metric != "duration"))
    {
        util::Log(logERROR) << "--metric must be 'weight' or 'duration'.";
        return ReturnCode::Fail;
    }
    if ((runtime_config.seed_metric != "weight") && (runtime_config.seed_metric != "duration"))
    {
        util::Log(logERROR) << "--seed-metric must be 'weight' or 'duration'.";
        return ReturnCode::Fail;
    }
    if (runtime_config.has_cap_seconds &&
        (!std::isfinite(runtime_config.cap_seconds) || runtime_config.cap_seconds < 0.))
    {
        util::Log(logERROR) << "--cap-seconds must be >= 0.";
        return ReturnCode::Fail;
    }
    if (runtime_config.has_cap_seconds && runtime_config.has_cap_weight)
    {
        util::Log(logERROR) << "Specify at most one traversal cap: --cap-seconds or --cap-weight.";
        return ReturnCode::Fail;
    }
    if (runtime_config.output_format != "phastfield-v1" && runtime_config.output_format != "raw-u32")
    {
        util::Log(logERROR) << "--output-format must be 'phastfield-v1' or 'raw-u32'.";
        return ReturnCode::Fail;
    }
    if (runtime_config.has_orientation)
    {
        if (runtime_config.orientation != "forward" && runtime_config.orientation != "reverse")
        {
            util::Log(logERROR) << "--orientation must be 'forward' or 'reverse'.";
            return ReturnCode::Fail;
        }
    }
    if (runtime_config.expected_resolution == 0)
    {
        util::Log(logERROR) << "--resolution must be > 0.";
        return ReturnCode::Fail;
    }
    if (runtime_config.has_raster_output_path)
    {
        if (runtime_config.has_output_path)
        {
            util::Log(logERROR) << "Specify only one of --output or --raster-output.";
            return ReturnCode::Fail;
        }
        if (!runtime_config.has_sample_file || !runtime_config.has_sample_snap_cache)
        {
            util::Log(logERROR)
                << "--raster-output requires both --sample-file and --sample-snap-cache.";
            return ReturnCode::Fail;
        }
    }

    return ReturnCode::Ok;
}

} // namespace osrm::contractor::phast
