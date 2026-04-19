#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"
#include "osrm/exception.hpp"
#include "storage/io_config.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/version.hpp"

#include <boost/program_options.hpp>

#include <cstdlib>
#include <filesystem>
#include <set>
#include <unordered_map>

using namespace osrm;

namespace
{
struct PhastConfig final : storage::IOConfig
{
    PhastConfig() : IOConfig({".osrm.hsgr", ".osrm.properties", ".osrm.phast"}, {}, {}) {}

    void UseDefaultOutputNames(const std::filesystem::path &base) { IOConfig::UseDefaultOutputNames(base); }

    bool IsValid() const { return IOConfig::IsValid(); }
};

enum class return_code : unsigned
{
    ok,
    fail,
    exit
};

return_code
parseArguments(int argc, char *argv[], std::string &verbosity, PhastConfig &phast_config)
{
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
        "list-inputs", "List required and optional input file extensions")(
        "verbosity,l",
        boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
        std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i",
        boost::program_options::value<std::filesystem::path>(&phast_config.base_path),
        "Input base file path");

    boost::program_options::positional_options_description positional_options;
    positional_options.add("input", 1);

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(hidden_options);

    const auto *executable = argv[0];
    boost::program_options::options_description visible_options(
        "Usage: " + std::filesystem::path(executable).filename().string() + " <input.osrm> [options]");
    visible_options.add(generic_options);

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
        PhastConfig config;
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

    return return_code::ok;
}
} // namespace

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();

    std::string verbosity;
    PhastConfig phast_config;

    const auto result = parseArguments(argc, argv, verbosity, phast_config);
    if (return_code::fail == result)
    {
        return EXIT_FAILURE;
    }
    if (return_code::exit == result)
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

    std::unordered_map<std::string, contractor::ContractedMetric> metrics = {{metric_name, {}}};
    std::uint32_t hsgr_checksum = 0;
    contractor::files::readGraph(phast_config.GetPath(".osrm.hsgr"), metrics, hsgr_checksum);

    contractor::PhastData phast_data;
    contractor::files::readPhast(phast_config.GetPath(".osrm.phast"), phast_data);

    const auto &graph = metrics.at(metric_name).graph;
    util::Log() << "CH metric: " << metric_name;
    util::Log() << "CH nodes: " << graph.GetNumberOfNodes() << ", edges: " << graph.GetNumberOfEdges();
    util::Log() << "PHAST version: " << phast_data.version;
    util::Log() << "PHAST metric: " << phast_data.metric_name;
    util::Log() << "PHAST nodes: " << phast_data.node_count;
    util::Log() << "PHAST checksum: " << phast_data.connectivity_checksum;
    util::Log() << "HSGR checksum: " << hsgr_checksum;

    if (metric_name != phast_data.metric_name)
    {
        util::Log(logWARNING) << "Metric name mismatch between .osrm.properties and .osrm.phast";
    }
    if (hsgr_checksum != phast_data.connectivity_checksum)
    {
        util::Log(logWARNING) << "Checksum mismatch between .osrm.hsgr and .osrm.phast";
    }

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
