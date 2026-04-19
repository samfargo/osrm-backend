#include "contractor/contracted_metric.hpp"
#include "contractor/files.hpp"
#include "extractor/files.hpp"
#include "extractor/profile_properties.hpp"
#include "osrm/exception.hpp"
#include "storage/io_config.hpp"
#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/version.hpp"

#include <boost/program_options.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <set>
#include <unordered_map>
#include <vector>

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

bool validateOrdering(const contractor::PhastData &phast_data)
{
    const auto node_count = phast_data.node_count;
    const auto &ordering = phast_data.ordering;

    if (ordering.order.size() != node_count || ordering.rank.size() != node_count)
    {
        util::Log(logERROR) << "PHAST ordering size mismatch.";
        return false;
    }
    if (ordering.contraction_to_original.size() != node_count ||
        ordering.original_to_contraction.size() != node_count)
    {
        util::Log(logERROR) << "PHAST permutation size mismatch.";
        return false;
    }

    for (const auto node : ordering.rank)
    {
        if (node >= node_count)
        {
            util::Log(logERROR) << "PHAST rank contains invalid node id.";
            return false;
        }
    }

    for (const auto i : util::irange<std::size_t>(0, node_count))
    {
        const auto node = ordering.order[i];
        if (node >= node_count)
        {
            util::Log(logERROR) << "PHAST order contains invalid node id at position " << i;
            return false;
        }
        if (ordering.rank[node] != i)
        {
            util::Log(logERROR) << "PHAST rank/order inverse mismatch at position " << i;
            return false;
        }
    }

    for (const auto local_id : util::irange<NodeID>(0, node_count))
    {
        if (ordering.contraction_to_original[local_id] >= node_count)
        {
            util::Log(logERROR) << "PHAST contraction_to_original contains invalid node id.";
            return false;
        }
    }

    for (const auto original_id : util::irange<NodeID>(0, node_count))
    {
        const auto local_id = ordering.original_to_contraction[original_id];
        if (local_id >= node_count || ordering.contraction_to_original[local_id] != original_id)
        {
            util::Log(logERROR) << "PHAST permutation inverse mismatch at node " << original_id;
            return false;
        }
    }

    return true;
}

const char *orientationToString(const contractor::PHASTOrientation orientation)
{
    switch (orientation)
    {
    case contractor::PHASTOrientation::Forward:
        return "forward";
    case contractor::PHASTOrientation::Reverse:
        return "reverse";
    }
    return "unknown";
}

bool isValidOrientation(const contractor::PHASTOrientation orientation)
{
    return orientation == contractor::PHASTOrientation::Forward ||
           orientation == contractor::PHASTOrientation::Reverse;
}

bool usesArc(const contractor::QueryEdge::EdgeData &edge_data,
             const contractor::PHASTOrientation orientation)
{
    switch (orientation)
    {
    case contractor::PHASTOrientation::Forward:
        return edge_data.forward;
    case contractor::PHASTOrientation::Reverse:
        return edge_data.backward;
    }
    return false;
}

bool validateMetadata(const std::string &metric_name,
                      const contractor::ContractedMetric &metric,
                      const std::uint32_t hsgr_checksum,
                      const contractor::PhastData &phast_data)
{
    if (phast_data.version < 3)
    {
        util::Log(logERROR) << "Unsupported .osrm.phast version " << phast_data.version
                            << ". Expected version >= 3.";
        return false;
    }
    if (!isValidOrientation(phast_data.orientation))
    {
        util::Log(logERROR) << "Unsupported PHAST orientation value.";
        return false;
    }
    if (metric_name != phast_data.metric_name)
    {
        util::Log(logERROR) << "Metric name mismatch between .osrm.properties and .osrm.phast";
        return false;
    }
    if (hsgr_checksum != phast_data.connectivity_checksum)
    {
        util::Log(logERROR) << "Checksum mismatch between .osrm.hsgr and .osrm.phast";
        return false;
    }
    if (metric.graph.GetNumberOfNodes() != phast_data.node_count)
    {
        util::Log(logERROR) << "Node count mismatch between .osrm.hsgr and .osrm.phast";
        return false;
    }
    if (phast_data.exclude_index >= metric.edge_filter.size())
    {
        util::Log(logERROR) << "PHAST exclude index " << phast_data.exclude_index
                            << " is out of range for this CH metric.";
        return false;
    }
    return true;
}

struct DerivedAdjacency
{
    std::vector<std::uint32_t> up_offsets;
    std::vector<NodeID> up_targets;
    std::vector<std::uint32_t> down_offsets;
    std::vector<NodeID> down_targets;
    std::size_t skipped_self_loops = 0;
};

bool classifyArc(const contractor::PhastData &phast_data,
                 const NodeID source,
                 const NodeID target,
                 bool &is_upward)
{
    const auto source_rank = phast_data.ordering.rank[source];
    const auto target_rank = phast_data.ordering.rank[target];
    if (source_rank == target_rank)
    {
        util::Log(logERROR) << "Rank tie detected for edge " << source << " -> " << target;
        return false;
    }

    is_upward = source_rank < target_rank;
    return true;
}

bool deriveAdjacency(const contractor::QueryGraph &graph,
                     const contractor::PhastData &phast_data,
                     DerivedAdjacency &adjacency)
{
    const auto node_count = phast_data.node_count;
    adjacency.up_offsets.assign(node_count + 1, 0);
    adjacency.down_offsets.assign(node_count + 1, 0);

    for (const auto source : util::irange<NodeID>(0, node_count))
    {
        for (const auto edge : graph.GetAdjacentEdgeRange(source))
        {
            const auto target = graph.GetTarget(edge);
            const auto &edge_data = graph.GetEdgeData(edge);
            if (!usesArc(edge_data, phast_data.orientation))
            {
                continue;
            }
            if (source == target)
            {
                ++adjacency.skipped_self_loops;
                continue;
            }

            bool is_upward = false;
            if (!classifyArc(phast_data, source, target, is_upward))
            {
                return false;
            }

            if (is_upward)
            {
                ++adjacency.up_offsets[source + 1];
            }
            else
            {
                ++adjacency.down_offsets[source + 1];
            }
        }
    }

    for (const auto index : util::irange<std::size_t>(1, adjacency.up_offsets.size()))
    {
        adjacency.up_offsets[index] += adjacency.up_offsets[index - 1];
        adjacency.down_offsets[index] += adjacency.down_offsets[index - 1];
    }

    adjacency.up_targets.resize(adjacency.up_offsets.back());
    adjacency.down_targets.resize(adjacency.down_offsets.back());

    auto up_cursor = adjacency.up_offsets;
    auto down_cursor = adjacency.down_offsets;

    for (const auto source : util::irange<NodeID>(0, node_count))
    {
        for (const auto edge : graph.GetAdjacentEdgeRange(source))
        {
            const auto target = graph.GetTarget(edge);
            const auto &edge_data = graph.GetEdgeData(edge);
            if (!usesArc(edge_data, phast_data.orientation))
            {
                continue;
            }
            if (source == target)
            {
                continue;
            }

            bool is_upward = false;
            if (!classifyArc(phast_data, source, target, is_upward))
            {
                return false;
            }

            auto &cursor = is_upward ? up_cursor[source] : down_cursor[source];
            const auto limit =
                is_upward ? adjacency.up_offsets[source + 1] : adjacency.down_offsets[source + 1];
            auto &targets = is_upward ? adjacency.up_targets : adjacency.down_targets;
            if (cursor >= limit)
            {
                util::Log(logERROR) << "Derived adjacency buffer overflow for source node " << source;
                return false;
            }
            targets[cursor++] = target;
        }
    }

    for (const auto source : util::irange<NodeID>(0, node_count))
    {
        if (up_cursor[source] != adjacency.up_offsets[source + 1] ||
            down_cursor[source] != adjacency.down_offsets[source + 1])
        {
            util::Log(logERROR) << "Derived adjacency fill mismatch for source node " << source;
            return false;
        }
    }

    return true;
}

void printArrayPreview(const char *label, const std::vector<NodeID> &values)
{
    std::ostringstream out;
    out << label << " [";
    const auto preview = std::min<std::size_t>(values.size(), 5);
    for (const auto i : util::irange<std::size_t>(0, preview))
    {
        if (i > 0)
            out << ", ";
        out << values[i];
    }
    if (values.size() > preview)
        out << ", ...";
    out << "]";
    util::Log() << out.str();
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

    const auto &metric = metrics.at(metric_name);
    const auto &graph = metric.graph;
    util::Log() << "CH metric: " << metric_name;
    util::Log() << "CH nodes: " << graph.GetNumberOfNodes() << ", edges: " << graph.GetNumberOfEdges();
    util::Log() << "PHAST version: " << phast_data.version;
    util::Log() << "PHAST metric: " << phast_data.metric_name;
    util::Log() << "PHAST nodes: " << phast_data.node_count;
    util::Log() << "PHAST exclude index: " << phast_data.exclude_index;
    util::Log() << "PHAST orientation: " << orientationToString(phast_data.orientation);
    util::Log() << "PHAST checksum: " << phast_data.connectivity_checksum;
    util::Log() << "HSGR checksum: " << hsgr_checksum;
    util::Log() << "PHAST order size: " << phast_data.ordering.order.size();
    util::Log() << "PHAST rank size: " << phast_data.ordering.rank.size();
    util::Log() << "PHAST c2o size: " << phast_data.ordering.contraction_to_original.size();
    util::Log() << "PHAST o2c size: " << phast_data.ordering.original_to_contraction.size();
    printArrayPreview("PHAST order preview", phast_data.ordering.order);
    printArrayPreview("PHAST rank preview", phast_data.ordering.rank);
    printArrayPreview("PHAST c2o preview", phast_data.ordering.contraction_to_original);
    printArrayPreview("PHAST o2c preview", phast_data.ordering.original_to_contraction);

    if (!validateMetadata(metric_name, metric, hsgr_checksum, phast_data))
    {
        return EXIT_FAILURE;
    }
    if (!validateOrdering(phast_data))
    {
        return EXIT_FAILURE;
    }

    DerivedAdjacency adjacency;
    if (!deriveAdjacency(graph, phast_data, adjacency))
    {
        return EXIT_FAILURE;
    }

    util::Log() << "Derived upward arcs: " << adjacency.up_targets.size();
    util::Log() << "Derived downward arcs: " << adjacency.down_targets.size();
    util::Log() << "Skipped self-loop arcs: " << adjacency.skipped_self_loops;

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
