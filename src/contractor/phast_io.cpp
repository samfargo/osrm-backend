#include "contractor/phast_io.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

#include "util/integer_range.hpp"
#include "util/log.hpp"

#include <cstdint>
#include <fstream>
#include <limits>

namespace osrm::contractor::phast
{

namespace
{

constexpr std::uint32_t PHASTFIELD_SCHEMA_VERSION = 1;
constexpr std::uint32_t PHASTFIELD_NO_CAP = std::numeric_limits<std::uint32_t>::max();

bool ShouldUseU16Encoding(const std::vector<EdgeWeight> &distances,
                          const std::optional<EdgeWeight> &cap_metric)
{
    const auto u16_sentinel = static_cast<std::int64_t>(std::numeric_limits<std::uint16_t>::max());
    if (cap_metric.has_value() && from_alias<std::int64_t>(cap_metric.value()) >= u16_sentinel)
    {
        return false;
    }
    for (const auto distance : distances)
    {
        if (distance == INVALID_EDGE_WEIGHT)
        {
            continue;
        }
        if (from_alias<std::int64_t>(distance) >= u16_sentinel)
        {
            return false;
        }
    }
    return true;
}

bool EncodeDistancesToU16(const std::vector<EdgeWeight> &distances, std::vector<std::uint16_t> &encoded)
{
    constexpr auto sentinel = std::numeric_limits<std::uint16_t>::max();
    encoded.resize(distances.size());
    for (const auto i : util::irange<std::size_t>(0, distances.size()))
    {
        const auto distance = distances[i];
        if (distance == INVALID_EDGE_WEIGHT)
        {
            encoded[i] = sentinel;
            continue;
        }

        const auto value = from_alias<std::int64_t>(distance);
        if (value < 0 || value >= static_cast<std::int64_t>(sentinel))
        {
            util::Log(logERROR) << "Distance value " << value << " cannot be encoded as uint16.";
            return false;
        }
        encoded[i] = static_cast<std::uint16_t>(value);
    }
    return true;
}

bool EncodeDistancesToU32(const std::vector<EdgeWeight> &distances, std::vector<std::uint32_t> &encoded)
{
    constexpr auto sentinel = std::numeric_limits<std::uint32_t>::max();
    encoded.resize(distances.size());
    for (const auto i : util::irange<std::size_t>(0, distances.size()))
    {
        const auto distance = distances[i];
        if (distance == INVALID_EDGE_WEIGHT)
        {
            encoded[i] = sentinel;
            continue;
        }

        const auto value = from_alias<std::int64_t>(distance);
        if (value < 0 || value >= static_cast<std::int64_t>(sentinel))
        {
            util::Log(logERROR) << "Distance value " << value << " cannot be encoded as uint32.";
            return false;
        }
        encoded[i] = static_cast<std::uint32_t>(value);
    }
    return true;
}

bool WritePhastFieldV1(const std::filesystem::path &path,
                       const std::vector<EdgeWeight> &distances,
                       const std::optional<EdgeWeight> &cap_metric,
                       const OutputMetadata &metadata)
{
    try
    {
        storage::tar::FileWriter writer(path, storage::tar::FileWriter::GenerateFingerprint);
        const auto metric_kind_value = static_cast<std::uint32_t>(metadata.metric_kind);
        const auto orientation_value = static_cast<std::uint32_t>(metadata.orientation);
        const auto cap_value =
            cap_metric.has_value() ? static_cast<std::uint32_t>(from_alias<std::int64_t>(cap_metric.value()))
                                   : PHASTFIELD_NO_CAP;
        const auto node_count = static_cast<std::uint32_t>(distances.size());
        const auto exclude_value = static_cast<std::uint32_t>(metadata.exclude_index);
        const auto poi_count_value = static_cast<std::uint32_t>(metadata.poi_count);

        writer.WriteElementCount64("/phast_result/meta/schema_version", 1);
        writer.WriteFrom("/phast_result/meta/schema_version", PHASTFIELD_SCHEMA_VERSION);
        writer.WriteElementCount64("/phast_result/meta/connectivity_checksum", 1);
        writer.WriteFrom("/phast_result/meta/connectivity_checksum", metadata.connectivity_checksum);
        storage::serialization::write(writer, "/phast_result/meta/metric_name", metadata.metric_name);
        writer.WriteElementCount64("/phast_result/meta/metric_kind", 1);
        writer.WriteFrom("/phast_result/meta/metric_kind", metric_kind_value);
        writer.WriteElementCount64("/phast_result/meta/orientation", 1);
        writer.WriteFrom("/phast_result/meta/orientation", orientation_value);
        writer.WriteElementCount64("/phast_result/meta/exclude_index", 1);
        writer.WriteFrom("/phast_result/meta/exclude_index", exclude_value);
        writer.WriteElementCount64("/phast_result/meta/cap_metric", 1);
        writer.WriteFrom("/phast_result/meta/cap_metric", cap_value);
        writer.WriteElementCount64("/phast_result/meta/node_count", 1);
        writer.WriteFrom("/phast_result/meta/node_count", node_count);
        writer.WriteElementCount64("/phast_result/meta/poi_count", 1);
        writer.WriteFrom("/phast_result/meta/poi_count", poi_count_value);

        if (ShouldUseU16Encoding(distances, cap_metric))
        {
            std::vector<std::uint16_t> costs_u16;
            if (!EncodeDistancesToU16(distances, costs_u16))
            {
                return false;
            }
            storage::serialization::write(writer, "/phast_result/costs_u16", costs_u16);
        }
        else
        {
            std::vector<std::uint32_t> costs_u32;
            if (!EncodeDistancesToU32(distances, costs_u32))
            {
                return false;
            }
            storage::serialization::write(writer, "/phast_result/costs_u32", costs_u32);
        }
    }
    catch (const std::exception &e)
    {
        util::Log(logERROR) << "Failed to write phastfield output: " << e.what();
        return false;
    }

    return true;
}

bool WriteRawU32(const std::filesystem::path &path, const std::vector<EdgeWeight> &distances)
{
    std::vector<std::uint32_t> encoded;
    if (!EncodeDistancesToU32(distances, encoded))
    {
        return false;
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        util::Log(logERROR) << "Could not open output file for writing: " << path.string();
        return false;
    }

    output.write(reinterpret_cast<const char *>(encoded.data()),
                 static_cast<std::streamsize>(encoded.size() * sizeof(std::uint32_t)));
    if (!output)
    {
        util::Log(logERROR) << "Failed to write output file: " << path.string();
        return false;
    }

    return true;
}

} // namespace

bool ParseOutputFormatKind(const std::string &output_format, OutputFormatKind &output_format_kind)
{
    if (output_format == "phastfield-v1")
    {
        output_format_kind = OutputFormatKind::PhastFieldV1;
        return true;
    }
    if (output_format == "raw-u32")
    {
        output_format_kind = OutputFormatKind::RawU32;
        return true;
    }
    return false;
}

std::filesystem::path DefaultOutputPath(const std::filesystem::path &base_path,
                                        OutputFormatKind output_format_kind)
{
    if (output_format_kind == OutputFormatKind::RawU32)
    {
        return std::filesystem::path(base_path.string() + ".phastfield.raw_u32");
    }
    return std::filesystem::path(base_path.string() + ".phastfield");
}

bool WriteOutput(const std::filesystem::path &path,
                 OutputFormatKind output_format_kind,
                 const std::vector<EdgeWeight> &distances,
                 const std::optional<EdgeWeight> &cap_metric,
                 const OutputMetadata &metadata)
{
    if (output_format_kind == OutputFormatKind::PhastFieldV1)
    {
        return WritePhastFieldV1(path, distances, cap_metric, metadata);
    }

    return WriteRawU32(path, distances);
}

} // namespace osrm::contractor::phast
