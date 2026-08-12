#include "contractor/iso_adj.hpp"

#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/uuid/detail/sha1.hpp>

#include <algorithm>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <queue>
#include <system_error>

namespace osrm::contractor::phast
{
namespace
{

bool validOrientation(contractor::PHASTOrientation orientation)
{
    return orientation == contractor::PHASTOrientation::Forward ||
           orientation == contractor::PHASTOrientation::Reverse;
}

bool validMetricKind(MetricKind metric_kind)
{
    return metric_kind == MetricKind::Weight || metric_kind == MetricKind::Duration;
}

template <typename T> void hashBinary(boost::uuids::detail::sha1 &hasher, const T &value)
{
    std::array<unsigned char, sizeof(T)> bytes{};
    auto remaining = static_cast<std::uint64_t>(value);
    for (const auto index : util::irange<std::size_t>(0, bytes.size()))
    {
        bytes[index] = static_cast<unsigned char>(remaining & 0xffU);
        remaining >>= 8U;
    }
    hasher.process_bytes(bytes.data(), bytes.size());
}

std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES>
digestBytes(const boost::uuids::detail::sha1::digest_type &digest)
{
    std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES> bytes{};
    static_assert(sizeof(digest) == bytes.size());
    std::memcpy(bytes.data(), digest, bytes.size());
    return bytes;
}

bool validateMetadata(const IsoAdjMetadata &metadata)
{
    if (metadata.phast_schema_version < PHAST_SCHEMA_VERSION)
    {
        util::Log(logERROR) << "iso-adj PHAST schema version " << metadata.phast_schema_version
                            << " is older than supported version " << PHAST_SCHEMA_VERSION << ".";
        return false;
    }
    if (!validMetricKind(metadata.metric_kind) || !validOrientation(metadata.orientation))
    {
        util::Log(logERROR) << "iso-adj has an invalid metric kind or orientation.";
        return false;
    }
    if (metadata.metric_name.empty() || metadata.metric_name.size() >= ISO_ADJ_METRIC_NAME_BYTES)
    {
        util::Log(logERROR) << "iso-adj metric name is empty or too long.";
        return false;
    }
    if (metadata.node_count == 0 || metadata.node_count >= SPECIAL_NODEID)
    {
        util::Log(logERROR) << "iso-adj node count is invalid: " << metadata.node_count;
        return false;
    }
    if (std::all_of(metadata.build_id.begin(), metadata.build_id.end(), [](unsigned char value) {
            return value == 0;
        }))
    {
        util::Log(logERROR) << "iso-adj build identity is empty.";
        return false;
    }
    return true;
}

} // namespace

std::array<unsigned char, ISO_ADJ_BUILD_ID_BYTES>
BuildIsoAdjBuildID(const std::string &dataset_timestamp, const IsoAdjMetadata &metadata)
{
    boost::uuids::detail::sha1 hasher;
    hasher.process_bytes(dataset_timestamp.data(), dataset_timestamp.size());
    const unsigned char separator = 0;
    hasher.process_bytes(&separator, sizeof(separator));
    hasher.process_bytes(metadata.metric_name.data(), metadata.metric_name.size());
    hasher.process_bytes(&separator, sizeof(separator));
    hashBinary(hasher, metadata.phast_schema_version);
    hashBinary(hasher, static_cast<std::uint32_t>(metadata.metric_kind));
    hashBinary(hasher, static_cast<std::uint32_t>(metadata.orientation));
    hashBinary(hasher, metadata.exclude_index);
    hashBinary(hasher, metadata.connectivity_checksum);
    hashBinary(hasher, metadata.node_count);
    hashBinary(hasher, metadata.edge_count);

    boost::uuids::detail::sha1::digest_type digest{};
    hasher.get_digest(digest);
    return digestBytes(digest);
}

bool WriteIsoAdj(const std::filesystem::path &path,
                 const IsoAdjMetadata &metadata,
                 const std::vector<std::uint64_t> &offsets,
                 const std::vector<std::uint32_t> &targets,
                 const std::vector<std::uint32_t> &costs)
{
    if (!validateMetadata(metadata) || offsets.size() != metadata.node_count + 1 ||
        targets.size() != metadata.edge_count || costs.size() != metadata.edge_count ||
        offsets.front() != 0 || offsets.back() != metadata.edge_count)
    {
        util::Log(logERROR) << "Refusing to write inconsistent iso-adj data.";
        return false;
    }

    const auto temporary_path = std::filesystem::path{path.string() + ".tmp"};
    std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        util::Log(logERROR) << "Could not open temporary iso-adj output: "
                            << temporary_path.string();
        return false;
    }

    IsoAdjFileHeader header;
    header.header_bytes = sizeof(header);
    header.phast_schema_version = metadata.phast_schema_version;
    header.metric_kind = static_cast<std::uint32_t>(metadata.metric_kind);
    header.orientation = static_cast<std::uint32_t>(metadata.orientation);
    header.exclude_index = metadata.exclude_index;
    header.connectivity_checksum = metadata.connectivity_checksum;
    header.node_count = metadata.node_count;
    header.edge_count = metadata.edge_count;
    header.build_id = metadata.build_id;
    header.metric_name_size = static_cast<std::uint32_t>(metadata.metric_name.size());
    std::copy(metadata.metric_name.begin(), metadata.metric_name.end(), header.metric_name.begin());

    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    output.write(reinterpret_cast<const char *>(offsets.data()),
                 static_cast<std::streamsize>(offsets.size() * sizeof(std::uint64_t)));
    output.write(reinterpret_cast<const char *>(targets.data()),
                 static_cast<std::streamsize>(targets.size() * sizeof(std::uint32_t)));
    output.write(reinterpret_cast<const char *>(costs.data()),
                 static_cast<std::streamsize>(costs.size() * sizeof(std::uint32_t)));
    output.close();
    if (!output)
    {
        util::Log(logERROR) << "Failed writing iso-adj output: " << temporary_path.string();
        std::error_code ignored;
        std::filesystem::remove(temporary_path, ignored);
        return false;
    }

    std::error_code rename_error;
    std::filesystem::rename(temporary_path, path, rename_error);
    if (rename_error)
    {
        util::Log(logERROR) << "Failed publishing iso-adj output " << path.string() << ": "
                            << rename_error.message();
        std::error_code ignored;
        std::filesystem::remove(temporary_path, ignored);
        return false;
    }
    return true;
}

bool LoadIsoAdj(const std::filesystem::path &path, IsoAdj &adjacency)
{
    adjacency.file.close();
    adjacency.metadata = {};
    adjacency.offsets = nullptr;
    adjacency.targets = nullptr;
    adjacency.costs = nullptr;

    try
    {
        adjacency.file.open(path.string());
    }
    catch (const std::exception &error)
    {
        util::Log(logERROR) << "Failed to mmap iso-adj file " << path.string() << ": "
                            << error.what();
        return false;
    }
    if (!adjacency.file.is_open())
    {
        util::Log(logERROR) << "Could not open iso-adj file: " << path.string();
        return false;
    }

    const char *data = adjacency.file.data();
    const auto size = adjacency.file.size();
    if (size < ISO_ADJ_HEADER_BYTES)
    {
        util::Log(logERROR) << "iso-adj file has bad magic or header size: " << path.string();
        return false;
    }

    IsoAdjFileHeader header;
    std::memcpy(&header, data, sizeof(header));
    if (header.magic != ISO_ADJ_MAGIC || header.format_version != ISO_ADJ_VERSION ||
        header.header_bytes != ISO_ADJ_HEADER_BYTES || header.reserved != 0)
    {
        util::Log(logERROR) << "iso-adj header version or size mismatch: got version "
                            << header.format_version << " and " << header.header_bytes
                            << " bytes; expected version "
                            << ISO_ADJ_VERSION << " and " << ISO_ADJ_HEADER_BYTES << " bytes.";
        return false;
    }

    adjacency.metadata.phast_schema_version = header.phast_schema_version;
    adjacency.metadata.metric_kind = static_cast<MetricKind>(header.metric_kind);
    adjacency.metadata.orientation = static_cast<contractor::PHASTOrientation>(header.orientation);
    adjacency.metadata.exclude_index = header.exclude_index;
    adjacency.metadata.connectivity_checksum = header.connectivity_checksum;
    adjacency.metadata.node_count = header.node_count;
    adjacency.metadata.edge_count = header.edge_count;
    adjacency.metadata.build_id = header.build_id;
    if (header.metric_name_size == 0 || header.metric_name_size >= ISO_ADJ_METRIC_NAME_BYTES)
    {
        util::Log(logERROR) << "iso-adj metric name is invalid: " << path.string();
        return false;
    }
    adjacency.metadata.metric_name.assign(header.metric_name.data(), header.metric_name_size);
    if (!validateMetadata(adjacency.metadata))
    {
        return false;
    }

    const auto node_count = adjacency.metadata.node_count;
    const auto edge_count = adjacency.metadata.edge_count;
    std::size_t offset = ISO_ADJ_HEADER_BYTES;
    const auto maximum_size = std::numeric_limits<std::size_t>::max();
    if (node_count + 1 > (maximum_size - offset) / sizeof(std::uint64_t))
    {
        util::Log(logERROR) << "iso-adj node count overflows its file size.";
        return false;
    }
    const auto offsets_bytes = static_cast<std::size_t>(node_count + 1) * sizeof(std::uint64_t);
    offset += offsets_bytes;
    if (edge_count > (maximum_size - offset) / (2 * sizeof(std::uint32_t)))
    {
        util::Log(logERROR) << "iso-adj edge count overflows its file size.";
        return false;
    }
    const auto edge_bytes = static_cast<std::size_t>(edge_count) * sizeof(std::uint32_t);
    const auto expected_size = offset + 2 * edge_bytes;
    if (size != expected_size)
    {
        util::Log(logERROR) << "iso-adj file size mismatch: have " << size << " expected "
                            << expected_size << ".";
        return false;
    }

    offset = ISO_ADJ_HEADER_BYTES;
    adjacency.offsets = reinterpret_cast<const std::uint64_t *>(data + offset);
    offset += offsets_bytes;
    adjacency.targets = reinterpret_cast<const std::uint32_t *>(data + offset);
    offset += edge_bytes;
    adjacency.costs = reinterpret_cast<const std::uint32_t *>(data + offset);
    if (adjacency.offsets[0] != 0 || adjacency.offsets[node_count] != edge_count)
    {
        util::Log(logERROR) << "iso-adj offset bounds do not match the edge count.";
        return false;
    }
    for (std::uint64_t node = 0; node < node_count; ++node)
    {
        if (adjacency.offsets[node] > adjacency.offsets[node + 1] ||
            adjacency.offsets[node + 1] > edge_count)
        {
            util::Log(logERROR) << "iso-adj offsets are not monotonic at node " << node << ".";
            return false;
        }
    }
    for (std::uint64_t edge = 0; edge < edge_count; ++edge)
    {
        if (adjacency.targets[edge] >= node_count)
        {
            util::Log(logERROR) << "iso-adj target is out of range at edge " << edge << ".";
            return false;
        }
    }
    return true;
}

void RunCappedDijkstraIsoAdj(const IsoAdj &adjacency,
                             const std::vector<PHASTSeed> &seeds,
                             const std::optional<EdgeWeight> &cap_metric,
                             std::vector<EdgeWeight> &distances)
{
    distances.assign(static_cast<std::size_t>(adjacency.metadata.node_count), INVALID_EDGE_WEIGHT);
    const auto cap_ticks = cap_metric.has_value()
                               ? from_alias<std::int64_t>(*cap_metric)
                               : std::numeric_limits<std::int64_t>::max();
    using QueueNode = std::pair<std::int64_t, std::uint32_t>;
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;

    for (const auto &seed : seeds)
    {
        if (seed.node >= adjacency.metadata.node_count || seed.cost == INVALID_EDGE_WEIGHT)
        {
            continue;
        }
        const auto seed_ticks = from_alias<std::int64_t>(seed.cost);
        if (seed_ticks <= cap_ticks &&
            (distances[seed.node] == INVALID_EDGE_WEIGHT ||
             seed_ticks < from_alias<std::int64_t>(distances[seed.node])))
        {
            distances[seed.node] = seed.cost;
            queue.push({seed_ticks, seed.node});
        }
    }

    while (!queue.empty())
    {
        const auto [distance, node] = queue.top();
        queue.pop();
        if (distance != from_alias<std::int64_t>(distances[node]))
        {
            continue;
        }
        const auto begin = adjacency.offsets[node];
        const auto end = adjacency.offsets[node + 1];
        for (auto edge = begin; edge < end; ++edge)
        {
            const auto target = adjacency.targets[edge];
            const auto candidate = distance + static_cast<std::int64_t>(adjacency.costs[edge]);
            if (candidate <= cap_ticks &&
                (distances[target] == INVALID_EDGE_WEIGHT ||
                 candidate < from_alias<std::int64_t>(distances[target])))
            {
                distances[target] = to_alias<EdgeWeight>(candidate);
                queue.push({candidate, target});
            }
        }
    }
}

} // namespace osrm::contractor::phast
