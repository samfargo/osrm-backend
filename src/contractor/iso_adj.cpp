#include "contractor/iso_adj.hpp"

#include "util/integer_range.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/uuid/detail/sha1.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <queue>
#include <system_error>
#include <unordered_map>

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
    if (metadata.node_order != ISO_ADJ_NODE_ORDER_MORTON)
    {
        util::Log(logERROR) << "iso-adj has an unsupported node order.";
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
    if (metadata.resolution == 0 || metadata.resolution > 15 || metadata.ordinal_count == 0 ||
        metadata.ordinal_count > std::numeric_limits<std::uint32_t>::max() ||
        metadata.ordinals_file_size != metadata.ordinal_count * sizeof(std::uint64_t) ||
        metadata.candidate_count == 0 || metadata.artifact_version.empty() ||
        metadata.artifact_version.size() >= ISO_ADJ_ARTIFACT_VERSION_BYTES)
    {
        util::Log(logERROR) << "iso-adj reverse cell index metadata is invalid.";
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
    hashBinary(hasher, metadata.node_order);
    hashBinary(hasher, metadata.node_count);
    hashBinary(hasher, metadata.edge_count);
    hashBinary(hasher, metadata.candidate_count);
    hashBinary(hasher, metadata.resolution);
    hashBinary(hasher, metadata.ordinal_count);
    hashBinary(hasher, metadata.ordinals_file_size);
    hashBinary(hasher, metadata.ordinals_hash_hi);
    hashBinary(hasher, metadata.ordinals_hash_lo);
    hasher.process_bytes(metadata.artifact_version.data(), metadata.artifact_version.size());

    boost::uuids::detail::sha1::digest_type digest{};
    hasher.get_digest(digest);
    return digestBytes(digest);
}

bool WriteIsoAdj(const std::filesystem::path &path,
                 const IsoAdjMetadata &metadata,
                 const std::vector<std::uint64_t> &offsets,
                 const std::vector<std::uint32_t> &targets,
                 const std::vector<std::uint32_t> &costs,
                 const std::vector<std::uint64_t> &cell_offsets,
                 const std::vector<std::uint32_t> &candidate_ordinals,
                 const std::vector<std::int32_t> &candidate_costs,
                 const std::vector<std::uint32_t> &original_to_spatial)
{
    if (!validateMetadata(metadata) || offsets.size() != metadata.node_count + 1 ||
        targets.size() != metadata.edge_count || costs.size() != metadata.edge_count ||
        offsets.front() != 0 || offsets.back() != metadata.edge_count ||
        cell_offsets.size() != metadata.node_count + 1 || cell_offsets.front() != 0 ||
        cell_offsets.back() != metadata.candidate_count ||
        candidate_ordinals.size() != metadata.candidate_count ||
        candidate_costs.size() != metadata.candidate_count ||
        original_to_spatial.size() != metadata.node_count ||
        std::any_of(candidate_ordinals.begin(), candidate_ordinals.end(), [&](const auto ordinal) {
            return ordinal >= metadata.ordinal_count;
        }) ||
        std::any_of(original_to_spatial.begin(), original_to_spatial.end(), [&](const auto node) {
            return node >= metadata.node_count;
        }))
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
    header.node_order = metadata.node_order;
    header.resolution = metadata.resolution;
    header.node_count = metadata.node_count;
    header.edge_count = metadata.edge_count;
    header.candidate_count = metadata.candidate_count;
    header.build_id = metadata.build_id;
    header.metric_name_size = static_cast<std::uint32_t>(metadata.metric_name.size());
    std::copy(metadata.metric_name.begin(), metadata.metric_name.end(), header.metric_name.begin());
    header.artifact_version_size = static_cast<std::uint32_t>(metadata.artifact_version.size());
    std::copy(metadata.artifact_version.begin(),
              metadata.artifact_version.end(),
              header.artifact_version.begin());
    header.ordinal_count = metadata.ordinal_count;
    header.ordinals_file_size = metadata.ordinals_file_size;
    header.ordinals_hash_hi = metadata.ordinals_hash_hi;
    header.ordinals_hash_lo = metadata.ordinals_hash_lo;

    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    output.write(reinterpret_cast<const char *>(offsets.data()),
                 static_cast<std::streamsize>(offsets.size() * sizeof(std::uint64_t)));
    output.write(reinterpret_cast<const char *>(targets.data()),
                 static_cast<std::streamsize>(targets.size() * sizeof(std::uint32_t)));
    output.write(reinterpret_cast<const char *>(costs.data()),
                 static_cast<std::streamsize>(costs.size() * sizeof(std::uint32_t)));
    output.write(reinterpret_cast<const char *>(cell_offsets.data()),
                 static_cast<std::streamsize>(cell_offsets.size() * sizeof(std::uint64_t)));
    output.write(reinterpret_cast<const char *>(candidate_ordinals.data()),
                 static_cast<std::streamsize>(candidate_ordinals.size() * sizeof(std::uint32_t)));
    output.write(reinterpret_cast<const char *>(candidate_costs.data()),
                 static_cast<std::streamsize>(candidate_costs.size() * sizeof(std::int32_t)));
    output.write(reinterpret_cast<const char *>(original_to_spatial.data()),
                 static_cast<std::streamsize>(original_to_spatial.size() * sizeof(std::uint32_t)));
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
    adjacency.cell_offsets = nullptr;
    adjacency.candidate_ordinals = nullptr;
    adjacency.candidate_costs = nullptr;
    adjacency.original_to_spatial = nullptr;

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
        header.header_bytes != ISO_ADJ_HEADER_BYTES ||
        header.node_order != ISO_ADJ_NODE_ORDER_MORTON)
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
    adjacency.metadata.node_order = header.node_order;
    adjacency.metadata.resolution = header.resolution;
    adjacency.metadata.node_count = header.node_count;
    adjacency.metadata.edge_count = header.edge_count;
    adjacency.metadata.candidate_count = header.candidate_count;
    adjacency.metadata.build_id = header.build_id;
    if (header.metric_name_size == 0 || header.metric_name_size >= ISO_ADJ_METRIC_NAME_BYTES)
    {
        util::Log(logERROR) << "iso-adj metric name is invalid: " << path.string();
        return false;
    }
    adjacency.metadata.metric_name.assign(header.metric_name.data(), header.metric_name_size);
    if (header.artifact_version_size == 0 ||
        header.artifact_version_size >= ISO_ADJ_ARTIFACT_VERSION_BYTES)
    {
        util::Log(logERROR) << "iso-adj artifact version is invalid: " << path.string();
        return false;
    }
    adjacency.metadata.artifact_version.assign(header.artifact_version.data(),
                                                header.artifact_version_size);
    adjacency.metadata.ordinal_count = header.ordinal_count;
    adjacency.metadata.ordinals_file_size = header.ordinals_file_size;
    adjacency.metadata.ordinals_hash_hi = header.ordinals_hash_hi;
    adjacency.metadata.ordinals_hash_lo = header.ordinals_hash_lo;
    if (!validateMetadata(adjacency.metadata))
    {
        return false;
    }

    const auto node_count = adjacency.metadata.node_count;
    const auto edge_count = adjacency.metadata.edge_count;
    const auto candidate_count = adjacency.metadata.candidate_count;
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
    offset += 2 * edge_bytes;
    if (node_count + 1 > (maximum_size - offset) / sizeof(std::uint64_t))
    {
        util::Log(logERROR) << "iso-adj cell offset count overflows its file size.";
        return false;
    }
    const auto cell_offsets_bytes =
        static_cast<std::size_t>(node_count + 1) * sizeof(std::uint64_t);
    offset += cell_offsets_bytes;
    if (candidate_count > (maximum_size - offset) / (2 * sizeof(std::uint32_t)))
    {
        util::Log(logERROR) << "iso-adj candidate count overflows its file size.";
        return false;
    }
    const auto candidate_bytes = static_cast<std::size_t>(candidate_count) * sizeof(std::uint32_t);
    offset += 2 * candidate_bytes;
    if (node_count > (maximum_size - offset) / sizeof(std::uint32_t))
    {
        util::Log(logERROR) << "iso-adj node map count overflows its file size.";
        return false;
    }
    const auto node_map_bytes = static_cast<std::size_t>(node_count) * sizeof(std::uint32_t);
    const auto expected_size = offset + node_map_bytes;
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
    offset += edge_bytes;
    adjacency.cell_offsets = reinterpret_cast<const std::uint64_t *>(data + offset);
    offset += cell_offsets_bytes;
    adjacency.candidate_ordinals = reinterpret_cast<const std::uint32_t *>(data + offset);
    offset += candidate_bytes;
    adjacency.candidate_costs = reinterpret_cast<const std::int32_t *>(data + offset);
    offset += candidate_bytes;
    adjacency.original_to_spatial = reinterpret_cast<const std::uint32_t *>(data + offset);
    if (adjacency.offsets[0] != 0 || adjacency.offsets[node_count] != edge_count)
    {
        util::Log(logERROR) << "iso-adj offset bounds do not match the edge count.";
        return false;
    }
    if (adjacency.cell_offsets[0] != 0 ||
        adjacency.cell_offsets[node_count] != candidate_count)
    {
        util::Log(logERROR) << "iso-adj cell offset bounds do not match the candidate count.";
        return false;
    }
    return true;
}

bool RunCappedDijkstraIsoAdj(const IsoAdj &adjacency,
                            const std::vector<PHASTSeed> &seeds,
                            const std::optional<EdgeWeight> &cap_metric,
                            std::vector<ReachedState> &reached)
{
    reached.clear();
    const auto cap_ticks = cap_metric.has_value()
                               ? from_alias<std::int64_t>(*cap_metric)
                               : std::numeric_limits<std::int64_t>::max();
    using QueueNode = std::pair<std::int64_t, std::uint32_t>;
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> queue;
    std::unordered_map<std::uint32_t, std::int64_t> distances;
    distances.reserve(seeds.size() * 4);

    for (const auto &seed : seeds)
    {
        if (seed.node >= adjacency.metadata.node_count || seed.cost == INVALID_EDGE_WEIGHT)
        {
            continue;
        }
        const auto seed_ticks = from_alias<std::int64_t>(seed.cost);
        if (seed_ticks <= cap_ticks &&
            (!distances.contains(seed.node) || seed_ticks < distances[seed.node]))
        {
            distances[seed.node] = seed_ticks;
            queue.push({seed_ticks, seed.node});
        }
    }

    while (!queue.empty())
    {
        const auto [distance, node] = queue.top();
        queue.pop();
        const auto current = distances.find(node);
        if (current == distances.end() || distance != current->second)
        {
            continue;
        }
        const auto begin = adjacency.offsets[node];
        const auto end = adjacency.offsets[node + 1];
        if (begin > end || end > adjacency.metadata.edge_count)
        {
            util::Log(logERROR) << "iso-adj edge offsets are invalid at reached node " << node
                                << ".";
            return false;
        }
        for (auto edge = begin; edge < end; ++edge)
        {
            const auto target = adjacency.targets[edge];
            if (target >= adjacency.metadata.node_count)
            {
                util::Log(logERROR) << "iso-adj target is invalid at reached edge " << edge << ".";
                return false;
            }
            const auto candidate = distance + static_cast<std::int64_t>(adjacency.costs[edge]);
            if (candidate <= cap_ticks &&
                (!distances.contains(target) || candidate < distances[target]))
            {
                distances[target] = candidate;
                queue.push({candidate, target});
            }
        }
    }

    reached.reserve(distances.size());
    for (const auto &[node, cost] : distances)
    {
        reached.push_back({node, to_alias<EdgeWeight>(cost)});
    }
    std::sort(reached.begin(), reached.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.node < rhs.node;
    });
    return true;
}

bool ReduceReachedCells(const IsoAdj &adjacency,
                        const std::vector<ReachedState> &reached,
                        const std::optional<EdgeWeight> &cap_metric,
                        std::vector<SparseResultRecord> &records)
{
    records.clear();
    if (adjacency.metadata.metric_kind != MetricKind::Duration)
    {
        util::Log(logERROR) << "Sparse custom results require the duration metric.";
        return false;
    }
    const auto cap_ticks = cap_metric.has_value()
                               ? from_alias<std::int64_t>(*cap_metric)
                               : std::numeric_limits<std::int64_t>::max();
    std::unordered_map<std::uint32_t, std::int64_t> minimum_ticks;
    minimum_ticks.reserve(reached.size());
    for (const auto &state : reached)
    {
        if (state.node >= adjacency.metadata.node_count || state.cost == INVALID_EDGE_WEIGHT)
        {
            util::Log(logERROR) << "Reached state is invalid: " << state.node;
            return false;
        }
        const auto state_ticks = from_alias<std::int64_t>(state.cost);
        const auto begin = adjacency.cell_offsets[state.node];
        const auto end = adjacency.cell_offsets[state.node + 1];
        if (begin > end || end > adjacency.metadata.candidate_count)
        {
            util::Log(logERROR) << "iso-adj cell offsets are invalid at reached node "
                                << state.node << ".";
            return false;
        }
        for (auto index = begin; index < end; ++index)
        {
            const auto total =
                state_ticks + static_cast<std::int64_t>(adjacency.candidate_costs[index]);
            if (total < 0 || total > cap_ticks)
            {
                continue;
            }
            const auto ordinal = adjacency.candidate_ordinals[index];
            if (ordinal >= adjacency.metadata.ordinal_count)
            {
                util::Log(logERROR) << "iso-adj candidate ordinal is invalid at reached record "
                                    << index << ".";
                return false;
            }
            const auto found = minimum_ticks.find(ordinal);
            if (found == minimum_ticks.end() || total < found->second)
            {
                minimum_ticks[ordinal] = total;
            }
        }
    }

    records.reserve(minimum_ticks.size());
    for (const auto &[ordinal, ticks] : minimum_ticks)
    {
        const auto rounded = std::nearbyint(static_cast<double>(ticks) / 10.0);
        if (!std::isfinite(rounded) || rounded < 0.0)
        {
            continue;
        }
        const auto clipped = std::min<double>(rounded, 65534.0);
        records.push_back({ordinal, static_cast<std::uint16_t>(clipped), 0});
    }
    std::sort(records.begin(), records.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.ordinal < rhs.ordinal;
    });
    return true;
}

bool WriteSparseResult(const std::filesystem::path &path,
                       const IsoAdjMetadata &metadata,
                       const std::uint32_t cap_seconds,
                       const std::vector<SparseResultRecord> &records)
{
    if (!validateMetadata(metadata) ||
        std::adjacent_find(records.begin(), records.end(), [](const auto &lhs, const auto &rhs) {
            return lhs.ordinal >= rhs.ordinal;
        }) != records.end() ||
        std::any_of(records.begin(), records.end(), [&](const auto &record) {
            return record.ordinal >= metadata.ordinal_count || record.reserved != 0 ||
                   record.seconds == std::numeric_limits<std::uint16_t>::max() ||
                   record.seconds > cap_seconds;
        }))
    {
        util::Log(logERROR) << "Refusing to write an invalid sparse custom result.";
        return false;
    }

    SparseResultFileHeader header;
    header.header_bytes = sizeof(header);
    header.phast_schema_version = metadata.phast_schema_version;
    header.metric_kind = static_cast<std::uint32_t>(metadata.metric_kind);
    header.orientation = static_cast<std::uint32_t>(metadata.orientation);
    header.exclude_index = metadata.exclude_index;
    header.connectivity_checksum = metadata.connectivity_checksum;
    header.resolution = metadata.resolution;
    header.cap_seconds = cap_seconds;
    header.node_count = metadata.node_count;
    header.edge_count = metadata.edge_count;
    header.build_id = metadata.build_id;
    header.ordinal_count = metadata.ordinal_count;
    header.ordinals_file_size = metadata.ordinals_file_size;
    header.ordinals_hash_hi = metadata.ordinals_hash_hi;
    header.ordinals_hash_lo = metadata.ordinals_hash_lo;
    std::copy(metadata.artifact_version.begin(),
              metadata.artifact_version.end(),
              header.artifact_version.begin());
    header.record_count = records.size();

    const auto temporary = std::filesystem::path{path.string() + ".tmp"};
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char *>(&header), sizeof(header));
    output.write(reinterpret_cast<const char *>(records.data()),
                 static_cast<std::streamsize>(records.size() * sizeof(SparseResultRecord)));
    output.close();
    if (!output)
    {
        util::Log(logERROR) << "Failed writing sparse custom result: " << temporary.string();
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }
    std::error_code rename_error;
    std::filesystem::rename(temporary, path, rename_error);
    if (rename_error)
    {
        util::Log(logERROR) << "Failed publishing sparse custom result " << path.string() << ": "
                            << rename_error.message();
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
    }
    return true;
}

} // namespace osrm::contractor::phast
