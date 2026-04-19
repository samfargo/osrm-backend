#ifndef OSRM_CONTRACTOR_PHAST_DATA_HPP
#define OSRM_CONTRACTOR_PHAST_DATA_HPP

#include "util/typedefs.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace osrm::contractor
{

enum class PHASTOrientation : std::uint8_t
{
    Forward = 0,
    Reverse = 1
};

struct PHASTOrdering
{
    // sweep position -> original node id
    std::vector<NodeID> order;
    // original node id -> sweep position
    std::vector<NodeID> rank;
    // final contraction-space node id -> original node id
    std::vector<NodeID> contraction_to_original;
    // original node id -> final contraction-space node id
    std::vector<NodeID> original_to_contraction;
};

struct PhastData
{
    std::uint32_t version = 0;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t node_count = 0;
    std::uint32_t exclude_index = 0;
    std::string metric_name;
    PHASTOrientation orientation = PHASTOrientation::Forward;
    PHASTOrdering ordering;
};

} // namespace osrm::contractor

#endif
