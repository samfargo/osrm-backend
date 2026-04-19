#ifndef OSRM_CONTRACTOR_PHAST_DATA_HPP
#define OSRM_CONTRACTOR_PHAST_DATA_HPP

#include <cstdint>
#include <string>

namespace osrm::contractor
{

struct PhastData
{
    std::uint32_t version = 0;
    std::uint32_t connectivity_checksum = 0;
    std::uint32_t node_count = 0;
    std::string metric_name;
};

} // namespace osrm::contractor

#endif
