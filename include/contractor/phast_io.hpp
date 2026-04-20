#ifndef OSRM_CONTRACTOR_PHAST_IO_HPP
#define OSRM_CONTRACTOR_PHAST_IO_HPP

#include "contractor/phast_common.hpp"
#include "contractor/phast_data.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace osrm::contractor::phast
{

struct OutputMetadata
{
    std::string metric_name;
    MetricKind metric_kind = MetricKind::Weight;
    contractor::PHASTOrientation orientation = contractor::PHASTOrientation::Forward;
    std::size_t exclude_index = 0;
    std::uint32_t connectivity_checksum = 0;
    std::size_t poi_count = 0;
};

bool ParseOutputFormatKind(const std::string &output_format, OutputFormatKind &output_format_kind);

std::filesystem::path DefaultOutputPath(const std::filesystem::path &base_path,
                                        OutputFormatKind output_format_kind);

bool WriteOutput(const std::filesystem::path &path,
                 OutputFormatKind output_format_kind,
                 const std::vector<EdgeWeight> &distances,
                 const std::optional<EdgeWeight> &cap_metric,
                 const OutputMetadata &metadata);

} // namespace osrm::contractor::phast

#endif
