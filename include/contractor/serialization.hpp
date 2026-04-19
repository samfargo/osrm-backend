#ifndef OSRM_CONTRACTOR_SERIALIZATION_HPP
#define OSRM_CONTRACTOR_SERIALIZATION_HPP

#include "contractor/contracted_metric.hpp"
#include "contractor/phast_data.hpp"

#include "util/serialization.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

namespace osrm::contractor::serialization
{

template <storage::Ownership Ownership>
void write(storage::tar::FileWriter &writer,
           const std::string &name,
           const detail::ContractedMetric<Ownership> &metric)
{
    util::serialization::write(writer, name + "/contracted_graph", metric.graph);

    writer.WriteElementCount64(name + "/exclude", metric.edge_filter.size());
    for (const auto index : util::irange<std::size_t>(0, metric.edge_filter.size()))
    {
        storage::serialization::write(writer,
                                      name + "/exclude/" + std::to_string(index) + "/edge_filter",
                                      metric.edge_filter[index]);
    }
}

template <storage::Ownership Ownership>
void read(storage::tar::FileReader &reader,
          const std::string &name,
          detail::ContractedMetric<Ownership> &metric)
{
    util::serialization::read(reader, name + "/contracted_graph", metric.graph);

    metric.edge_filter.resize(reader.ReadElementCount64(name + "/exclude"));
    for (const auto index : util::irange<std::size_t>(0, metric.edge_filter.size()))
    {
        storage::serialization::read(reader,
                                     name + "/exclude/" + std::to_string(index) + "/edge_filter",
                                     metric.edge_filter[index]);
    }
}

inline void write(storage::tar::FileWriter &writer, const std::string &name, const PhastData &data)
{
    writer.WriteFrom(name + "/version.meta", data.version);
    writer.WriteFrom(name + "/connectivity_checksum.meta", data.connectivity_checksum);
    writer.WriteFrom(name + "/node_count.meta", data.node_count);
    storage::serialization::write(writer, name + "/metric_name", data.metric_name);
    storage::serialization::write(writer, name + "/order", data.ordering.order);
    storage::serialization::write(writer, name + "/rank", data.ordering.rank);
    storage::serialization::write(writer,
                                  name + "/permutation/contraction_to_original",
                                  data.ordering.contraction_to_original);
    storage::serialization::write(writer,
                                  name + "/permutation/original_to_contraction",
                                  data.ordering.original_to_contraction);
}

inline void read(storage::tar::FileReader &reader, const std::string &name, PhastData &data)
{
    reader.ReadInto(name + "/version.meta", data.version);
    reader.ReadInto(name + "/connectivity_checksum.meta", data.connectivity_checksum);
    reader.ReadInto(name + "/node_count.meta", data.node_count);
    storage::serialization::read(reader, name + "/metric_name", data.metric_name);
    storage::serialization::read(reader, name + "/order", data.ordering.order);
    storage::serialization::read(reader, name + "/rank", data.ordering.rank);
    storage::serialization::read(reader,
                                 name + "/permutation/contraction_to_original",
                                 data.ordering.contraction_to_original);
    storage::serialization::read(reader,
                                 name + "/permutation/original_to_contraction",
                                 data.ordering.original_to_contraction);
}
} // namespace osrm::contractor::serialization

#endif
