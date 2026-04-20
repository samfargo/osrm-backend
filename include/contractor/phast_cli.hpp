#ifndef OSRM_CONTRACTOR_PHAST_CLI_HPP
#define OSRM_CONTRACTOR_PHAST_CLI_HPP

#include "contractor/phast_common.hpp"

namespace osrm::contractor::phast
{

ReturnCode ParseArguments(int argc,
                          char *argv[],
                          std::string &verbosity,
                          PhastConfig &phast_config,
                          RuntimeConfig &runtime_config);

} // namespace osrm::contractor::phast

#endif
