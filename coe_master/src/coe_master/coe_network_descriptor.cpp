/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */
#include <cnr_param/cnr_param.h>
#include <coe_core/coe_sdo.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_master/modules/coe_network_descriptor.h>
#include <coe_master/modules/coe_network_descriptor_data.h>
#include <coe_master/modules/utilities/common.h>
#include <boost/interprocess/shared_memory_object.hpp>

using namespace coe_core;

namespace coe_master
{

bool NetworkDescriptor::initNetworkNames()
{
  try
  {
    printf("[%s%s%s] %sExtract the coe configuration information from ros param server ",
           BOLDMAGENTA(),
           "START",
           RESET(),
           BOLDYELLOW());

    std::string what;
    cnr::param::node_t module_list;
    if (!cnr::param::get(param_root_ + std::string("/") + "adapter", adapter_, what) ||
        !cnr::param::get(param_root_ + std::string("/") + "operational_time", operational_time_, what) ||
        !cnr::param::get(param_root_ + std::string("/") + "module_list", module_list, what))
    {
      printf("[%s%s%s] %sError in the extraction of the configuration from param server: %s",
             BOLDRED(),
             "ERROR",
             RESET(),
             BOLDYELLOW(),
             what.c_str());
      return false;
    }

    for (std::size_t i = 0; i < cnr::param::size(module_list); i++)
    {
      cnr::param::node_t module;
      cnr::param::at(module_list, i, module, what);
      std::string label =
          cnr::param::extract<std::string>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::LABEL]);
      int addr = cnr::param::extract<int>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::ADDRESS]);
      bool mapped = cnr::param::extract<bool>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::MAPPED]);
      bool standard =
          cnr::param::extract<bool>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::DEFAULT]);

      if (mapped)
      {
        if (module_addresses_map_.count(addr) > 0)
        {
          printf("More than one module has the same address '%d'. Abort.", addr);
          return false;
        }
        module_addresses_map_[addr] = label;
        module_default_map_[addr] = standard;
      }
    }
    printf("[%s%s%s] %sExtract the coe configuration information from ros param server ",
           BOLDGREEN(),
           "  OK ",
           RESET(),
           BOLDYELLOW());
  }
  catch (std::exception &e)
  {
    printf("--------------------------");
    printf("%s", e.what());
    printf("--------------------------");
    return false;
  }

  return true;
}

std::vector<int> NetworkDescriptor::getAddresses() const
{
  std::vector<int> mapped_addresses;
  mapped_addresses.resize(module_addresses_map_.size());
  std::transform(module_addresses_map_.begin(),
                 module_addresses_map_.end(),
                 mapped_addresses.begin(),
                 [](const std::pair<int, std::string> &m) { return m.first; });

  return mapped_addresses;
}

std::vector<std::string> NetworkDescriptor::getNodeUniqueIDs() const { return getNodeUniqueIDs(module_addresses_map_); }

std::map<int, std::string> NetworkDescriptor::getAddressLabelsMap() const
{
  std::map<int, std::string> sorted_names;
  for (auto const &module : module_addresses_map_)
  {
    sorted_names[module.first] = module.second;
  }

  return sorted_names;
}

std::map<int, std::string> NetworkDescriptor::getAddressUniqueIdMap() const
{
  return getAddressUniqueIdMap(module_addresses_map_);
}

bool NetworkDescriptor::hasDefaultConfiguration(int addr) const
{
  auto it = module_default_map_.find(addr);
  if (it != module_default_map_.end())
  {
    return module_default_map_.at(addr);
  }
  return false;
}

bool NetworkDescriptor::checkAddress(int addr) const { return checkAddress(module_addresses_map, int addr); }

int NetworkDescriptor::checkModuleName(const std::string &id, const bool verbose) const
{
  std::string what;
  int ret = checkModuleName(module_addresses_map_, id, what);

  if (verbose)
  {
    printf("%s", what.c_str());
  }
  return ret;
}

}  // namespace coe_master
