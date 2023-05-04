/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */
#include <boost/interprocess/shared_memory_object.hpp>
#include <cnr_param/cnr_param.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_core/coe_sdo.h>
#include <coe_master/modules/coe_network_descriptor.h>
#include <coe_master/modules/coe_network_descriptor_data.h>

using namespace coe_core;

namespace coe_master
{

  bool NetworkDescriptor::initNetworkNames()
  {

    try
    {
      printf("[%s%s%s] %sExtract the coe configuration information from ros param server ", BOLDMAGENTA(), "START",
             RESET(), BOLDYELLOW());

      std::string what;
      cnr::param::node_t module_list;
      if (!cnr::param::get(param_root_ + std::string("/") + "adapter", adapter_, what) ||
          !cnr::param::get(param_root_ + std::string("/") + "operational_time", operational_time_, what) ||
          !cnr::param::get(param_root_ + std::string("/") + "module_list", module_list, what))
      {
        printf("[%s%s%s] %sError in the extraction of the configuration from param server: %s", BOLDRED(), "ERROR",
               RESET(), BOLDYELLOW(), what.c_str());
        return false;
      }

      for (std::size_t i = 0; i < cnr::param::size(module_list); i++)
      {
        cnr::param::node_t module;
        cnr::param::at(module_list,i,module,what);
        std::string label = cnr::param::extract<std::string>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::LABEL]);
        int addr = cnr::param::extract<int>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::ADDRESS]);
        bool mapped = cnr::param::extract<bool>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::MAPPED]);
        bool standard = cnr::param::extract<bool>(module, coe_master::NetworkData::KeysId[coe_master::NetworkData::DEFAULT]);

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
      printf("[%s%s%s] %sExtract the coe configuration information from ros param server ", BOLDGREEN(), "  OK ",
             RESET(), BOLDYELLOW());
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
    std::transform(module_addresses_map_.begin(), module_addresses_map_.end(), mapped_addresses.begin(),
                   [](const std::pair<int, std::string> &m) { return m.first; });

    return mapped_addresses;
  }

  std::vector<std::string> NetworkDescriptor::getNodeUniqueIDs() const
  {
    std::vector<std::string> mapped_names;
    mapped_names.resize(module_addresses_map_.size());
    std::transform(module_addresses_map_.begin(), module_addresses_map_.end(), mapped_names.begin(),
                   [](const std::pair<int, std::string> &m) { return uniqueId(m.second, m.first); });

    return mapped_names;
  }

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
    std::map<int, std::string> sorted_names;
    for (auto const &module : module_addresses_map_)
    {
      sorted_names[module.first] = uniqueId(module.second, module.first);
    }

    return sorted_names;
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

  bool NetworkDescriptor::checkAddress(int addr) const
  {
    return module_addresses_map_.find(addr) != module_addresses_map_.end();
    return false;
  }

  int NetworkDescriptor::checkModuleName(const std::string &id, const bool verbose) const
  {
    int ret = -1;

    auto it = std::find_if(module_addresses_map_.begin(), module_addresses_map_.end(),
                           [&id](const std::pair<int, std::string> &m) { return uniqueId(m.second, m.first) == id; });

    ret = (it != module_addresses_map_.end()) ? it->first : -1;

    if ((ret == -1) && verbose)
    {
      printf("[checkModuleName] Requested: '%s', while the name available are:", id.c_str());
      for (auto const &s : getNodeUniqueIDs())
      {
        std::cout << "- " << s << std::endl;
      }
    }
    return ret;
  }

}
