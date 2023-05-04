
#ifndef COE_MASTER__MODULES__COE_NETWORK_DESCRIPTOR_H
#define COE_MASTER__MODULES__COE_NETWORK_DESCRIPTOR_H

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

#include <coe_core/coe_pdo.h>
#include <coe_core/coe_utilities.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <tuple>

namespace coe_master
{

class NetworkDescriptor
{
public:
  typedef std::shared_ptr<NetworkDescriptor> Ptr;

  NetworkDescriptor()
  {
    // nothing to do so far
  }

  NetworkDescriptor(const std::string &coe_device_parameter_namespace)
      : param_root_(coe_device_parameter_namespace), adapter_("none"), operational_time_(-1)
  {
    // nothing to do so far
  }

  void initialize(const std::string &coe_device_parameter_namespace)
  {
    param_root_ = coe_device_parameter_namespace;
    adapter_ = "none";
    operational_time_ = -1;
  }

  bool initNetworkNames();

  const std::string &getNamespace() const { return param_root_; }
  const std::string &getAdapterName() const { return adapter_; }
  const double &getOperationalTime() const { return operational_time_; }

  std::vector<int> getAddresses() const;
  std::vector<std::string> getNodeUniqueIDs() const;
  std::map<int, std::string> getAddressLabelsMap() const;
  std::map<int, std::string> getAddressUniqueIdMap() const;

  bool hasDefaultConfiguration(int addr) const;

  bool checkAddress(int addr) const;
  int checkModuleName(const std::string &id, const bool verbose = false) const;

private:
  std::string param_root_;

  std::string adapter_;
  double operational_time_;
  std::map<int, std::string> module_addresses_map_;
  std::map<int, bool> module_default_map_;
};

typedef NetworkDescriptor::Ptr NetworkDescriptorPtr;

} // namespace coe_master

#endif // COE_MASTER__MODULES__COE_NETWORK_DESCRIPTOR_H
