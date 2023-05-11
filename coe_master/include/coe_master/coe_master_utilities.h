#ifndef COE_MASTER__COE_MASTER_UTILITES__H
#define COE_MASTER__COE_MASTER_UTILITES__H

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <iostream>
#include <mutex>

#include <ethercat.h>
#include <ethercatdc.h>
#include <ethercattype.h>

#include <cnr_param/cnr_param.h>
#include <realtime_utilities/circular_buffer.h>
#include <realtime_utilities/realtime_utilities.h>

#include <coe_core/coe_sdo.h>
#include <coe_core/coe_utilities.h>
#include <coe_master/ipc/coe_ipc.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_network_descriptor.h>
#include <coe_master/ipc/coe_srv_utilities.h>
#include <coe_soem_utilities/coe_soem_utilities.h>

class MainThreadSharedData
{
 private:
  const int windows_dim_;
  std::mutex mtx_;
  double cycle_time_;
  int expectedWKC_;
  char* IOmap_;
  realtime_utilities::circ_buffer<double> latency_msr_;
  realtime_utilities::circ_buffer<double> cycle_time_msr_;
  realtime_utilities::circ_buffer<double> calc_time_;
  realtime_utilities::circ_buffer<uint32_t> missed_cycles_;
  realtime_utilities::circ_buffer<int> wkc_;
  std::map<std::string, bool> rxbond_;
  std::map<std::string, bool> txbond_;
  std::map<std::string, bool> rxhardrt_;
  std::map<std::string, bool> txhardrt_;

 public:
  MainThreadSharedData(const int windows_dim)
      : windows_dim_(windows_dim),
        cycle_time_(0),
        expectedWKC_(0),
        IOmap_(NULL),
        latency_msr_(windows_dim),
        cycle_time_msr_(windows_dim),
        calc_time_(windows_dim),
        missed_cycles_(windows_dim),
        wkc_(windows_dim)
  {
  }

  int getWindowDim() { return windows_dim_; }
  double getCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return cycle_time_;
  }
  int getExpectedWKC()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return expectedWKC_;
  }
  char* getIOmap()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return &IOmap_[0];
  }

  double getMeanActCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(cycle_time_msr_.get()) * 1e3;
  }
  double getMeanCalcTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(calc_time_.get()) * 1e3;
  }
  double getMeanLatencyTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(latency_msr_.get()) * 1e3;
  }
  uint32_t getMeanMissedCycles()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(missed_cycles_.get());
  }
  int getMeanWkc()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::mean(wkc_.get());
  }

  double getMaxActCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(cycle_time_msr_.get()) * 1e3;
  }
  double getMaxCalcTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(calc_time_.get()) * 1e3;
  }
  double getMaxLatencyTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(latency_msr_.get()) * 1e3;
  }
  uint32_t getMaxMissedCycles()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(missed_cycles_.get());
  }
  int getMaxWkc()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::max(wkc_.get());
  }

  double getMinActCycleTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(cycle_time_msr_.get()) * 1e3;
  }
  double getMinCalcTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(calc_time_.get()) * 1e3;
  }
  double getMinLatencyTime()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(latency_msr_.get()) * 1e3;
  }
  uint32_t getMinMissedCycles()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(missed_cycles_.get());
  }
  int getMinWkc()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return realtime_utilities::min(wkc_.get());
  }

  void setCycleTime(double v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cycle_time_ = v;
  }
  void setExpectedWKC(int v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    expectedWKC_ = v;
  }
  void setIOmap(char* v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    IOmap_ = v;
  }

  void setActualCycleTime(double v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cycle_time_msr_.push_back(v);
  }
  void setLatencyTime(double v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latency_msr_.push_back(v);
  }
  void setCalcTime(double v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    calc_time_.push_back(v);
  }
  void setMissedCycles(uint32_t v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    missed_cycles_.push_back(v);
  }
  int setWkc(int v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    wkc_.push_back(v);
    return v;
  }

  void setRxBonded(std::string s, bool v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    rxbond_[s] = v;
  }
  std::map<std::string, bool> getRxBonded()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return rxbond_;
  }

  void setTxBonded(std::string s, bool v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    txbond_[s] = v;
  }
  std::map<std::string, bool> getTxBonded()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return txbond_;
  }

  void setRxHardRT(std::string s, bool v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    rxhardrt_[s] = v;
  }
  std::map<std::string, bool> getRxHardRT()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return rxhardrt_;
  }

  void setTxHardRT(std::string s, bool v)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    txhardrt_[s] = v;
  }
  std::map<std::string, bool> getTxHardRT()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return txhardrt_;
  }

  void incIOmap()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    IOmap_[0]++;
  }
};

static const std::size_t SDO_STACK_SIZE = 100 * 1024;

inline bool initNodes(const coe_master::NetworkDescriptorPtr network,
                      std::vector<coe_master::ModuleDescriptorPtr>& nodes)
{
  try
  {
    printf("[%s%s%s] %sInit the nodes of the network", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW());

    cnr::param::node_t param_root;
    rosparam_utilities::extractParam(nh, network->getNamespace(), param_root);

    std::map<int, std::string> module_addresses_map = network->getAddressLabelsMap();
    for (auto module_address : module_addresses_map)
    {
      bool default_config = network->hasDefaultConfiguration(module_address.first);
      std::string msg =
          std::string(RESET()) + "[-----]" +
          std::string("[ " + (BOLDBLUE() + std::string("Init Params") + RESET()) + " ] ") +
          std::string("[ " +
                      (BOLDCYAN() + std::to_string(module_address.first) + "# " + module_address.second + RESET()) +
                      " ] ") +
          std::string("[ " + (BOLDYELLOW() + std::string(default_config ? "DEFAULT" : " PARAMS") + RESET()) + " ] ");

      std::cout << msg << " [ " << BOLDMAGENTA() << "RUNNING" << RESET()
                           << " ] ");
      coe_master::ModuleDescriptorPtr module(new coe_master::ModuleDescriptor(
          nh, network->getNamespace(), module_address.second, module_address.first, default_config));

      if (!module->initNodeConfigurationFromParams())
      {
        std::cout << msg << " [ " << RED() << " ERROR " << RESET()
                             << " ] Basic Configurations from params failed. ");
        return false;
      }

      if (network->hasDefaultConfiguration(module_address.first))
      {
        if (!module->initNodeCoeConfigurationFromSoem(module_address.first, module_address.second, true))
        {
          std::cout << msg << " [ " << RED() << " ERROR " << RESET()
                               << " ] Configurations from SOEM failed. ");
          return false;
        }
      }
      else
      {
        if (!module->initNodeCoeConfigurationFromParams(true))  // force sdo configuration from ROSPARAM
        {
          std::cout << msg
                           << " [ " << RED() << " ERROR " << RESET()
                           << " ] Configurations from Param Server failed. ");
          return false;
        }
      }

      auto jt = std::find_if(nodes.begin(),
                             nodes.end(),
                             [&module](const coe_master::ModuleDescriptorPtr& m)
                             { return m->getIdentifier() == module->getIdentifier(); });
      if (jt != nodes.end())
      {
        std::cout << 
            msg << " [ " << RED() << " ERROR " << RESET()
                << " ] There are two modules with the same 'identifier'. ");
        return false;
      }

      if (!module->initHandles())
      {
        std::cout << msg << " [ " << RED() << " ERROR " << RESET()
                             << " ] Configurations from SOEM failed. ");
        return false;
      }

      //
      nodes.push_back(module);

      std::cout << msg << " [ " << BOLDGREEN() << "OK" << RESET() << " ]");
    }

    if (nodes.size() != nodes.size())
    {
      std::cout << " [ ERROR : Number of modules configured: "
                       << nodes.size()
                       << "/ Number of modules mapped in the rosparam server: "
                       << module_addresses_map.size() << "]");
      return false;
    }

    printf("[%s%s%s] %sInit the nodes of the network ", BOLDGREEN(), "  OK ", RESET(), BOLDYELLOW());
  }
  catch (std::exception& e)
  {
    printf("--------------------------");
    printf("%s", e.what());
    printf("--------------------------");
    return false;
  }

  return true;
}

inline bool updateNodeConfiguration(coe_master::ModuleDescriptorPtr& module, char* IOmap)
{
  bool ret = false;
  std::string HDR = std::string(
      std::string(RESET()) + std::string("[-----]") + "[ " +
      (BOLDCYAN() + std::to_string(module->getAddress()) + "# " + module->getIdentifier() + RESET()) +
      (BOLDCYAN() + std::string(module->isDefaultConfig() ? " - DEFAULT CFG" : " - PARAMS CFG") + RESET()) + " ]");

  if (ec_slave[module->getAddress()].mbx_proto & ECT_MBXPROT_COE)
  {
    std::cout << HDR << " PDO Mapping through SDO [ " << BOLDMAGENTA()
                         << "RUNNING" << RESET() << "] ");
    ret = coe_soem_utilities::get_pdo_map_through_sdo(
        module->getAddress(), module->getRxPdo(), module->getTxPdo(), IOmap);
    std::cout << 
        HDR << " PDO Mapping through SDO [ "
            << (ret ? (BOLDGREEN() + std::string("  OK   ") + RESET())
                    : (BOLDRED() + std::string(" CHECK ") + RESET()))
            << "] ");
  }
  else
  {
    std::cout << HDR << " PDO Mapping through SII [ " << BOLDMAGENTA()
                         << "RUNNING" << RESET() << "]");
    ret = coe_soem_utilities::get_pdo_map_through_sii(
        module->getAddress(), module->getRxPdo(), module->getTxPdo(), IOmap);
    std::cout << 
        HDR << " PDO Mapping through SII [ "
            << (ret ? (BOLDGREEN() + std::string("  OK   ") + RESET())
                    : (BOLDRED() + std::string(" CHECK ") + RESET()))
            << "] ");
  }

  if (ret)
  {
    std::cout << HDR << " PDO Mapping Finalization [ " << BOLDMAGENTA()
                         << "RUNNING" << RESET() << "] ");
    module->getTxPdo().finalize();
    module->getRxPdo().finalize();
    std::cout << HDR << " PDO Mapping Finalization [ " << BOLDGREEN()
                         << " OK " << RESET() << "] ");
  }

  std::cout << HDR << " Handles Connection [ " << BOLDMAGENTA() << "RUNNING"
                       << RESET() << "] ");
  if (!module->connectHandles())
  {
    std::cout << HDR << " Handles Connection [ " << RED() << "ERROR"
                         << RESET() << "] ");
    return false;
  }
  std::cout << HDR << " Handles Connection [ " << BOLDGREEN() << "OK"
                       << RESET() << "] ");
  module->setConfiguratedSdo(ret);
  return ret;
}

inline bool updateNodes(std::vector<coe_master::ModuleDescriptorPtr>& nodes, char* IOmap, bool update_ros_param_server)
{
  try
  {
    printf("[%s%s%s] %sUpdate the nodes of the network", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW());

    for (coe_master::ModuleDescriptorPtr module : nodes)
    {
      updateNodeConfiguration(module, IOmap);

      if (update_ros_param_server)
      {
        module->updateROSParamServer();
      }
    }

    printf("[%s%s%s] %sUpdate the nodes of the network", BOLDGREEN(), "  OK ", RESET(), BOLDYELLOW());
  }
  catch (std::exception& e)
  {
    printf("--------------------------");
    printf("%s", e.what());
    printf("--------------------------");
    return false;
  }

  return true;
}

inline bool configDc(const std::vector<coe_master::ModuleDescriptorPtr>& nodes)
{
  auto it = std::find_if(
      nodes.begin(), nodes.end(), [](const coe_master::ModuleDescriptorPtr& m) { return m->isDcEnabled(); });
  return it != nodes.end();
}

inline std::map<int, bool> configSdoCa(const std::vector<coe_master::ModuleDescriptorPtr>& nodes)
{
  std::map<int, bool> ret;
  for (auto const& module : nodes)
  {
    ret[module->getAddress()] = module->isSdoCaSupported();
  }

  return ret;
}

inline std::vector<std::string> getNodeUniqueIDs(const std::vector<coe_master::ModuleDescriptorPtr>& nodes)
{
  std::vector<std::string> ret(nodes.size());
  std::transform(
      nodes.begin(), nodes.end(), ret.begin(), [](coe_master::ModuleDescriptorPtr m) { return m->getIdentifier(); });
  return ret;
}

inline int checkModuleName(const std::string& id,
                           const std::vector<coe_master::ModuleDescriptorPtr>& nodes,
                           const bool verbose)
{
  int ret = -1;

  auto it = std::find_if(
      nodes.begin(), nodes.end(), [&id](const coe_master::ModuleDescriptorPtr& m) { return m->getIdentifier() == id; });
  ret = (it != nodes.end()) ? (*it)->getAddress() : -1;

  if ((ret == -1) && verbose)
  {
    printf("[checkModuleName] Requested: '%s', while the name available are:", id.c_str());
    for (auto const& s : getNodeUniqueIDs(nodes))
    {
      std::cout << "- " << s << std::endl;
    }
  }
  return ret;
}

#endif  // COE_MASTER__COE_MASTER_UTILITES__H
