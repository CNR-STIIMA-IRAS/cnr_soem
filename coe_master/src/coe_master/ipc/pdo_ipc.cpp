/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */
#include <sys/stat.h>
#include <sys/types.h>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread.hpp>

#include <ethercat.h>

#include <coe_core/coe_sdo.h>
#include <coe_master/ipc/coe_pdo_ipc.h>
#include <realtime_utilities/realtime_utilities.h>

namespace coe_master
{

  PdoIPC::ErrorCode PdoIPC::update(const uint8_t *ibuffer, double time, const std::size_t &n_bytes)
  {
    if (((sdo_assignement_ != ECT_SDO_TXPDOASSIGN) || (access_mode_ != CREATE)) &&
        ((sdo_assignement_ != ECT_SDO_RXPDOASSIGN) || (access_mode_ != OPEN)))
    {
      printf("FATAL ERROR! Wrong Shared Memory Call.");
      return coe_master::PdoIPC::UNCORRECT_CALL;
    }

    return IPC::update(ibuffer, time, n_bytes);
  }

  PdoIPC::ErrorCode PdoIPC::flush(uint8_t *obuffer, double *time, double *latency_time, const std::size_t &n_bytes)
  {
    if (((sdo_assignement_ != ECT_SDO_RXPDOASSIGN) || (access_mode_ != CREATE)) &&
        ((sdo_assignement_ != ECT_SDO_TXPDOASSIGN) || (access_mode_ != OPEN)))
    {
      printf("FATAL ERROR! Wrong Shared Memory Call.");
      return coe_master::PdoIPC::UNCORRECT_CALL;
    }

    return IPC::flush(obuffer, time, latency_time, n_bytes);
  }

  /****
   *
   *
   *
   *
   ****/
  ModuleIPC::ModuleIPC(const ModuleDescriptorPtr &module, double operational_time, const PdoIPC::IPCAccessMode &mode)
      : identifier_(module->getIdentifier()), rx_pdo_(module, ECT_SDO_RXPDOASSIGN, operational_time, mode),
        tx_pdo_(module, ECT_SDO_TXPDOASSIGN, operational_time, mode)
  {
  }

  ModuleIPC::ModuleIPC(const std::string &identifier, double operational_time, int watchdog_decimation)
      : identifier_(identifier), rx_pdo_(identifier, ECT_SDO_RXPDOASSIGN, operational_time, watchdog_decimation),
        tx_pdo_(identifier, ECT_SDO_TXPDOASSIGN, operational_time, watchdog_decimation)
  {
  }
  
  ModuleIPC::~ModuleIPC() {}

  ModulesIPC::~ModulesIPC()
  {
    printf("Destroying the Modules Shared Memory ");
    {
      for (auto e : modules_shm_)
      {
        e.reset();
      }
    }
  }

  void ModulesIPC::clear() { modules_shm_.clear(); }

  ModulesIPC::iterator ModulesIPC::begin() { return modules_shm_.begin(); }

  ModulesIPC::iterator ModulesIPC::end() { return modules_shm_.end(); }

  ModulesIPC::const_iterator ModulesIPC::begin() const { return modules_shm_.begin(); }

  ModulesIPC::const_iterator ModulesIPC::end() const { return modules_shm_.end(); }

  ModulesIPC::const_iterator ModulesIPC::cbegin() const { return modules_shm_.cbegin(); }

  ModulesIPC::const_iterator ModulesIPC::cend() const { return modules_shm_.cend(); }

  const ModuleIPCPtr &ModulesIPC::operator[](const std::string &i) const
  {
    auto const &it =
        std::find_if(modules_shm_.begin(), modules_shm_.end(), [&i](ModuleIPCPtr m) { return i == (m->identifier_); });
    if (it == modules_shm_.end())
      throw std::runtime_error(("Shared memory identifier '" + i + "' not in the mapped list").c_str());
    return *it;
  }

  ModuleIPCPtr &ModulesIPC::operator[](const std::string &i)
  {
    auto it = std::find_if(modules_shm_.begin(), modules_shm_.end(),
                           [&i](const ModuleIPCPtr &m) { return i == m->identifier_; });
    if (it == modules_shm_.end())
      throw std::runtime_error(("Shared memory identifier '" + i + "' not in the mapped list").c_str());
    return *it;
  }

  bool ModulesIPC::insert(ModuleIPCPtr module_shm)
  {
    for (const ModuleIPCPtr &module : modules_shm_)
    {
      if (module->identifier_ == module_shm->identifier_)
      {
        printf("Module already in the list.");
        return false;
      }
    }
    modules_shm_.push_back(module_shm);
    return true;
  }

}
