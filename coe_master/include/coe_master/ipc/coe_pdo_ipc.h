
#ifndef COE_MASTER__IPC__COE_PDO_IPC_H
#define COE_MASTER__IPC__COE_PDO_IPC_H

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>

#include <tuple>

#include <coe_core/coe_pdo.h>
#include <coe_core/coe_utilities.h>
#include <coe_master/modules/coe_network_descriptor.h>

#include <coe_master/ipc/coe_ipc.h>

namespace coe_master
{

inline std::size_t pdoIPCDim(const ModuleDescriptorPtr &module, const int sdo_assignement)
{
  std::size_t dim =
      (sdo_assignement == ECT_SDO_RXPDOASSIGN ? module->getRxPdo().nBytes(true) : module->getTxPdo().nBytes(true));
  return dim;
}

inline std::string pdoIPCIdentifier(const ModuleDescriptorPtr &module, const int sdo_assignement)
{
  return (sdo_assignement == ECT_SDO_RXPDOASSIGN ? module->getSoemOutputId() : module->getSoemInputId());
}
inline std::string pdoIPCIdentifier(const std::string &identifier, const int sdo_assignement)
{
  return (identifier + (sdo_assignement == ECT_SDO_RXPDOASSIGN ? "_rxpdo" : "_txpdo"));
}

class PdoIPC : public IPC
{
public:
    typedef std::shared_ptr<PdoIPC> Ptr;

    PdoIPC(const ModuleDescriptorPtr &module, const int sdo_assignement, double operational_time,
           const IPCAccessMode &mode)
        : IPC(pdoIPCIdentifier(module, sdo_assignement), operational_time,
              (double)module->getWatchdogDecimation() * operational_time, mode, pdoIPCDim(module, sdo_assignement)),
          sdo_assignement_(sdo_assignement)
    {
    }

    PdoIPC(const std::string &identifier, const int sdo_assignement, double operational_time, int watchdog_decimation)
        : IPC(pdoIPCIdentifier(identifier, sdo_assignement), operational_time,
              (double)watchdog_decimation * operational_time),
          sdo_assignement_(sdo_assignement)
    {
      // nothing to do so far
    }

    ErrorCode update(const uint8_t *buffer, const double time, const std::size_t &n_bytes);
    ErrorCode flush(uint8_t *buffer, double *time, double *latency_time, const std::size_t &n_bytes);

private:
    const int sdo_assignement_;
  };

  typedef PdoIPC::Ptr PdoIPCPtr;

  class ModuleIPC
  {
public:
    typedef std::shared_ptr<ModuleIPC> Ptr;

    const std::string identifier_;
    PdoIPC rx_pdo_;
    PdoIPC tx_pdo_;

    ModuleIPC(const ModuleDescriptorPtr &module, double operational_time, const PdoIPC::IPCAccessMode &mode);
    ModuleIPC(const std::string &identifier, double operational_time, int watchdog_decimation);
    ~ModuleIPC();
  };

  typedef ModuleIPC::Ptr ModuleIPCPtr;

  class ModulesIPC
  {

public:
    typedef std::vector<ModuleIPCPtr> List;
    typedef List::iterator iterator;
    typedef List::const_iterator const_iterator;

    ~ModulesIPC();

    void clear();
    iterator begin();
    iterator end();

    const_iterator begin() const;
    const_iterator end() const;

    const_iterator cbegin() const;
    const_iterator cend() const;

    const ModuleIPCPtr &operator[](const std::string &i) const;
    ModuleIPCPtr &operator[](const std::string &i);

    bool insert(ModuleIPCPtr module_shm);

private:
  List modules_shm_;
};

} // namespace coe_master

#endif // COE_MASTER__IPC__COE_PDO_IPC_H
