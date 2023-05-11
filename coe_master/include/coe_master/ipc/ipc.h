
#ifndef COE_MASTER__IPC__COE_IPC_H
#define COE_MASTER__IPC__COE_IPC_H

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

#include <coe_core/coe_utilities.h>
#include <realtime_utilities/realtime_utilities.h>

namespace coe_master
{

/**
 * @class IPC
 *
 */
class IPC
{
public:
  typedef std::shared_ptr<IPC> Ptr;
  typedef std::shared_ptr<const IPC> ConstPtr;

  enum ErrorCode
  {
    NONE_ERROR = 0,
    UNMACTHED_DATA_DIMENSION = 1,
    UNCORRECT_CALL = 2,
    WATCHDOG = 3
  };

  struct IPCStruct
  {

    struct Header
    {
      uint8_t bond_flag_;
      uint8_t rt_flag_;
      double time_;
    } __attribute__((packed)) header_;

    char buffer[1024];
    IPCStruct() { clear(); }
    void clear()
    {
      header_.bond_flag_ = 0;
      header_.rt_flag_ = 0;
      header_.time_ = 0;
      std::memset(&buffer[0], 0x0, 1024 * sizeof(char));
    }
  };

  enum IPCAccessMode
  {
    CREATE,
    OPEN
  };

  IPC(const std::string &identifier, double operational_time, double watchdog_decimation, const IPCAccessMode &mode,
      const std::size_t dim);
  IPC(const std::string &identifier, double operational_time, double watchdog_decimation);
  ~IPC();

  bool isHardRT();
  bool setHardRT();
  bool setSoftRT();

  bool isBonded();
  bool bond();
  bool breakBond();
  ErrorCode update(const uint8_t *buffer, const double time, const std::size_t &n_bytes);
  ErrorCode flush(uint8_t *buffer, double *time, double *latency_time, const std::size_t &n_bytes);

  std::size_t getSize(bool prepend_header) const;
  std::string getName() const;
  double getWatchdog() const;
  std::string to_string(ErrorCode err);

  void dump(IPCStruct *buffer) { return getIPCStruct(buffer); }

protected:
  const IPCAccessMode access_mode_;
  const double operational_time_;
  const double watchdog_;

  const std::string name_;
  std::size_t dim_data_;
  std::size_t dim_with_header_;
  boost::interprocess::mapped_region shared_map_;
  boost::interprocess::shared_memory_object shared_memory_;
  std::shared_ptr<boost::interprocess::named_mutex> mutex_;
  double start_watchdog_time_;
  double data_time_prev_;
  double flush_time_prev_;

  std::size_t bond_cnt_;
  bool bonded_prev_;
  bool is_hard_rt_prev_;

  void getIPCStruct(IPCStruct *shmem);
  void setIPCStruct(const IPCStruct *shmem);
};
}

#endif  // COE_MASTER__IPC__COE_IPC_H
