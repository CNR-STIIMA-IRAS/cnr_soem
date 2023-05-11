#include <coe_core/coe_string_utilities.h>
#include <coe_master/ipc/coe_ipc.h>

#include <ctime>
#if defined(_MSC_VER)
#define VAL_LIKELY(x)       (x)
#define VAL_UNLIKELY(x)     (x)
#else
#define VAL_LIKELY(x)       __builtin_expect((x),1)
#define VAL_UNLIKELY(x)     __builtin_expect((x),0)
#endif

#define CONSOLE_THROTTLE_CHECK(now, last, period)\
  (VAL_UNLIKELY(last + period <= now) || VAL_UNLIKELY(now < last))

#define TIME_NOW()\
  std::time(0)

#define PRINTF_THROTTLE(period, ...)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      printf(__VA_ARGS__); \
    } \
  } while (false);

using namespace coe_core;

namespace coe_master
{

IPC::IPC(const std::string &identifier, double operational_time, double watchdog_decimation,
          const IPCAccessMode &mode, const std::size_t dim)
    : access_mode_(mode), operational_time_(operational_time), watchdog_(watchdog_decimation), name_(identifier),
      dim_data_(dim), dim_with_header_(dim + sizeof(IPC::IPCStruct::Header)) // time and bonding index
      ,
      start_watchdog_time_(-1), data_time_prev_(0), flush_time_prev_(0), bond_cnt_(0), bonded_prev_(false),
      is_hard_rt_prev_(false)
{
  bool ok = true;
  try
  {
    printf("[%sSTART%s] %sShMem Init%s [ %s%s%s ]", BOLDMAGENTA(), RESET(), BOLDBLUE(), RESET(), BOLDCYAN(),
              name_.c_str(), RESET());
    if (dim_data_ == 0)
    {
      printf("[%sCHECK%s%s] %sShMem Init%s [ %s%s%s%s ] Memory does not exist, is it correct?", BOLDRED(), RESET(),
                YELLOW(), BLUE(), RESET(), BOLDCYAN(), name_.c_str(), RESET(), YELLOW());
      return;
    }

    //------------------------------------------
    boost::interprocess::permissions permissions(0677);
    if (access_mode_ == CREATE)
    {
      if (!boost::interprocess::named_mutex::remove(name_.c_str()))
      {
        printf("[CHECK] %sShMem Init%s [ %s%s%s%s ] Error in Removing Mutex", BLUE(), YELLOW(), BOLDCYAN(),
                  name_.c_str(), RESET(), YELLOW());
      }

      if (!boost::interprocess::shared_memory_object::remove(name_.c_str()))
      {
        printf("[CHECK] %sShMem Init%s [ %s%s%s%s ] Error in Removing Shmem", BLUE(), YELLOW(), BOLDCYAN(),
                  name_.c_str(), RESET(), YELLOW());
      }

      // store old
      mode_t old_umask = umask(0);

      printf("[-----] %sShMem Init%s [ %s%s%s ] Sizeof uint8 %zu sizeof double %zu sizeof header %zu", BLUE(),
                RESET(), BOLDCYAN(), name_.c_str(), RESET(), sizeof(uint8_t), sizeof(double),
                sizeof(IPCStruct::Header));
      printf("[-----] %sShMem Init%s [ %s%s%s ] Create memory (bytes %s%zu/%zu%s)", BLUE(), RESET(), BOLDCYAN(),
                name_.c_str(), RESET(), BOLDCYAN(), dim_data_, dim_with_header_, RESET());
      shared_memory_ = boost::interprocess::shared_memory_object(boost::interprocess::create_only, name_.c_str(),
                                                                  boost::interprocess::read_write, permissions);

      shared_memory_.truncate(dim_with_header_);
      shared_map_ = boost::interprocess::mapped_region(shared_memory_, boost::interprocess::read_write);

      std::memset(shared_map_.get_address(), 0, shared_map_.get_size());

      mutex_.reset(
          new boost::interprocess::named_mutex(boost::interprocess::create_only, name_.c_str(), permissions));

      // restore old
      umask(old_umask);
    }
    else
    {
      printf("[-----] %sShMem Init%s [ %s%s%s ] Bond to Shared Memory (bytes %s%zu/%zu%s)", BLUE(), RESET(),
                BOLDCYAN(), name_.c_str(), RESET(), BOLDCYAN(), dim_data_, dim_with_header_, RESET());
      shared_memory_ = boost::interprocess::shared_memory_object(boost::interprocess::open_only, name_.c_str(),
                                                                  boost::interprocess::read_write);

      shared_map_ = boost::interprocess::mapped_region(shared_memory_, boost::interprocess::read_write);

      printf("[-----] %sShMem Init%s [ %s%s%s ] Bond to Mutex", BLUE(), RESET(), BOLDCYAN(), name_.c_str(),
                RESET());
      mutex_.reset(new boost::interprocess::named_mutex(boost::interprocess::open_only, name_.c_str()));
    }
    printf("[%s DONE%s] %sShMem Init%s [ %s%s%s ]", BOLDGREEN(), RESET(), BOLDBLUE(), RESET(), BOLDCYAN(),
              name_.c_str(), RESET());
  }
  catch (boost::interprocess::interprocess_exception &e)
  {
    std::cout << "In processing module '" << identifier << "' got the error: " << e.what() << std::endl;
    ok = false;
  }
  catch (std::exception &e)
  {
    std::cout <<"In processing module '" << identifier << "' got the error: " << e.what() << std::endl;
    ok = false;
  }

  if (!ok)
    throw std::runtime_error("Error in creation of the shared memory. Exit.");
}

IPC::IPC(const std::string &identifier, double operational_time, double watchdog_decimation)
    : access_mode_(OPEN), operational_time_(operational_time), watchdog_(watchdog_decimation), name_(identifier),
      dim_data_(0), dim_with_header_(0 + sizeof(IPC::IPCStruct::Header)) // time and bonding index
      ,
      start_watchdog_time_(-1), data_time_prev_(0), flush_time_prev_(0), bond_cnt_(0), bonded_prev_(false),
      is_hard_rt_prev_(false)
{
  bool ok = true;
  try
  {

    printf("[-----] %sShMem Init%s [ %s%s%s ] Check Memory", BLUE(), RESET(), BOLDCYAN(), name_.c_str(), RESET());
    shared_memory_ = boost::interprocess::shared_memory_object(boost::interprocess::open_only, name_.c_str(),
                                                                boost::interprocess::read_write);

    shared_map_ = boost::interprocess::mapped_region(shared_memory_, boost::interprocess::read_write);

    printf("[-----] %sShMem Init%s [ %s%s%s ] Check Mutex", BLUE(), RESET(), BOLDCYAN(), name_.c_str(), RESET());
    mutex_.reset(new boost::interprocess::named_mutex(boost::interprocess::open_only, name_.c_str()));

    dim_with_header_ = shared_map_.get_size();
    dim_data_ = shared_map_.get_size() - sizeof(IPC::IPCStruct::Header);

    assert(dim_data_ > 0);

    printf("[%sREADY%s] %sShMem Init%s [ %s%s%s ] Ready", BOLDGREEN(), RESET(), BLUE(), RESET(), BOLDCYAN(),
              name_.c_str(), RESET());
  }
  catch (boost::interprocess::interprocess_exception &e)
  {
    if (e.get_error_code() == boost::interprocess::not_found_error)
    {
      printf("[ %s%s%s ] Memory does not exist, Check if correct?", BOLDCYAN(), name_.c_str(), YELLOW());
      dim_data_ = 0.0;
      return;
    }
    else
    {
      std::cout << "In processing module '" << identifier << "' [" << name_ << "] got the error: " << e.what()
                                                << " error code: " << e.get_error_code() << std::endl;
      ok = false;
    }
  }
  catch (std::exception &e)
  {
    std::cout << "In processing module '" << identifier << "' got the error: " << e.what() << std::endl;
    ok = false;
  }

  if (!ok)
    throw std::runtime_error("Error in creation of the shared memory. Exit.");
}

IPC::~IPC()
{

  if (dim_data_ > 0)
  {
    printf("[ %s%s%s ][ %sShMem Destructor%s ] Shared Mem Destructor", BOLDCYAN(), name_.c_str(), RESET(),
              BOLDBLUE(), RESET());

    if (isBonded())
      breakBond();

    if (access_mode_ == CREATE)
    {

      printf("[ %s%s%s ][ %sShMem Destructor%s ] Remove Shared Mem ", BOLDCYAN(), name_.c_str(), RESET(),
                BOLDBLUE(), RESET());

      if (!boost::interprocess::shared_memory_object::remove(name_.c_str()))
      {
        printf("Error in removing the shared memory object");
      }

      printf("[ %s%s%s ][ %sShMem Destructor%s ] Remove Mutex", BOLDCYAN(), name_.c_str(), RESET(), BOLDBLUE(),
                RESET());

      if (!boost::interprocess::named_mutex::remove(name_.c_str()))
      {
        printf("[ %s%s%s ][ %sShMem Destructor%s ] Error", BOLDCYAN(), name_.c_str(), RED(), BOLDBLUE(), RESET());
      }
    }
  }
}

void IPC::getIPCStruct(IPCStruct *shmem)
{
  assert(shmem);
  assert(shared_map_.get_size() < sizeof(IPC::IPCStruct));

  shmem->clear();
  boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(
      *mutex_); // from local buffer to shared memory
  std::memcpy(shmem, shared_map_.get_address(), shared_map_.get_size());
  lock.unlock();
}

void IPC::setIPCStruct(const IPCStruct *shmem)
{
  assert(shmem);
  assert(shared_map_.get_size() < sizeof(IPC::IPCStruct));

  boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*mutex_);
  std::memcpy(shared_map_.get_address(), shmem, shared_map_.get_size());
  lock.unlock();
}

bool IPC::isHardRT()
{
  if (shared_map_.get_size() == 0)
    return false;

  IPCStruct shmem;
  getIPCStruct(&shmem);

  bool is_hard_rt = (shmem.header_.rt_flag_ == 1);
  if (is_hard_rt_prev_ != is_hard_rt)
  {
    printf("[ %s%s%s ] RT State Changed from '%s%s%s' to '%s%s%s'", BOLDCYAN(), name_.c_str(), RESET(), BOLDCYAN(),
              (is_hard_rt_prev_ ? "HARD" : "SOFT"), RESET(), BOLDCYAN(), (is_hard_rt ? "HARD" : "SOFT"), RESET());
    is_hard_rt_prev_ = is_hard_rt;
  }
  return (shmem.header_.rt_flag_ == 1);
}

bool IPC::setHardRT()
{
  if (shared_map_.get_size() == 0)
    return false;

  printf("[ %s%s%s ] [%sSTART%s] Set Hard RT", BOLDCYAN(), name_.c_str(), RESET(), BOLDCYAN(), RESET());

  IPCStruct shmem;
  getIPCStruct(&shmem);

  if ((shmem.header_.rt_flag_ == 1))
  {
    printf("[ %s%s%s%s ] Already hard RT!", BOLDCYAN(), name_.c_str(), RESET(), RED());
  }

  shmem.header_.rt_flag_ = 1;
  setIPCStruct(&shmem);

  printf("[ %s%s%s ] [%s DONE%s] Set Hard RT", BOLDCYAN(), name_.c_str(), RESET(), BOLDGREEN(), RESET());
  return true;
}

bool IPC::setSoftRT()
{
  if (shared_map_.get_size() == 0)
    return false;

  printf("[ %s%s%s ] [%sSTART%s] Set Soft RT", BOLDCYAN(), name_.c_str(), RESET(), BOLDCYAN(), RESET());

  IPCStruct shmem;
  getIPCStruct(&shmem);

  if ((shmem.header_.rt_flag_ == 0))
  {
    printf("[ %s%s%s%s ] Already soft RT!", BOLDCYAN(), name_.c_str(), RESET(), RED());
  }

  shmem.header_.rt_flag_ = 0;
  setIPCStruct(&shmem);

  printf("[ %s%s%s ] [%s DONE%s] Set soft RT", BOLDCYAN(), name_.c_str(), RESET(), BOLDGREEN(), RESET());
  return true;
}

bool IPC::isBonded()
{
  if (shared_map_.get_size() == 0)
    return false;

  IPCStruct shmem;
  getIPCStruct(&shmem);

  bool is_bonded = (shmem.header_.bond_flag_ == 1);
  if (bonded_prev_ != is_bonded)
  {
    printf("[ %s%s%s ] Bonding State Changed from '%s%s%s' to '%s%s%s'", BOLDCYAN(), name_.c_str(), RESET(),
              BOLDCYAN(), (bonded_prev_ ? "BONDED" : "UNBONDED"), RESET(), BOLDCYAN(),
              (is_bonded ? "BONDED" : "UNBONDED"), RESET());
    bonded_prev_ = is_bonded;
  }
  return (shmem.header_.bond_flag_ == 1);
}

bool IPC::bond()
{
  if (shared_map_.get_size() == 0)
    return false;

  printf("[ %s%s%s ] [%sSTART%s] Bonding", BOLDCYAN(), name_.c_str(), RESET(), BOLDCYAN(), RESET());

  IPCStruct shmem;
  getIPCStruct(&shmem);

  if ((shmem.header_.bond_flag_ == 1))
  {
    printf("[ %s%s%s%s ] Already Bonded! Abort. \n\n****** RESET CMD FOR SAFETTY **** \n", BOLDCYAN(),
              name_.c_str(), RESET(), RED());
    return false;
  }

  shmem.header_.bond_flag_ = 1;
  setIPCStruct(&shmem);

  printf("[ %s%s%s ] [%sDONE%s] Bonding", BOLDCYAN(), name_.c_str(), RESET(), BOLDGREEN(), RESET());
  return true;
}

bool IPC::breakBond()
{
  printf("[ %s%s%s ] Break Bond", BOLDCYAN(), name_.c_str(), RESET());
  IPCStruct shmem;
  getIPCStruct(&shmem);

  shmem.header_.rt_flag_ = 0;
  shmem.header_.bond_flag_ = 0;

  setIPCStruct(&shmem);

  return true;
}

IPC::ErrorCode IPC::update(const uint8_t *ibuffer, double time, const std::size_t &n_bytes)
{
  if (dim_data_ != n_bytes)
  {
    printf("FATAL ERROR! Shared memory map '%zu' bytes, while the input is of '%zu' bytes", dim_data_, n_bytes);
    return coe_master::IPC::UNMACTHED_DATA_DIMENSION;
  }

  if (dim_data_ <= 0)
  {
    return coe_master::IPC::NONE_ERROR;
  }

  IPCStruct shmem;
  getIPCStruct(&shmem);

  if (shmem.header_.bond_flag_ == 1)
  {
    shmem.header_.time_ = time;
    std::memcpy(shmem.buffer, ibuffer, dim_data_);
  }

  setIPCStruct(&shmem);

  return coe_master::IPC::NONE_ERROR;
}

IPC::ErrorCode IPC::flush(uint8_t *obuffer, double *time, double *latency_time, const std::size_t &n_bytes)
{
  IPC::ErrorCode ret = coe_master::IPC::NONE_ERROR;
  if (dim_data_ != n_bytes)
  {
    printf("FATAL ERROR! Wrong Memory Dimensions.");
    return coe_master::IPC::UNMACTHED_DATA_DIMENSION;
  }

  if (dim_data_ <= 0)
  {
    return coe_master::IPC::NONE_ERROR;
  }

  IPCStruct shmem;
  getIPCStruct(&shmem);

  if (shmem.header_.bond_flag_ == 1)
  {
    *time = shmem.header_.time_;
    std::memcpy(obuffer, shmem.buffer, dim_data_);
  }
  else
  {
    // printf_THROTTLE( 2, "[ %s%s%s%s ] SAFETTY CMD (not bonded)", BOLDCYAN(), name_.c_str(), RESET(), RED()) ;
    *time = 0.0;
    std::memset(obuffer, 0x0, dim_data_);
  }

  // check
  *latency_time = (*time - data_time_prev_);

  struct timespec flush_ts;
  clock_gettime(CLOCK_MONOTONIC, &flush_ts);
  double flush_time = realtime_utilities::timer_to_s(&flush_ts);
  if ((access_mode_ == OPEN) && (std::fabs(flush_time - *time) > 2 * watchdog_))
  {
    printf("Data not updated! (%f,%f,%f)!", flush_time, *time, watchdog_);
    return coe_master::IPC::WATCHDOG;
  }

  if (shmem.header_.bond_flag_ == 1)
  {
    /////////////////////////////////////////////////
    if ((*latency_time < watchdog_) &&
        (*latency_time > 1e-5)) // the client cycle time is in the acceptable trange watchdog
    {
      start_watchdog_time_ = -1;
    }
    else if (*latency_time > watchdog_)
    {
      printf("Latency overcome the watchdog (%f,%f,%f)!", *latency_time, *time, data_time_prev_);
      ret = coe_master::IPC::WATCHDOG;
    }
    else if (*latency_time < 1e-5) // the client is not writing new data in the shared memory ...
    {
      if (start_watchdog_time_ == -1)
      {
        start_watchdog_time_ = flush_time;
      }
      else if ((flush_time - start_watchdog_time_) > watchdog_)
      {
        ret = coe_master::IPC::WATCHDOG;
      }
    }
    /////////////////////////////////////////////////

    /////////////////////////////////////////////////
    if (ret == coe_master::IPC::WATCHDOG)
    {
      if (shmem.header_.rt_flag_ == 1)
      {
        PRINTF_THROTTLE(2, "[ %s%s%s%s ] Watchdog %fms (allowed: %f) ****** RESET CMD FOR SAFETTY ****",
                            BOLDCYAN(), name_.c_str(), RESET(), RED(), (flush_time - start_watchdog_time_), watchdog_);
        if (access_mode_ == CREATE)
        {
          std::memset(obuffer, 0x0, dim_data_);
        }
      }
      else
      {
        PRINTF_THROTTLE(2, "[ %s%s%s%s ] Watchdog %fms (allowed: %f) ****** SOFT RT, DON'T CARE ****", BOLDCYAN(),
                  name_.c_str(), RESET(), YELLOW(), (flush_time - start_watchdog_time_), watchdog_);
        ret = coe_master::IPC::NONE_ERROR;
      }
    }
    /////////////////////////////////////////////////
  }
  data_time_prev_ = *time;
  flush_time_prev_ = flush_time;

  return ret;
}

std::string IPC::to_string(IPC::ErrorCode err)
{
  std::string ret = "na";
  switch (err)
  {
  case NONE_ERROR:
    ret = "SHARED MEMORY NONE ERROR";
    break;
  case UNMACTHED_DATA_DIMENSION:
    ret = "SHARED MEMORY UNMATHCED DATA DIMENSION";
    break;
  case UNCORRECT_CALL:
    ret = "SHARED MEMORY UNCORRECT CALL SEQUENCE";
    break;
  case WATCHDOG:
    ret = "SHARED MEMORY WATCHDOG";
    break;
  }
  return ret;
}

std::size_t IPC::getSize(bool prepend_header) const { return prepend_header ? dim_with_header_ : dim_data_; }

double IPC::getWatchdog() const { return watchdog_; }

}