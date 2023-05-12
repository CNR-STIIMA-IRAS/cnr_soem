#include <array>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>

#include <boost/asio.hpp>

#include <ethercat.h>
#include <ethercattype.h>

#include <coe_master/ipc/sdo_ipc.h>
#include <coe_master/modules/utilities/common.h>

namespace coe_master
{

template <typename R>
bool syncSdo(const int addr,
             const uint8_t& sdotype,
             const uint16_t& index,
             const uint8_t& subindex,
             const double& /*timeout_ms*/,
             const bool& read_sdo,
             uint8_t* value,
             std::string& what,
             bool fake_ec)
{
  size_t ll = __LINE__;
  try
  {
    ll = __LINE__;
    if (addr < 0)
    {
      what = "The address is negative??";
      return false;
    }

    ll = __LINE__;
    int nominal_dim = -1;
    int dim = -1;

    ll = __LINE__;
    switch (sdotype)
    {
      case R::TYPE_U8:
        dim = 1;
        break;
      case R::TYPE_U16:
        dim = 2;
        break;
      case R::TYPE_U32:
        dim = 4;
        break;
      case R::TYPE_U64:
        dim = 8;
        break;
      case R::TYPE_I8:
        dim = 1;
        break;
      case R::TYPE_I16:
        dim = 2;
        break;
      case R::TYPE_I32:
        dim = 4;
        break;
      case R::TYPE_I64:
        dim = 4;
        break;
      default:
      {
        what = "SDO Type (uint8: " + std::to_string(sdotype) + ")not recognized";
        return false;
      }
    }
    ll = __LINE__;
    nominal_dim = dim;

    int retval = -1;
    ll = __LINE__;
    if (read_sdo)
    {
      ll = __LINE__;
      uint8_t* buffer = new uint8_t[dim];

      ll = __LINE__;
      std::memset(&buffer[0], 0x0, sizeof(uint8_t) * dim);

      ll = __LINE__;
      if(fake_ec)
      {
        retval = 1;
      }
      else
      {
        retval = ec_SDOread(addr, index, (uint8_t)subindex, FALSE, &dim, buffer, EC_TIMEOUTSAFE);
      }

      ll = __LINE__;
      std::memcpy(value, buffer, dim * sizeof(uint8_t));
    }
    else
    {
      ll = __LINE__;
      uint8_t* buffer = new uint8_t[dim];

      ll = __LINE__;
      std::memset(&buffer[0], 0x0, sizeof(uint8_t) * dim);

      ll = __LINE__;
      std::memcpy(buffer, value, sizeof(uint8_t) * dim);

      ll = __LINE__;
      std::size_t max_trial = 5;
      do
      {
        ll = __LINE__;
        if(fake_ec)
        {
          retval = 1;
        }
        else
        {
          retval = ec_SDOwrite(
              addr, index, (uint8_t)subindex, dim, ((uint8_t)subindex == 0) ? TRUE : FALSE, buffer, EC_TIMEOUTRXM);
        }

        ll = __LINE__;
        if (retval > 0) break;

        ll = __LINE__;
      } while (max_trial--);
    }

    ll = __LINE__;
    if (retval <= 0)
    {
      ll = __LINE__;
      what = "ec_SDOread failed. See CoE diagnostics for further information.";
      return false;
    }

    if (nominal_dim != dim)
    {
      ll = __LINE__;
      what = "Read only " + std::to_string(nominal_dim) + " bytes, while " + std::to_string(dim) +
             " bytes were supposed to be read";
      return false;
    }
    return retval > 0;
  }
  catch (std::exception& e)
  {
    what = __PRETTY_FUNCTION__ + (":  Last executed line: " + std::to_string(ll) + ". Error: \n");
    what += e.what();
    return false;
  }
  catch (...)
  {
    what = __PRETTY_FUNCTION__ + (":  Last executed line: " + std::to_string(ll) + ". Unhandled exception ");
    return false;
  }
  return true;
}

SdoManager::SdoManager(const std::map<int, std::string>& module_addresses_map,
                       bool force_sdo,
                       const std::string& host,
                       short set_sdo_port,
                       short get_sdo_port)
    : module_addresses_map_(module_addresses_map), force_sdo_(force_sdo)
{
  auto ll = __LINE__;
  try
  {
    ll = __LINE__;
    std::cout << "Prepare the SET SDO server .." << std::endl;
    set_sdo_server_ = coe_master::utils::make_shared(
        io_context_,
        host,
        set_sdo_port,
        std::bind(&SdoManager::setSdo, this, std::placeholders::_1, std::placeholders::_2));

    std::cout << "Prepare the GET SDO server .." << std::endl;
    ll = __LINE__;
    get_sdo_server_ = coe_master::utils::make_shared(
        io_context_,
        host,
        get_sdo_port,
        std::bind(&SdoManager::getSdo, this, std::placeholders::_1, std::placeholders::_2));

    std::cout << "Run the context .." << std::endl;
    ll = __LINE__;
    io_context_thread_ = std::thread(
        [this]()
        {
          std::cout << "The IO CONTEXT IS READY, RUN!" << std::endl;
          io_context_.run();
          std::cout << "The IO CONTEXT HAS BEEN TERMINATED" << std::endl;
        });
  }
  catch (std::exception& e)
  {
    std::stringstream ss;
    ss << __PRETTY_FUNCTION__ << ": Last line executed: " << ll << " Exception:" << e.what() << std::endl;
    throw std::runtime_error(ss.str().c_str());
  }
}

SdoManager::~SdoManager()
{
  std::cout << "The IO CONTEXT WILL BE STOPPED" << std::endl;
  io_context_.stop();
  std::cout << "Joint the thread..." << std::endl;
  io_context_thread_.join();

  std::cout << "Destroy servers ..." << std::endl;
  set_sdo_server_.reset();
  get_sdo_server_.reset();
}

void SdoManager::setSdo(const std::string& income, std::string& outcome)
{
  coe_master::set_sdo_t::Response res;
  std::string what = "setSdo";

  if (!coe_master::validate<coe_master::set_sdo_t::Request>(income, what))
  {
    res.what = "Failed SdoManager::setSdo: " + what;
    res.success = false;
  }
  else
  {
    coe_master::set_sdo_t::Request req = coe_master::from_string<coe_master::set_sdo_t::Request>(income);

    std::string what;
    int addr = checkModuleName(module_addresses_map_, req.module_id, what);
    res.success = (addr < 0)
                      ? false
                      : syncSdo<coe_master::set_sdo_t::Request>(
                            addr, req.sdotype, req.index, req.subindex, req.timeout_ms, false, &(req.value[0]), what, true);

    if (!res.success)
    {
      res.what = "Failed SdoManager::setSdo: " + what;
    }
  }
  outcome = coe_master::to_string(res);
}

void SdoManager::getSdo(const std::string& income, std::string& outcome)
{
  coe_master::get_sdo_t::Response res;
  std::string what;

  if (!coe_master::validate<coe_master::get_sdo_t::Request>(income, what))
  {
    res.what = "Failed SdoManager::getSdo: " + what;
    res.success = false;
  }
  else
  {
    coe_master::get_sdo_t::Request req = coe_master::from_string<coe_master::get_sdo_t::Request>(income);

    std::string what;
    int addr = checkModuleName(module_addresses_map_, req.module_id, what);
    if (addr < 0)
    {
      res.what = what;
      res.success = false;
    }
    else
    {
      res.success = syncSdo<coe_master::get_sdo_t::Request>(
          addr, req.sdotype, req.index, req.subindex, req.timeout_ms, true, &(res.value[0]), res.what, true);
    }
  }
  outcome = coe_master::to_string(res);
}

}  // namespace coe_master