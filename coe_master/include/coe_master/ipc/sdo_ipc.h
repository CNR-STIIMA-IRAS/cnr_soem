#ifndef CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_IPC_SDO_IPC
#define CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_IPC_SDO_IPC

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <cnr_ipc_utilities/asio_ipc.h>
#include <coe_master/ipc/sdo_ipc_msgs.h>


namespace coe_master
{

class SdoManager : public std::enable_shared_from_this<SdoManager>
{
 public:
  SdoManager(const std::map<int, std::string>& module_addresses_map,
             bool force_sdo,
             const std::string& host,
             short set_sdo_port,
             short get_sdo_port);

  ~SdoManager();

  void setSdo(const std::string& income, std::string& outcome);
  void getSdo(const std::string& income, std::string& outcome);

 private:
  const std::map<int, std::string> module_addresses_map_;
  bool force_sdo_;

  boost::asio::io_context io_context_;
  std::thread io_context_thread_;
  std::shared_ptr<cnr::ipc::AsioServer> set_sdo_server_;
  std::shared_ptr<cnr::ipc::AsioServer> get_sdo_server_;
};

}  // namespace coe_master

#endif  // CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_IPC_SDO_IPC
