#ifndef COE_MASTER__COE_MASTER_H
#define COE_MASTER__COE_MASTER_H

#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <iostream>

#include <boost/thread.hpp>
#include <soem/ethercat.h>
#include <soem/ethercatdc.h>
#include <soem/ethercattype.h>


#include <coe_core/coe_sdo.h>
#include <coe_core/coe_utilities.h>
#include <coe_soem_utilities/coe_soem_utilities.h>
#include <realtime_utilities/circular_buffer_stamped.h>

#include <coe_master/ipc/coe_ipc.h>
#include <coe_master/ipc/coe_pdo_ipc.h>

#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_network_descriptor.h>
#include <coe_master/modules/coe_srv_utilities.h>

#include <coe_master/coe_master_fsm.h>
#include <coe_master/coe_master_utilities.h>

static const std::size_t PRE_ALLOCATION_SIZE = 1024 * 1024 * 1024;
static const std::size_t MY_STACK_SIZE = 1024 * 1024;
static const std::string COE_DEVICE_PARAMETER_NAMESPACE = "coe";

inline std::ostream &operator<<(std::ostream &stream, const ec_errort err)
{
  stream << coe_soem_utilities::to_string(err);
  return stream;
}

namespace coe_master
{

  int moduleSetup(uint16 slave);

  class CoeMaster : public coe_master::DriverFSM
  {
public:
    typedef realtime_utilities::circ_buffer_stamped_named<std::string> MasterDiagnostic;
    typedef std::shared_ptr<MasterDiagnostic> MasterDiagnosticPtr;

    typedef realtime_utilities::circ_buffer_stamped_named<ec_errort> ModuleDiagnostic;
    typedef std::shared_ptr<ModuleDiagnostic> ModuleDiagnosticPtr;

private:
    friend int moduleSetup(uint16 slave);

    ros::NodeHandle nh_;

    // ---
    bool network_info_;
    coe_master::NetworkDescriptorPtr network_;
    std::vector<coe_master::ModuleDescriptorPtr> modules_;
    coe_master::ModulesIPC module_shmem_;
    // ---

    // ---
    std::string connect_fail_;
    MasterDiagnosticPtr master_diagnostic_;
    std::map<int, ModuleDiagnosticPtr> module_diagnostic_;
    // ---

    std::shared_ptr<MainThreadSharedData> data_;

    bool soem_configured_;
    boost::shared_ptr<boost::thread> coe_main_thread_;

    std::map<int, coe_soem_utilities::PO2SOconfigFcn> config_fcn_;

    void coeMainThread();
    bool setPO2SOcallback(coe_soem_utilities::PO2SOconfigFcn setupFcn);

public:
    CoeMaster();
    ~CoeMaster();

    void doOpen();
    void doClose();
    void doStart();
    void doStop();

    std::string getID();

    // ACCESSORS
    const std::shared_ptr<MainThreadSharedData> &getData();

    const std::vector<coe_master::ModuleDescriptorPtr> &getModules() const;
    std::vector<coe_master::ModuleDescriptorPtr> &getModules();

    const coe_master::ModuleDescriptorPtr getModule(int addr) const;
    coe_master::ModuleDescriptorPtr getModule(int addr);

    const coe_master::NetworkDescriptorPtr &getNetworkDescriptor() const;
    coe_master::NetworkDescriptorPtr getNetworkDescriptor();

    const std::map<int, std::string> getAddressUniqueIdMap() const;

    const ModuleDiagnosticPtr &getCoeModuleTypedErrors(int addr) const;
    ModuleDiagnosticPtr &getCoeModuleTypedErrors(int addr);

    const std::map<int, ModuleDiagnosticPtr> &getCoeDriverTypedErrors() const;
    std::map<int, ModuleDiagnosticPtr> &getCoeDriverTypedErrors();

    const MasterDiagnosticPtr &getCoeDriverGenericErrors() const;
    MasterDiagnosticPtr &getCoeDriverGenericErrors();

    bool resetErrors();
  };

}

#endif  // COE_MASTER__COE_MASTER_H
