#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <iostream>

#include <ethercat.h>
#include <ethercatdc.h>
#include <ethercattype.h>

#include <coe_master/coe_master.h>
#include <coe_master/ipc/coe_ipc.h>

namespace coe_master
{

std::shared_ptr<coe_master::CoeMaster> master;

CoeMaster::CoeMaster() : soem_configured_(false) {}

CoeMaster::~CoeMaster()
{
  printf("Killing the Coe Driver.");
  if (coe_main_thread_)
  {
    coe_main_thread_->join();
    coe_main_thread_.reset();
  }
}

bool CoeMaster::setPO2SOcallback(coe_soem_utilities::PO2SOconfigFcn setupFcn)
{
  // TODO make more elegant ...
  printf("Connect the Controller to the Driver");
  master.reset(this);

  printf("Connect the PO2SO Controller Callback Functions");
  for (auto const i : network_->getAddresses())
  {
    config_fcn_[i] = setupFcn;
  }
  return true;
}

void CoeMaster::doOpen()
{
  try
  {
    network_.reset(new coe_master::NetworkDescriptor(COE_DEVICE_PARAMETER_NAMESPACE));

    if (!network_->initNetworkNames())
    {
      printf("Fail in extracting the coe configuration information from ros param server. Abort.");
      throw std::runtime_error("Fail in extracting the coe configuration information from ros param server. Abort.");
    }
    setStatusMessage("Network configuration successfully.", true);

    data_.reset(new MainThreadSharedData(500));
    data_->setCycleTime(network_->getOperationalTime());
    setStatusMessage("Diagnostic Configuration successfully.", true);

    for (auto const& module : network_->getAddressUniqueIdMap())
    {
      module_diagnostic_[module.first].reset(new ModuleDiagnostic("Module " + std::to_string(module.first), 10));
    }

    master_diagnostic_.reset(new MasterDiagnostic("Unspecialized Errors", 10));

    setPO2SOcallback(moduleSetup);

    setStatusMessage("Device opened successfully.", true);
    state_ = OPENED;
  }
  catch (std::exception& e)
  {
    printf("%s", e.what());
    doClose();
    setStatusMessagef("Exception thrown while opening Hokuyo.\n%s", e.what());
    return;
  }
}

void CoeMaster::doClose()
{
  try
  {
    setStatusMessage("Device closed successfully.", true);
  }
  catch (std::exception& e)
  {
    setStatusMessagef("Exception thrown while trying to close:\n%s", e.what());
  }
  state_ = CLOSED;  // If we can't close, we are done for anyways.
}

void CoeMaster::doStart()
{
  try
  {
    printf("Prepare RT thread.");

    boost::thread::attributes coe_main_thread_attr;

    coe_main_thread_attr.set_stack_size(PTHREAD_STACK_MIN + MY_STACK_SIZE);

    setStatusMessagef("Wating for RT Coe Loop Thread");
    coe_main_thread_.reset(new boost::thread(coe_main_thread_attr, boost::bind(&CoeMaster::coeMainThread, this)));

    state_ = RUNNING;
  }
  catch (std::exception& e)
  {
    doClose();
    setStatusMessagef("Exception thrown while starting Hokuyo.\n%s", e.what());
    connect_fail_ = e.what();
    return;
  }
}

void CoeMaster::doStop()
{
  if (state_ != RUNNING)  // RUNNING can exit asynchronously.
    return;

  state_ = OPENED;

  if (coe_main_thread_ && !coe_main_thread_->timed_join((boost::posix_time::milliseconds)10000))
  {
    printf("coe_main_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign.");
  }
  coe_main_thread_.reset();
  setStatusMessagef("Stopped.", true);
}

std::string CoeMaster::getID()
{
  std::string id = "CoE Master";
  if (id == std::string("H0000000")) return "unknown";
  return id;
}

void CoeMaster::coeMainThread()
{
  try
  {
    realtime_utilities::period_info pinfo;
    if (!realtime_utilities::rt_init_thread(
            MY_STACK_SIZE, sched_get_priority_max(SCHED_RR), SCHED_RR, &pinfo, network_->getOperationalTime() * 1e9))
    {
      printf("Failed in setting thread rt properties. Exit. ");
      std::raise(SIGINT);
      return;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    // Bringup the COE MASTER
    if (!(soem_configured_ =
              coe_soem_utilities::soem_init(network_->getAdapterName(), 10.0, network_->getAddressUniqueIdMap())))
    {
      printf("Fail in initialize the SOEM.");
      std::raise(SIGINT);
      return;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    // Initialization of the struct for the nodes management. Attention: if the nodes does not support the configuration
    // trhough SDO, they will be not fully initialized. In the case, it is necessary to update the nodes information
    // once the IOmap is avaliable
    if (!initNodes(network_, modules_))
    {
      printf("Fail in extracting the coe configuration information from ros param server. Abort.");
      return;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    // Get the IOmap and, then configuration of the modules (the transition from PO to SO is when the IOmap is
    // calculated)
    char* IOmap = coe_soem_utilities::soem_config(10.0, configDc(modules_), configSdoCa(modules_), config_fcn_);
    if (IOmap == NULL)
    {
      printf("Fail in getting the IOmap. Abort.");
      std::raise(SIGINT);
      return;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    // Get the PDO structure for all the nodes that started with the default config, furthermore, it writes the
    // configuration on the ROSPARAM SERVER
    if (!updateNodes(modules_, IOmap, true))
    {
      printf("Fail in extracting the coe configuration information from ros param server. Abort.");
      std::raise(SIGINT);
      return;
    }
    data_->setIOmap(IOmap);

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    // Shared memory creation
    printf("[%s%s%s] %sPrepare the shared memory", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW());
    for (auto const& module : modules_)
    {
      coe_master::ModuleIPCPtr shm_mem(
          new coe_master::ModuleIPC(module, network_->getOperationalTime(), coe_master::PdoIPC::CREATE));
      module_shmem_.insert(shm_mem);
    }
    printf("[%s%s%s] %sPrepare the shared memory", BOLDGREEN(), " DONE", RESET(), BOLDYELLOW());

    //----------------------------------------------------------------------------------------
    if (!coe_soem_utilities::soem_wait_for_state(EC_STATE_OPERATIONAL))
    {
      printf("Arg. Why??");
      std::raise(SIGINT);
      return;
    }
    printf("\n\n\n%s***************** Ethercat send/receive Loop. *****************%s\n\n", BOLDGREEN(), RESET());
    // ----------------------------------------------

    uint32_t missed_cycles = 0;
    int wkc = 0;
    struct timespec update_time;
    struct timespec timer_cycle_end;
    struct timespec timer_cycle_end_prev;

    int64_t time_offset = 0;

    data_->setExpectedWKC((ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC);

    clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
    pinfo.next_period.tv_nsec = ((pinfo.next_period.tv_nsec / 1000000) + 1) * 1000000; /* round to nearest ms */

    timer_cycle_end_prev = pinfo.next_period;

    ec_send_processdata();

    int cnt = 0;
    int break_cnt = 0;
    while (true)
    {
      state_t actual_state = state_;
      try
      {
        /// -------------------------------------------------------
        // Sync the cycle with the absolute timer Linux/DC
        if (ec_slave[0].hasdc)
        {
          missed_cycles = realtime_utilities::timer_inc_period(&pinfo, time_offset);
        }
        else
        {
          clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
          missed_cycles = realtime_utilities::timer_inc_period(&pinfo);
        }

        data_->setMissedCycles(missed_cycles);
        if (missed_cycles > 2)
        {
          printf("Coe Fatal Error. %u Missed Cycles in One Clock. Abort.", missed_cycles);
          break;
        }

        if (data_->getMeanMissedCycles() > 5)
        {
          printf("Coe Fatal Error. Decrease of perfromance (%u mean missed, %u max %u min in %fs). Abort.",
                 data_->getMeanMissedCycles(),
                 data_->getMaxMissedCycles(),
                 data_->getMinMissedCycles(),
                 data_->getWindowDim() * data_->getCycleTime());
          break;
        }

        realtime_utilities::timer_wait_rest_of_period(&(pinfo.next_period));
        // -----------------------------------------------------------

        /// -------------------------------------------
        /// Process the CoE
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        data_->setWkc(wkc);

        if (wkc < data_->getExpectedWKC())
        {
          cnt++;
          if (cnt > 5)
          {
            printf("WorkCouter incorrect (%d/%d). Try to recover the error", wkc, data_->getExpectedWKC());
            master_diagnostic_->push_back("Broken Communication. WorkCouter incorrect.");
            if (!coe_soem_utilities::soem_reset_to_operational_state())
            {
              printf("Coe Fatal Error. Communication Interrupted. Abort.");
              break;
            }
            clock_gettime(CLOCK_MONOTONIC, &update_time);
            double act_time = realtime_utilities::timer_to_s(&update_time);
            for (auto& module : modules_)
            {
              std::memset(ec_slave[module->getAddress()].outputs, 0x0, ec_slave[module->getAddress()].Obytes);
            }
          }
        }
        else
        {
          cnt = 0;
          clock_gettime(CLOCK_MONOTONIC, &update_time);
          double act_time = realtime_utilities::timer_to_s(&update_time);
          //-------------------------------------
          for (auto& module : modules_)
          {
            coe_master::PdoIPC::ErrorCode errcode;

            // The "ec_slave[i].inputs" are the feedback from the devices
            // The feebdack are written in the shared memeory, and then the status of the network_ object is update
            errcode = module_shmem_[module->getIdentifier()]->tx_pdo_.update(
                ec_slave[module->getAddress()].inputs, act_time, ec_slave[module->getAddress()].Ibytes);
            if (errcode)
            {
              master_diagnostic_->push_back("Tx Update Failed. " +
                                            module_shmem_[module->getIdentifier()]->tx_pdo_.to_string(errcode));
            }
            data_->setTxHardRT(module->getIdentifier(), module_shmem_[module->getIdentifier()]->tx_pdo_.isHardRT());
            data_->setTxBonded(module->getIdentifier(), module_shmem_[module->getIdentifier()]->tx_pdo_.isBonded());
            module->updateInputs(ec_slave[module->getAddress()].inputs, false);

            // The "ec_slave[i].outputs" are the command to the devices that someone has written in the sharedmem
            // The command are read from the shared memory, and then the status of the network_ object is update
            double tm;
            double latency;
            if (actual_state == RUNNING)
            {
              errcode = module_shmem_[module->getIdentifier()]->rx_pdo_.flush(
                  ec_slave[module->getAddress()].outputs, &tm, &latency, ec_slave[module->getAddress()].Obytes);
              if (errcode)
              {
                master_diagnostic_->push_back(module->getIdentifier() + " " +
                                              module_shmem_[module->getIdentifier()]->tx_pdo_.to_string(errcode));
              }
            }
            else
            {
              std::memset(ec_slave[module->getAddress()].outputs, 0x0, ec_slave[module->getAddress()].Obytes);
            }
            data_->setRxHardRT(module->getIdentifier(), module_shmem_[module->getIdentifier()]->rx_pdo_.isHardRT());
            data_->setRxBonded(module->getIdentifier(), module_shmem_[module->getIdentifier()]->rx_pdo_.isBonded());
            data_->setLatencyTime(latency);
            module->updateOutputs(ec_slave[module->getAddress()].outputs, false);
          }
        }

        if (ec_slave[0].hasdc)
        {
          realtime_utilities::timer_calc_sync_offset(ec_DCtime, data_->getCycleTime() * 1e9, &time_offset);
        }

        //=====================
        if (EcatError)
        {
          printf("RT Loop - EcatError: adding errors to diagnostic");
          auto errors = coe_soem_utilities::soem_errors();
          printf("RT Loop - EcatError: %zu errors added to diagnostic", errors.size());
          for (ec_errort& error : errors)
          {
            std::cout << coe_soem_utilities::to_string(error) << std::endl;
            module_diagnostic_[error.Slave]->push_back(error);
          }
          for (auto& module : modules_)
          {
            std::memset(ec_slave[module->getAddress()].outputs, 0x0, ec_slave[module->getAddress()].Obytes);
          }
        }
        //=====================

        ec_send_processdata();

        clock_gettime(CLOCK_MONOTONIC, &timer_cycle_end);
        data_->setCalcTime(realtime_utilities::timer_difference_s(&timer_cycle_end, &(pinfo.next_period)));
        data_->setActualCycleTime(realtime_utilities::timer_difference_s(&timer_cycle_end, &timer_cycle_end_prev));
        timer_cycle_end_prev = timer_cycle_end;

        if (actual_state != RUNNING)
        {
          if (break_cnt++ > 10)
          {
            printf("RT Loop - Controlled exit.");
            break;
          }
        }
      }
      catch (std::exception& e)
      {
        printf("Exception thrown while trying to get scan.\n%s", e.what());
        doClose();
        return;
      }
    }
    //----------------------------------------------------------------------------------------

    printf("%sRequest init state for all slaves.%s", BOLDCYAN(), RESET());
    if (!coe_soem_utilities::soem_wait_for_state(EC_STATE_INIT))
    {
      printf("Arg. Why??");
      std::raise(SIGINT);
      return;
    }
    //    ec_slave[0].state = EC_STATE_INIT;
    //    ec_writestate(0);/* request INIT state for all slaves */
    //    ec_close();

    printf("\n\n\n%s***************** Ethercat Communication Closed. *****************%s\n\n", BOLDMAGENTA(), RESET());
    state_ = OPENED;
  }
  catch (std::exception& e)
  {
    printf("%s", e.what());
    printf("Abort.");
    return;
  }
  catch (...)
  {
    printf("Unhandled exception ");
    printf("Abort.");
    return;
  }
}

const std::shared_ptr<MainThreadSharedData>& CoeMaster::getData() { return data_; }

const std::vector<coe_master::ModuleDescriptorPtr>& CoeMaster::getModules() const { return modules_; }

std::vector<coe_master::ModuleDescriptorPtr>& CoeMaster::getModules() { return modules_; }
const coe_master::NetworkDescriptorPtr& CoeMaster::getNetworkDescriptor() const { return network_; }
coe_master::NetworkDescriptorPtr CoeMaster::getNetworkDescriptor() { return network_; }
const std::map<int, std::string> CoeMaster::getAddressUniqueIdMap() const { return network_->getAddressUniqueIdMap(); }
const std::map<int, CoeMaster::ModuleDiagnosticPtr>& CoeMaster::getCoeDriverTypedErrors() const
{
  return module_diagnostic_;
}
std::map<int, CoeMaster::ModuleDiagnosticPtr>& CoeMaster::getCoeDriverTypedErrors() { return module_diagnostic_; }
const CoeMaster::MasterDiagnosticPtr& CoeMaster::getCoeDriverGenericErrors() const { return master_diagnostic_; }
CoeMaster::MasterDiagnosticPtr& CoeMaster::getCoeDriverGenericErrors() { return master_diagnostic_; }

const coe_master::ModuleDescriptorPtr CoeMaster::getModule(int addr) const
{
  auto const it = std::find_if(modules_.begin(),
                               modules_.end(),
                               [&addr](const coe_master::ModuleDescriptorPtr m) { return m->getAddress() == addr; });
  if (it == modules_.end())
    throw std::runtime_error(std::string("address " + std::to_string(addr) + "not mapped").c_str());
  return *it;
}

coe_master::ModuleDescriptorPtr CoeMaster::getModule(int addr)
{
  auto it = std::find_if(
      modules_.begin(), modules_.end(), [&addr](coe_master::ModuleDescriptorPtr m) { return m->getAddress() == addr; });
  if (it == modules_.end())
    throw std::runtime_error(std::string("address " + std::to_string(addr) + "not mapped").c_str());
  return *it;
}

const coe_master::CoeMaster::ModuleDiagnosticPtr& CoeMaster::getCoeModuleTypedErrors(int address) const
{
  auto it = module_diagnostic_.find(address);
  if (it != module_diagnostic_.end()) return module_diagnostic_.at(address);

  throw std::runtime_error("Address not mapped");
}

CoeMaster::ModuleDiagnosticPtr& CoeMaster::getCoeModuleTypedErrors(int address)
{
  auto it = module_diagnostic_.find(address);
  if (it != module_diagnostic_.end()) return module_diagnostic_.at(address);

  throw std::runtime_error("Address not mapped");
}

bool CoeMaster::resetErrors()
{
  for (auto& e : module_diagnostic_)
  {
    e.second->clear();
  }

  master_diagnostic_->clear();

  return true;
}

int moduleSetup(uint16 slave)
{
  // Check Configuration -----------------------------------------------------
  if (!master->network_->checkAddress(slave))
  {
    printf("[ %sSetup Slave %d# %s%s ] The slave is among the not mapped nodes", BOLDCYAN(), slave, RESET(), YELLOW());
    return -1;
  }
  auto const module = master->getModule(slave);

  std::string msg =
      (BOLDBLUE() + std::string("Module Setup ") + RESET()) +
      std::string("[ " + (BOLDCYAN() + std::to_string(slave) + "# " + module->getIdentifier() + RESET()) + " ]");

  std::cout <<  "[" << BOLDMAGENTA() << "START" << RESET() << "] " << msg );
  // Check Configuration -----------------------------------------------------

  // Check Default Configuration -----------------------------------------------------
  std::cout <<  "[-----] " << msg << " Default configuration? "<< BOLDCYAN() << (module->isDefaultConfig() ? "YES" : "NO" ) << RESET());
  if (module->isDefaultConfig())
  {
    std::cout <<  "[ " << BOLDGREEN() << "DONE"<< RESET()<<"] "<< RESET() << msg  );
    return 1;
  }
  // Check Default Configuration -----------------------------------------------------

  auto& error_diagnostic = master->getCoeDriverTypedErrors();

  // PDO Assignement Configuration (through SDO) ------------------------------------------------------
  uint16_t ect_sdo_assign[2] = {ECT_SDO_TXPDOASSIGN, ECT_SDO_RXPDOASSIGN};
  for (std::size_t i = 0; i < 2; i++)
  {
    coe_core::Pdo& pdo = ect_sdo_assign[i] == ECT_SDO_TXPDOASSIGN ? module->getTxPdo() : module->getRxPdo();

    std::size_t n_pdo_entries = pdo.nEntries();  // not grouped. i.e., pdo not grouped. i.e., 1A00:1, 1A00:2, 1A00:3 is
                                                 // one Can Dictionary Object composed by 3 Can Objects
    std::vector<coe_core::DataObjectPtr> assign_pdo = coe_core::split(pdo);
    std::size_t n_pdo_grouped_entries =
        assign_pdo.size();  // grouped. i.e., pdo not grouped. i.e., 1A00:1, 1A00:2, 1A00:3 is one Can Dictionary Object
                            // composed by 3 Can Objects
    std::cout << "[-----] " << msg << (ect_sdo_assign[i] == ECT_SDO_TXPDOASSIGN ? " TxPDO" : " RxPDO") << " Write " << n_pdo_entries << "pdo entries, and "<< n_pdo_grouped_entries  << "grouped entries" << RESET() );
    if (n_pdo_grouped_entries > 0)
    {
      uint16_t map_1c1x[n_pdo_grouped_entries + 1];  //
      map_1c1x[0] = n_pdo_grouped_entries;

      for (std::size_t j = 0; j < n_pdo_grouped_entries; j++)
      {
        map_1c1x[j + 1] = assign_pdo.at(j)->index();
      }
      int retval = ec_SDOwrite(slave, ect_sdo_assign[i], 0x00, TRUE, sizeof(map_1c1x), &map_1c1x, EC_TIMEOUTSAFE);

      if (retval < 1)
      {
        while (EcatError)
        {
          printf("EcatError: adding errors to the Diagnostic queue");

          auto errors = coe_soem_utilities::soem_errors();
          for (ec_errort& error : errors)
          {
            std::cout << coe_soem_utilities::to_string(error) << std::endl;
            error_diagnostic[error.Slave]->push_back(error);
          }
        }
        std::cout <<  "[ " << RED() << " ERROR " << RESET() << " ] " << msg << " Failed in settings the PDOs. See CoE diagnostics for further information." );
      }
    }
  }
  // PDO Assignement Configuration (through SDO) ------------------------------------------------------

  // Driver Configuration thorugh SDO ------------------------------------------------------
  {
    const coe_core::Sdo& conf_sdo = module->getConfigurationSdo();
    std::size_t n_sdo_entries = conf_sdo.nEntries();

    std::cout <<  "[-----] " << msg << " SDO " << n_sdo_entries << "sdo entries" << RESET() );
    for (auto const& cob : conf_sdo)
    {
      int retval = -1;
      if (conf_sdo.write_access.at(cob->address()))
      {
        std::cout <<  "[-----] " << msg << " Write SDO:  " << cob->to_string( ) );
        retval = ec_SDOwrite(slave,
                             cob->index(),
                             cob->subindex(),
                             FALSE,
                             sizeof(uint16_t) * cob->sizeBytes(),
                             cob->data(),
                             EC_TIMEOUTSAFE);
        std::cout <<  "[-----] " << msg << " Write SDO:  retval: " << retval );
        int dim = sizeof(uint16_t) * cob->sizeBytes();
        uint32_t data;
        retval = ec_SDOread(slave, cob->index(), cob->subindex(), FALSE, &dim, &data, EC_TIMEOUTSAFE);
        std::cout <<  "[-----] " << msg << " Check:  "
                       << "index: "<< coe_core::to_string_hex( cob->index() ) <<":" << coe_core::to_string_hex( cob->subindex() ) << "val: "  << data );
      }
      else
      {
        printf("%s COB 0x%x, size: %zu, %s",
               msg.c_str(),
               cob->index(),
               cob->sizeBytes(),
               (conf_sdo.write_access.at(cob->address()) ? "WRITE" : "READ"));
        int dim = sizeof(uint16_t) * cob->sizeBytes();
        retval = ec_SDOread(slave, cob->index(), cob->subindex(), FALSE, &dim, cob->data(), EC_TIMEOUTSAFE);
      }
      if (retval < 1)
      {
        while (EcatError)
        {
          printf("EcatError: adding errors to the Diagnostic queue");
          auto errors = coe_soem_utilities::soem_errors();
          for (ec_errort& error : errors)
          {
            std::cout << coe_soem_utilities::to_string(error) << std::endl;
            error_diagnostic[error.Slave]->push_back(error);
          }
        }
        std::cout <<  "[ " << RED() << " ERROR " << RESET() << " ]" <<  msg << " Failed in access to SDO. See CoE diagnostics for further information." );
      }
    }
  }
  // SDO Configuration ------------------------------------------------------

  std::cout <<  "[ " << BOLDGREEN() << "DONE" << RESET() << "] "  << msg );
  return 1;
}

}  // namespace coe_master
