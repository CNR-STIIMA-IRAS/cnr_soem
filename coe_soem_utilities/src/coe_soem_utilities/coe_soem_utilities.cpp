#include <boost/algorithm/string.hpp>
#include <chrono>
#include <inttypes.h>
#include <iostream>
#include <regex>
#include <thread>

#include <ethercat.h>
#include <ethercatdc.h>
#include <ethercattype.h>

#include <coe_core/coe_string_utilities.h>
#include <coe_soem_utilities/coe_soem_utilities.h>

namespace coe_soem_utilities
{

  const char *RESET() { return "\033[0m"; }
  const char *BLACK() { return "\033[30m"; }
  const char *RED() { return "\033[31m"; }
  const char *GREEN() { return "\033[32m"; }
  const char *YELLOW() { return "\033[33m"; }
  const char *BLUE() { return "\033[34m"; }
  const char *MAGENTA() { return "\033[35m"; }
  const char *CYAN() { return "\033[36m"; }
  const char *WHITE() { return "\033[37m"; }
  const char *BOLDBLACK() { return "\033[1m\033[30m"; }
  const char *BOLDRED() { return "\033[1m\033[31m"; }
  const char *BOLDGREEN() { return "\033[1m\033[32m"; }
  const char *BOLDYELLOW() { return "\033[1m\033[33m"; }
  const char *BOLDBLUE() { return "\033[1m\033[34m"; }
  const char *BOLDMAGENTA() { return "\033[1m\033[35m"; }
  const char *BOLDCYAN() { return "\033[1m\033[36m"; }
  const char *BOLDWHITE() { return "\033[1m\033[37m"; }

  const char *RST() { return RESET(); }
  const char *BLK() { return BLACK(); }
  const char *R() { return RED(); }
  const char *G() { return GREEN(); }
  const char *Y() { return YELLOW(); }
  const char *BLE() { return BLUE(); }
  const char *M() { return MAGENTA(); }
  const char *C() { return CYAN(); }
  const char *W() { return WHITE(); }
  const char *BBLK() { return BOLDBLACK(); }
  const char *BR() { return BOLDRED(); }
  const char *BG() { return BOLDGREEN(); }
  const char *BY() { return BOLDYELLOW(); }
  const char *BBLE() { return BOLDBLUE(); }
  const char *BM() { return BOLDMAGENTA(); }
  const char *BC() { return BOLDCYAN(); }
  const char *BW() { return BOLDWHITE(); }

  bool ec_statecheck(uint16 slave, uint16 reqstate, double timeout_s)
  {
    uint16 ret = ::ec_statecheck(slave, reqstate, int(timeout_s * 1e6));
    if (reqstate != ret)
    {
      printf("Timeout elasped. Abort. Actual state: '%s', Requested %s, ret '%s' ",
             coe_core::to_string((ec_state)(ec_slave[slave].state)).c_str(),
             coe_core::to_string((ec_state)(reqstate)).c_str(), coe_core::to_string((ec_state)(ret)).c_str());
      return false;
    }

    return true;
  }

  uint32 soem_check_network_configuration(const std::map<int, std::string> ordered_list_of_devices)
  {
    printf("[%sSTART%s] %s[ Names ]%s %sCheck Coherence between yaml and network ", BOLDMAGENTA(), RESET(), BOLDBLUE(),
           RESET(), BOLDYELLOW());
    printf("[-----] SOEM got %d slaves, while %zu were expected. ", ec_slavecount, ordered_list_of_devices.size());
    /* Do we got expected number of slaves from config */
    if ((std::size_t)ec_slavecount < ordered_list_of_devices.size())
    {
      printf("[-----] SOEM got %d slaves, while %zu were expected. Abort. ", ec_slavecount,
             ordered_list_of_devices.size());
      return 0;
    }

    std::vector<bool> ok(ordered_list_of_devices.size(), false);
    for (int i = 1; i <= ec_slavecount; i++)
    {
      std::string device_name_from_coe = ec_slave[i].name;
      bool warn = true;
      std::string info = (BOLDBLUE() + std::string("[ Names ] ") + RESET()) +
                         (CYAN() + std::string("[ ") + std::to_string(i) + std::string("# node ID ] ") + RESET()) +
                         "SOEM: '" + std::string(BOLDCYAN() + device_name_from_coe + RESET()) + "', ";

      if (ordered_list_of_devices.find(i) != ordered_list_of_devices.end())
      {
        std::string device_name_from_rosparam = ordered_list_of_devices.at(i);
        info += " ROSPARAM: '" + std::string(BOLDCYAN() + device_name_from_rosparam + RESET()) + "'  ";

        std::regex allowed_chars_in_name("^[A-Za-z][\w_]*$");

        if (!std::regex_match(device_name_from_coe, allowed_chars_in_name))
        {
          device_name_from_coe = std::regex_replace(device_name_from_coe, std::regex(R"([^A-Za-z\d])"), "");
        }
        boost::to_lower(device_name_from_coe);

        boost::to_lower(device_name_from_rosparam);

        if (device_name_from_coe == device_name_from_rosparam)
          warn = false;

        ok[std::distance(ordered_list_of_devices.begin(), ordered_list_of_devices.find(i))] = true;
      }
      else
        info += " ROSPARAM: '" + std::string(BOLDRED()) + "NOT MAPPED" + std::string(RESET()) + "'  ";

      if (warn)
        printf("[%sCHECK%s%s] %s", BOLDRED(), RESET(), YELLOW(), info.c_str());
      else
        printf("[-----] %s", info.c_str());
    }

    for (const bool &b : ok)
    {
      if (!b)
      {
        printf("Some modules of the configuration are missing. Abort.");
        return false;
      }
    }

    printf("[%s%s%s] %s[ Names ]%s %s-----------------------------------------", BOLDGREEN(), "  OK ", RESET(),
           BOLDBLUE(), RESET(), BOLDYELLOW());

    return 1;
  }

  bool soem_init(const std::string &adapter_name, const double timeout_s)
  {
    printf("[%s%s%s] %sConfig the SOEM ", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW());

    if (!ec_init(adapter_name.c_str()))
    {
      printf("Impossible to init the coe (adapter: '%s') ", adapter_name.c_str());
      return false;
    }

    auto t0 = std::chrono::steady_clock::now();
    while (EcatError)
    {
      printf("%s", ec_elist2string());
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
      {
        printf("Timeout elasped. Abort ");
        ec_close();
        return false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (ec_config_init(FALSE) <= 0)
    {
      printf("Impossible to config init. No slaves found! Abort.");
      ec_close();
      return false;
    }

    printf("[%s%s%s] %sConfig the SOEM ", BOLDGREEN(), "  OK ", RESET(), BOLDYELLOW());

    return true;
  }

  bool soem_init(const std::string &adapter_name, const double timeout_s,
                 const std::map<int, std::string> ordered_list_of_devices)
  {

    if (!soem_init(adapter_name, timeout_s))
    {
      printf("SOEM Init failed. Abort.");
      return false;
    }

    if (!soem_check_network_configuration(ordered_list_of_devices))
    {
      printf("The network is different form the expected one. Abort.");
      return false;
    }

    return true;
  }

  char *soem_config(const double timeout_s, bool support_dc, const std::map<int, bool> &sdo_complete_access,
                    const std::map<int, PO2SOconfigFcn> &config_fcn)
  {
    printf("[%s%s%s] %sConfig the modules", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW());

    // CompleteAccess -------------------------
    for (int iSlave = 1; iSlave <= ec_slavecount; iSlave++)
    {
      bool enabled_sdoca =
          (sdo_complete_access.find(iSlave) != sdo_complete_access.end()) ? (sdo_complete_access.at(iSlave)) : false;

      printf("[-----] %sSDO CA%s [ %s%s%s ]  SOEM '%s', ROSPARAM '%s'", BOLDBLUE(), RESET(), BOLDCYAN(),
             (std::to_string(iSlave) + "# " + ec_slave[iSlave].name).c_str(), RESET(),
             (ec_slave[iSlave].CoEdetails & ECT_COEDET_SDOCA ? BOLDGREEN() + std::string("SUPPORTED") + RESET()
                                                             : BOLDRED() + std::string("NOT SUPPORTED") + RESET())
                 .c_str(),
             (!enabled_sdoca ? BOLDYELLOW() + std::string("FORCE DISABLING") + RESET()
                             : BOLDRED() + std::string("DEFAULT") + RESET())
                 .c_str());

      if ((ec_slave[iSlave].CoEdetails & ECT_COEDET_SDOCA) && (!enabled_sdoca))
        ec_slave[iSlave].CoEdetails ^= ECT_COEDET_SDOCA;
    }
    // CompleteAccess -------------------------

    // DC Config ------------------------------
    printf("[-----] %sDC SUPPORT%s [ %sALL%s ] Forced to %s", BOLDBLUE(), RESET(), BOLDCYAN(), RESET(),
           (support_dc ? std::string(YELLOW() + std::string("ENABLE") + RESET()).c_str()
                       : std::string(BOLDRED() + std::string("NOT ENABLE") + RESET()).c_str()));
    if (support_dc)
    {
      bool ok = ec_configdc();
      printf(
          "[-----] %sDC SUPPORT%s [ %sALL%s ] SOEM '%s', ROSPARAM '%s'", BOLDBLUE(), RESET(), BOLDCYAN(), RESET(),
          (ok ? BOLDGREEN() + std::string("SUPPORTED") + RESET() : BOLDRED() + std::string("NOT SUPPORTED") + RESET())
              .c_str(),
          (BOLDGREEN() + std::string("DEFAULT") + RESET()).c_str());
    }
    // DC Config -------------------------------

    // PS 2 OP ---------------------------------
    for (auto const &setup : config_fcn)
    {
      if (setup.second != NULL)
      {
        ec_slave[setup.first].PO2SOconfig = setup.second;
        (*setup.second)(setup.first);
      }
    }
    // PS 2 OP ---------------------------------

    // IO Map ---------------------------------
    printf("[-----] %sIOmap Set %s [ %sALL%s ]", BOLDBLUE(), RESET(), BOLDCYAN(), RESET());
    static char IOmap[4096];
    memset(&IOmap[0], 0x0, sizeof(char) * 4096);

    if (ec_config_map(&IOmap) <= 0)
    {
      printf("[-----] Impossible to get the IOmap");
      ec_close();
      return NULL;
    }
    // IO Map ---------------------------------

    auto t0 = std::chrono::steady_clock::now();
    while (EcatError)
    {
      printf("%s", ec_elist2string());
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
      {
        printf("[-----] Timeout elasped. Abort ");
        ec_close();
        return NULL;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("[-----] Command the state transition to SAFE_OP to all the slaves [%s%s%s]", BOLDMAGENTA(), "IN PROGRESS",
           RESET());
    if (!ec_statecheck(0, EC_STATE_SAFE_OP, 10.0))
    {
      bool ok = true;
      for (int slave = 1; slave <= ec_slavecount; slave++)
      {
        ok &= ec_statecheck(slave, EC_STATE_SAFE_OP, 2.0);
      }
      if (!ok)
      {
        while (EcatError)
        {
          printf("%s", ec_elist2string());
          if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
          {
            printf("Timeout elasped. Abort ");
            ec_close();
            return NULL;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ec_close();
      }
      return NULL;
    }
    printf("[-----] Command the state transition to SAFE_OP to all the slaves [%s%s%s]", BOLDGREEN(), " OK ", RESET());

    printf("[%s%s%s] %sConfig the modules", BOLDGREEN(), "  OK ", RESET(), BOLDYELLOW());

    return IOmap;
  }

  bool soem_wait_for_state(const ec_state &target_state)
  {

    int MASTER_ID = 0;
    if (target_state == EC_STATE_OPERATIONAL)
    {
      // ----------------------------------------------
      ec_slave[MASTER_ID].state = EC_STATE_OPERATIONAL;

      ec_send_processdata(); // send one valid process data to make outputs in slaves happy
      ec_receive_processdata(EC_TIMEOUTRET);
#if NO_STADE
      ec_writestate(0); // request OP state for all slaves
#else
      if (EC_NOFRAME == ec_writestate(MASTER_ID))
      {
        printf("Error in writing state to all the slave ...");
        return false;
      }
      ec_statecheck(MASTER_ID, target_state, EC_TIMEOUTSTATE * 3);
#endif
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(MASTER_ID, EC_STATE_OPERATIONAL, 50000);
      } while (ec_slave[MASTER_ID].state != EC_STATE_OPERATIONAL);
    }
    else
    {
      ec_slave[MASTER_ID].state = EC_STATE_INIT;
      //
      if (EC_NOFRAME == ec_writestate(MASTER_ID))
      {
        printf("Error in writing state to all the slave ...");
        return false;
      }

      ec_statecheck(MASTER_ID, target_state, EC_TIMEOUTSTATE * 3);
      if (ec_slave[0].state != target_state)
      {
        printf("Not all slaves reached '%s' state.", coe_core::to_string(target_state).c_str());
        for (int i = 1; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != target_state)
          {
            printf("Slave %d State: %s (StatusCode: %4x : %s)", i,
                   coe_core::to_string(ec_state(ec_slave[i].state)).c_str(), ec_slave[i].ALstatuscode,
                   ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
        return false;
      }
      ec_readstate();
    }

    return true;
  }

  bool soem_reset_to_operational_state()
  {
    printf("Reset to OP state.");
    ec_group[0].docheckstate = FALSE;
    ec_readstate();
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
      printf("%d # %s slave is in %s state.", slave, std::string(ec_slave[slave].name).c_str(),
             ::coe_core::to_string((ec_state)ec_slave[slave].state).c_str());
      if (ec_slave[slave].state != EC_STATE_OPERATIONAL)
      {
        ec_group[0].docheckstate = TRUE;
        if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
        {
          printf("Slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
          ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
          ec_writestate(slave);
        }
        else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
        {
          printf("Slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
          ec_slave[slave].state = EC_STATE_OPERATIONAL;
          ec_writestate(slave);
        }
        else if (ec_slave[slave].state > EC_STATE_NONE)
        {
          if (ec_reconfig_slave(slave, 500))
          {
            ec_slave[slave].islost = FALSE;
            printf("slave %d reconfigured, state %s", slave,
                   coe_core::to_string((ec_state)(ec_slave[slave].state)).c_str());
          }
        }
        else if (!ec_slave[slave].islost)
        {
          /* re-check state */
          ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
          if (ec_slave[slave].state == EC_STATE_NONE)
          {
            ec_slave[slave].islost = TRUE;
            printf("ERROR : slave %d lost", slave);
          }
        }
      }
      if (ec_slave[slave].islost)
      {
        if (ec_slave[slave].state == EC_STATE_NONE)
        {
          if (ec_recover_slave(slave, 500))
          {
            ec_slave[slave].islost = FALSE;
            printf("MESSAGE : slave %d recovered", slave);
          }
        }
        else
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d found", slave);
        }
      }
    }

    if (!ec_group[0].docheckstate)
    {
      printf("OK : all slaves resumed OPERATIONAL.");
      return true;
    }
    return false;
  }

  std::vector<ec_errort> soem_errors()
  {
    std::vector<ec_errort> ret;
    ec_errort Ec;

    while (ecx_poperror(&ecx_context, &Ec))
    {
      ret.push_back(Ec);
    }
    return ret;
  }

  std::string to_string(ec_errort Ec)
  {
    char ret[1024] = {0};
    char timestr[20];

    sprintf(timestr, "Time:%12.3f", Ec.Time.sec + (Ec.Time.usec / 1000000.0));
    switch (Ec.Etype)
    {
    case EC_ERR_TYPE_SDO_ERROR:
    {
      sprintf(ret, "%s SDO slave:%d index:%4.4x.%2.2x error:%8.8x %s\n", timestr, Ec.Slave, Ec.Index, Ec.SubIdx,
              (unsigned)Ec.AbortCode, ec_sdoerror2string(Ec.AbortCode));
      break;
    }
    case EC_ERR_TYPE_EMERGENCY:
    {
      sprintf(ret, "%s EMERGENCY slave:%d error:%4.4x\n", timestr, Ec.Slave, Ec.ErrorCode);
      break;
    }
    case EC_ERR_TYPE_PACKET_ERROR:
    {
      sprintf(ret, "%s PACKET slave:%d index:%4.4x.%2.2x error:%d\n", timestr, Ec.Slave, Ec.Index, Ec.SubIdx,
              Ec.ErrorCode);
      break;
    }
    case EC_ERR_TYPE_SDOINFO_ERROR:
    {
      sprintf(ret, "%s SDO slave:%d index:%4.4x.%2.2x error:%8.8x %s\n", timestr, Ec.Slave, Ec.Index, Ec.SubIdx,
              (unsigned)Ec.AbortCode, ec_sdoerror2string(Ec.AbortCode));
      break;
    }
    case EC_ERR_TYPE_SOE_ERROR:
    {
      sprintf(ret, "%s SoE slave:%d IDN:%4.4x error:%4.4x %s\n", timestr, Ec.Slave, Ec.Index, (unsigned)Ec.AbortCode,
              ec_soeerror2string(Ec.ErrorCode));
      break;
    }
    case EC_ERR_TYPE_MBX_ERROR:
    {
      sprintf(ret, "%s MBX slave:%d error:%4.4x %s\n", timestr, Ec.Slave, Ec.ErrorCode,
              ec_mbxerror2string(Ec.ErrorCode));
      break;
    }
    default:
    {
      sprintf(ret, "%s error:%8.8x\n", timestr, (unsigned)Ec.AbortCode);
      break;
    }
    }
    return std::string(ret);
  }

  std::string to_string(const ec_err_type &t)
  {
    std::string ret;
    switch (t)
    {
    case EC_ERR_TYPE_SDO_ERROR:
      ret = "SDO ERROR";
      break;
    case EC_ERR_TYPE_EMERGENCY:
      ret = "EMERGENCY";
      break;
    case EC_ERR_TYPE_PACKET_ERROR:
      ret = "PACKET ERROR";
      break;
    case EC_ERR_TYPE_SDOINFO_ERROR:
      ret = "SDOINFO ERROR";
      break;
    case EC_ERR_TYPE_FOE_ERROR:
      ret = "FOE ERROR";
      break;
    case EC_ERR_TYPE_FOE_BUF2SMALL:
      ret = "FOE BUF2SMALL";
      break;
    case EC_ERR_TYPE_FOE_PACKETNUMBER:
      ret = "FOE PACKETNUMBER";
      break;
    case EC_ERR_TYPE_SOE_ERROR:
      ret = "SOE ERRROR";
      break;
    case EC_ERR_TYPE_MBX_ERROR:
      ret = "MBX ERROR";
      break;
    case EC_ERR_TYPE_FOE_FILE_NOTFOUND:
      ret = "FOE_FILE_NOTFOUND";
      break;
    }
    return ret;
  }

  int get_node_information(const uint16_t cnt)
  {

    /////////////////////////////////////////////////////////////////////////////////////////////
    printf("Slave:%d\n", cnt);
    printf("Name:%s\n ", ec_slave[cnt].name);
    printf("Output size: %dbits\n", ec_slave[cnt].Obits);
    printf("Input  size: %dbits\n", ec_slave[cnt].Ibits);
    printf("State: %d\n", ec_slave[cnt].state);
    printf("Delay: %d[ns]\n", ec_slave[cnt].pdelay);
    printf("Has DC: %d\n", ec_slave[cnt].hasdc);
    if (ec_slave[cnt].hasdc)
      printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
    /////////////////////////////////////////////////////////////////////////////////////////////

    printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0, (ec_slave[cnt].activeports & 0x02) > 0,
           (ec_slave[cnt].activeports & 0x04) > 0, (ec_slave[cnt].activeports & 0x08) > 0);

    printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);

    printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id,
           (int)ec_slave[cnt].eep_rev);
    for (int nSM = 0; nSM < EC_MAXSM; nSM++)
    {
      if (ec_slave[cnt].SM[nSM].StartAddr > 0)
        printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM, ec_slave[cnt].SM[nSM].StartAddr,
               ec_slave[cnt].SM[nSM].SMlength, (int)ec_slave[cnt].SM[nSM].SMflags, ec_slave[cnt].SMtype[nSM]);
    }
    for (int j = 0; j < ec_slave[cnt].FMMUunused; j++)
    {
      printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
             (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
             ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
             ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
    }
    printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n", ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func,
           ec_slave[cnt].FMMU3func);
    printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl,
           ec_slave[cnt].mbx_proto);
    int ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
    /* SII general section */
    if (ssigen)
    {
      ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
      ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
      ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
      ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
      if ((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
      {
        ec_slave[cnt].blockLRW = 1;
        ec_slave[0].blockLRW++;
      }
      ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
      ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
      ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
    }
    printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n", ec_slave[cnt].CoEdetails,
           ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
    printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n", ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);

    return 1;
  }

  int get_object_description_list(uint16_t cnt, const double timeout_s)
  {
    ec_ODlistt ODlist;
    ec_OElistt OElist;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if (!ec_readODlist(cnt, &ODlist))
    {
      auto t0 = std::chrono::steady_clock::now();
      while (EcatError)
      {
        printf("%s", ec_elist2string());
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
        {
          printf("Timeout elasped. Abort ");
          return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      return -1;
    }

    printf(" CoE Object Description found, %d entries.\n", ODlist.Entries);
    for (int i = 0; i < ODlist.Entries; i++)
    {
      ec_readODdescription(i, &ODlist);

      auto t0 = std::chrono::steady_clock::now();
      while (EcatError)
      {
        printf("%s", ec_elist2string());
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
        {
          printf("Timeout elasped. Abort ");
          return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      printf(">>>>[%d/%d]>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n", i,
             ODlist.Entries);
      printf("    [Idx:sub] Type Code Length Access Name\t(Value)\n");
      printf("    [%4.4x:%2.2x]", ODlist.Index[i], 0x0);
      printf(" %4.4x ", ODlist.DataType[i]);
      printf(" %2.2x  ", ODlist.ObjectCode[i]);
      printf(" %4.4x  ", 0x0);
      printf(" %4.4x ", 0x0);
      printf(" %s\n", ODlist.Name[i]);
      printf("    ------------------------------------------------------------------------------\n");
      memset(&OElist, 0, sizeof(OElist));
      ec_readOE(i, &ODlist, &OElist);

      t0 = std::chrono::steady_clock::now();
      while (EcatError)
      {
        printf("%s", ec_elist2string());
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
        {
          printf("Timeout elasped. Abort ");
          return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      for (int j = 0; j < ODlist.MaxSub[i] + 1; j++)
      {
        if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
        {
          printf("    [%4.4x:%2.2x]", ODlist.Index[i], j);
          printf(" %4.4x ", OElist.DataType[j]);
          printf(" %2.2x  ", ODlist.ObjectCode[i]);
          printf(" %4.4x  ", OElist.BitLength[j]);
          printf(" %4.4x ", OElist.ObjAccess[j]);
          printf(" %s ", OElist.Name[j]);
          if ((OElist.ObjAccess[j] & 0x0007))
          {
            printf("(%s)", sdo2string(cnt, ODlist.Index[i], j, OElist.DataType[j]).c_str());
          }
          printf("\n");
        }
      }
      printf("<<<<[%d/%d]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n", i,
             ODlist.Entries);
    }
    return 1;
  }

  bool get_cob_via_sdo(uint16_t addr, coe_core::BaseDataObjectEntry *in)
  {
    char usdo[256] = {0};
    int l = in->sizeBytes();
    int wrc = ec_SDOread(addr, in->index(), in->subindex(), FALSE, &l, &usdo, EC_TIMEOUTRXM);
    std::memcpy(in->data(), usdo, in->sizeBytes());
    return wrc == (int)in->sizeBytes();
  }

  bool set_cob_via_sdo(uint16_t addr, const coe_core::BaseDataObjectEntry *in)
  {
    char usdo[256] = {0};
    std::memcpy(&usdo[0], in->data(), in->sizeBytes());
    int wrc = ec_SDOwrite(addr, in->index(), in->subindex(), FALSE, in->sizeBytes(), &usdo[0], EC_TIMEOUTRXM);
    return wrc == (int)in->sizeBytes();
  }

  /**
   *
   *
   *
   *
   *
   *
   *
   *
   *
   */
  bool get_pdo_map_through_sdo(coe_core::Pdo &pdo, const uint16_t &iSlave, const uint16_t &PDOassign,
                               std::size_t mapoffset, std::size_t bitoffset)
  {
    const bool push_back = pdo.nEntries() == 0;

    int obj_bitoffset = 0;
    int wkc, rdl;
    int32 rdat, rdat2;
    rdl = sizeof(rdat);
    rdat = 0;

    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(iSlave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM * 10);
    rdat = etohs(rdat);

    if ((wkc <= 0) || (rdat < 0)) // negative result from iSlave ?
    {
      printf("The module '%s%s%s' has none PDO (wkc: %d, rdat %d)", BOLDYELLOW(), ec_slave[iSlave].name, RESET(), wkc,
             rdat);
      return false;
    }
    uint16_t nidx = rdat; // number of available sub indexes

    for (int pdo_assign_subindex = 1; pdo_assign_subindex <= nidx; pdo_assign_subindex++) // read all PDO's
    {
      rdl = sizeof(rdat);
      rdat = 0;

      wkc = ec_SDOread(iSlave, PDOassign, (uint8)pdo_assign_subindex, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);

      uint16_t pdo_entry_index = etohl(rdat); // result is index of PDO: something like 1Axx, 16yy
      if (wkc <= 0)
      {
        printf("Error in reading the SDO (iSlave %#04x) addr: %#08x:%d/%d wkc %u", iSlave, pdo_entry_index,
               pdo_assign_subindex, nidx, wkc);
        return false;
      }

      rdl = sizeof(uint8_t);
      uint8_t subcnt = 0;

      wkc = ec_SDOread(iSlave, pdo_entry_index, 0x00, FALSE, &rdl, &subcnt,
                       EC_TIMEOUTRXM); // read number of subindexes of PDO
      uint16_t pdo_entry_num_subindexes = subcnt;

      for (uint16_t pdo_entry_subindex = 1; pdo_entry_subindex <= pdo_entry_num_subindexes;
           pdo_entry_subindex++) // for each subindex
      {
        rdl = sizeof(rdat2);
        rdat2 = 0;
        /* read SDO that is mapped in PDO */
        wkc = ec_SDOread(iSlave, pdo_entry_index, (uint8)pdo_entry_subindex, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
        if (wkc <= 0)
        {
          printf("Error in reading the SDO (iSlave %#04x) addr: %#08x:%d wkc %u", iSlave, pdo_entry_index,
                 pdo_entry_subindex, wkc);
          return false;
        }
        rdat2 = etohl(rdat2);
        uint8_t obj_bitlen = LO_BYTE(rdat2);
        uint16_t obj_idx = (uint16_t)(rdat2 >> 16);
        uint8_t obj_subidx = (uint8_t)((rdat2 >> 8) & 0x000000ff);

        int abs_offset = mapoffset + (obj_bitoffset / 8);
        int abs_bit = obj_bitoffset % 8;

        ec_ODlistt ODlist;
        ODlist.Slave = iSlave;
        ODlist.Index[0] = obj_idx;

        ec_OElistt OElist;
        OElist.Entries = 0;
        wkc = 0;

        if (obj_idx || obj_subidx) // the object is a filler (0x0000:0x00), it introduces a shift in the packed buffer
        {

          wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
          if ((wkc <= 0) || OElist.Entries <= 0)
          {
            printf("Error in reading the object entity properties. obj_idx %d obj_subidx %d wkc %d", obj_idx,
                   obj_subidx, wkc);
            return false;
          }

          if (push_back)
          {
            coe_core::BaseDataObjectEntryPtr obj;
            switch (OElist.DataType[obj_subidx])
            {
            case ECT_BOOLEAN:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_INTEGER8:
              obj.reset(new coe_core::DataObjectEntry<int8_t>(pdo_entry_index, pdo_entry_subindex,
                                                              std::string(OElist.Name[obj_subidx]).c_str(),
                                                              *(int8_t *)&rdat2));
              break;
            case ECT_INTEGER16:
              obj.reset(new coe_core::DataObjectEntry<int16_t>(pdo_entry_index, pdo_entry_subindex,
                                                               std::string(OElist.Name[obj_subidx]).c_str(),
                                                               *(int16_t *)&rdat2));
              break;
            case ECT_INTEGER32:
              obj.reset(new coe_core::DataObjectEntry<int32_t>(pdo_entry_index, pdo_entry_subindex,
                                                               std::string(OElist.Name[obj_subidx]).c_str(),
                                                               *(int32_t *)&rdat2));
              break;
            case ECT_INTEGER64:
              obj.reset(new coe_core::DataObjectEntry<int64_t>(pdo_entry_index, pdo_entry_subindex,
                                                               std::string(OElist.Name[obj_subidx]).c_str(),
                                                               *(int64_t *)&rdat2));
              break;
            case ECT_UNSIGNED8:
              obj.reset(new coe_core::DataObjectEntry<uint8_t>(pdo_entry_index, pdo_entry_subindex,
                                                               std::string(OElist.Name[obj_subidx]).c_str(),
                                                               *(uint8_t *)&rdat2));
              break;
            case ECT_UNSIGNED16:
              obj.reset(new coe_core::DataObjectEntry<uint16_t>(pdo_entry_index, pdo_entry_subindex,
                                                                std::string(OElist.Name[obj_subidx]).c_str(),
                                                                *(uint16_t *)&rdat2));
              break;
            case ECT_UNSIGNED32:
              obj.reset(new coe_core::DataObjectEntry<uint32_t>(pdo_entry_index, pdo_entry_subindex,
                                                                std::string(OElist.Name[obj_subidx]).c_str(),
                                                                *(uint32_t *)&rdat2));
              break;
            case ECT_UNSIGNED64:
              obj.reset(new coe_core::DataObjectEntry<uint64_t>(pdo_entry_index, pdo_entry_subindex,
                                                                std::string(OElist.Name[obj_subidx]).c_str(),
                                                                *(uint64_t *)&rdat2));
              break;
            case ECT_REAL32:
              obj.reset(new coe_core::DataObjectEntry<double>(pdo_entry_index, pdo_entry_subindex,
                                                              std::string(OElist.Name[obj_subidx]).c_str(),
                                                              *(double *)&rdat2));
              break;
            case ECT_REAL64:
              obj.reset(new coe_core::DataObjectEntry<long double>(pdo_entry_index, pdo_entry_subindex,
                                                                   std::string(OElist.Name[obj_subidx]).c_str(),
                                                                   *(long double *)&rdat2));
              break;
            case ECT_BIT1:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT2:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT3:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT4:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT5:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT6:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT7:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            case ECT_BIT8:
              obj.reset(new coe_core::DataObjectEntry<bool>(
                  pdo_entry_index, pdo_entry_subindex, std::string(OElist.Name[obj_subidx]).c_str(), *(bool *)&rdat2));
              break;
            default:
              throw std::runtime_error("Type not yet implemented.");
            }
            std::string what;
            if (!pdo.push_back(obj, what))
            {
              throw std::runtime_error(std::string("Error!! " + what).c_str());
            }
          }
          else
          {
            if (pdo.find(pdo_entry_index, pdo_entry_subindex) == pdo.end())
            {
              printf("PDO from SOEM: 0x%4.4X:0x%2.2X", pdo_entry_index, pdo_entry_subindex);
              printf("PDO from params: \n%s", pdo.to_string().c_str());
              throw std::runtime_error(("The index '" + coe_core::to_string_hex(pdo_entry_index) + ":" +
                                        coe_core::to_string_hex(pdo_entry_subindex) +
                                        "' is not in the PDO. Check the config files...")
                                           .c_str());
            }
          }

          uint32_t wd = 0;
          wd = ((uint32_t)pdo_entry_index << 16) | (uint32_t)pdo_entry_subindex;
          pdo.start_bytes_map_[wd] = abs_offset;
          pdo.start_bits_map_[wd] = abs_bit;
          pdo.size_bits_map_[wd] = obj_bitlen;
        }
        int length = 0;
        char buffer[1024] = {0};
        length += sprintf(buffer + length, " PDO 0x%4.4X:0x%2.2X -> [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X",
                          pdo_entry_index, pdo_entry_subindex, abs_offset, abs_bit, obj_idx, obj_subidx, obj_bitlen);
        if ((wkc > 0) && OElist.Entries)
        {
          length += sprintf(buffer + length, " %-12s %s\n",
                            coe_core::dtype2string(OElist.DataType[obj_subidx], true).c_str(), OElist.Name[obj_subidx]);
        }
        else
          length += sprintf(buffer + length, "\n");
        printf("%s", std::string(buffer).c_str());

        obj_bitoffset += obj_bitlen;
      }
    }
    assert((obj_bitoffset % 8) == 0);
    pdo.setPackedBytesLenght(obj_bitoffset / 8);
    pdo.finalize();

    std::size_t m = (*std::min_element(pdo.start_bytes_map_.begin(), pdo.start_bytes_map_.end(),
                                       [](std::pair<std::size_t, std::size_t> l, std::pair<std::size_t, std::size_t> r)
                                       { return l.second < r.second; }))
                        .second;
    for (auto &b : pdo.start_bytes_map_)
      b.second -= m;
    return true;
  }

  bool get_pdo_map_through_sii(coe_core::Pdo &pdo, const uint16_t &iSlave, const uint16_t &SIIcategory,
                               std::size_t mapoffset, std::size_t bitoffset)
  {

    if ((SIIcategory != 1) && (SIIcategory != 0))
    {
      printf(" The SII PDO Catergory is %x while is expected 1 or 0 ", SIIcategory);
      return false;
    }
    uint16_t totalsize = 0;
    uint16 a, w, c, Size;

    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint16 obj_bitoffset = bitoffset;
    uint8 bitlen;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;

    char str_name[EC_MAXNAME + 1];

    Size = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0; c < EC_MAXSM; c++)
      PDO->SMbitsize[c] = 0;

    PDO->Startpos = ec_siifind(iSlave, ECT_SII_PDO + SIIcategory);

    if (PDO->Startpos > 0)
    {

      a = PDO->Startpos;
      w = ec_siigetbyte(iSlave, a++);
      w += (ec_siigetbyte(iSlave, a++) << 8);
      PDO->Length = w;
      c = 1;
      /* traverse through all PDOs */
      do
      {

        PDO->nPDO++;
        PDO->Index[PDO->nPDO] = ec_siigetbyte(iSlave, a++);
        PDO->Index[PDO->nPDO] += (ec_siigetbyte(iSlave, a++) << 8);
        PDO->BitSize[PDO->nPDO] = 0;
        c++;

        /* number of entries in PDO */
        uint16 nEntries = ec_siigetbyte(iSlave, a++);
        PDO->SyncM[PDO->nPDO] = ec_siigetbyte(iSlave, a++);
        a++;
        obj_name = ec_siigetbyte(iSlave, a++);
        a += 2;
        c += 2;

        if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
        {

          str_name[0] = 0;
          if (obj_name)
            ec_siistring(str_name, iSlave, obj_name);

          /* read all entries defined in PDO */
          for (uint16 er = 1; er <= nEntries; er++)
          {

            c += 4;
            obj_idx = ec_siigetbyte(iSlave, a++);
            obj_idx += (ec_siigetbyte(iSlave, a++) << 8);
            obj_subidx = ec_siigetbyte(iSlave, a++);
            obj_name = ec_siigetbyte(iSlave, a++);
            obj_datatype = ec_siigetbyte(iSlave, a++);
            bitlen = ec_siigetbyte(iSlave, a++);
            abs_offset = mapoffset + (obj_bitoffset / 8);
            abs_bit = obj_bitoffset % 8;

            PDO->BitSize[PDO->nPDO] += bitlen;
            a += 2;

            /* skip entry if filler (0x0000:0x00) */
            if (obj_idx || obj_subidx)
            {

              str_name[0] = 0;
              if (obj_name)
                ec_siistring(str_name, iSlave, obj_name);

              long double rdat2 = 0;
              coe_core::BaseDataObjectEntryPtr obj;
              switch (obj_datatype)
              {
              case ECT_BOOLEAN:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(uint8_t *)&rdat2));
                break;
              case ECT_INTEGER8:
                obj.reset(new coe_core::DataObjectEntry<int8_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                *(int8_t *)&rdat2));
                break;
              case ECT_INTEGER16:
                obj.reset(new coe_core::DataObjectEntry<int16_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                 *(int16_t *)&rdat2));
                break;
              case ECT_INTEGER32:
                obj.reset(new coe_core::DataObjectEntry<int32_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                 *(int32_t *)&rdat2));
                break;
              case ECT_INTEGER64:
                obj.reset(new coe_core::DataObjectEntry<int64_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                 *(int64_t *)&rdat2));
                break;
              case ECT_UNSIGNED8:
                obj.reset(new coe_core::DataObjectEntry<uint8_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                 *(uint8_t *)&rdat2));
                break;
              case ECT_UNSIGNED16:
                obj.reset(new coe_core::DataObjectEntry<uint16_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                  *(uint16_t *)&rdat2));
                break;
              case ECT_UNSIGNED32:
                obj.reset(new coe_core::DataObjectEntry<uint32_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                  *(uint32_t *)&rdat2));
                break;
              case ECT_UNSIGNED64:
                obj.reset(new coe_core::DataObjectEntry<uint64_t>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                  *(uint64_t *)&rdat2));
                break;
              case ECT_REAL32:
                obj.reset(new coe_core::DataObjectEntry<double>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                *(double *)&rdat2));
                break;
              case ECT_REAL64:
                obj.reset(new coe_core::DataObjectEntry<long double>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                                     *(long double *)&rdat2));
                break;
              case ECT_BIT1:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT2:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT3:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT4:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT5:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT6:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT7:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              case ECT_BIT8:
                obj.reset(new coe_core::DataObjectEntry<bool>(obj_idx, obj_subidx, std::string(str_name).c_str(),
                                                              *(bool *)&rdat2));
                break;
              default:
                throw std::runtime_error("Type not yet implemented.");
              }

              std::string what;
              if (!pdo.push_back(obj, what))
              {
                throw std::runtime_error(std::string("Error!! " + what).c_str());
              }
              uint32_t wd = 0;
              wd = ((uint32_t)obj_idx << 16) | (uint32_t)obj_subidx;
              pdo.start_bytes_map_[wd] = abs_offset;
              pdo.start_bits_map_[wd] = abs_bit;
              pdo.size_bits_map_[wd] = bitlen;
            }
            int length = 0;
            char buffer[1024] = {0};
            length += sprintf(buffer + length, "  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx,
                              obj_subidx, bitlen);
            length +=
                sprintf(buffer + length, "  %-12s %s\n", coe_core::dtype2string(obj_datatype, true).c_str(), str_name);
            printf("%s", std::string(buffer).c_str());

            obj_bitoffset += bitlen;
            totalsize += bitlen;
          }

          PDO->SMbitsize[PDO->SyncM[PDO->nPDO]] += PDO->BitSize[PDO->nPDO];
          Size += PDO->BitSize[PDO->nPDO];
          c++;
        }
        else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
        {
          c += 4 * nEntries;
          a += 8 * nEntries;
          c++;
        }
        if (PDO->nPDO >= (EC_MAXEEPDO - 1))
          c = PDO->Length; /* limit number of PDO entries in buffer */

      } while (c < PDO->Length);
    }
    if ((totalsize % 8) != 0)
    {
      std::cout << __PRETTY_FUNCTION__ << "@" << __LINE__ << ": Weird Module Configuration" << std::endl;
      std::cout << __PRETTY_FUNCTION__ << "@" << __LINE__ << ": Device Address: " << iSlave << std::endl;
      std::cout << __PRETTY_FUNCTION__ << "@" << __LINE__ << ": Device Name: " << ec_slave[iSlave].name << std::endl;
      std::cout << __PRETTY_FUNCTION__ << "@" << __LINE__ << ": Pdo: \n" << pdo.to_string() << std::endl;
      std::cout << __PRETTY_FUNCTION__ << "@" << __LINE__ << ": totalsize: " << totalsize << std::endl;
    }
    pdo.setPackedBytesLenght(totalsize / 8);
    pdo.finalize();
    std::size_t m = (*std::min_element(pdo.start_bytes_map_.begin(), pdo.start_bytes_map_.end(),
                                       [](std::pair<std::size_t, std::size_t> l, std::pair<std::size_t, std::size_t> r)
                                       { return l.second < r.second; }))
                        .second;
    for (auto &b : pdo.start_bytes_map_)
      b.second -= m;

    return true;
  }

  /**
   *
   *
   *
   *
   *
   *
   *
   */
  bool get_pdo_map_through_sdo(const uint16_t &iSlave, coe_core::Pdo &rx_pdo, coe_core::Pdo &tx_pdo, char *IOmap)
  {
    bool ok = false;

    if (IOmap == NULL)
    {
      printf("IOmap null. Abort.");
      return false;
    }

    int wkc, rdl;
    uint8 nSM, tSM;
    uint8 SMt_bug_add = 0;

    rdl = sizeof(nSM);
    nSM = 0;

    wkc = ec_SDOread(iSlave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM,
                     EC_TIMEOUTRXM); /* read SyncManager Communication Type object count */

    if ((wkc <= 0) || (nSM <= 2)) /* positive result from slave ? */
    {
      printf("Error in accessing the SM configuration, module '%s%d%s' ", BOLDRED(), iSlave, RESET());
      return false;
    }
    // printf("Number of SynManagers: %s%d%s", BOLDYELLOW(),nSM,RESET());

    nSM--; // make nSM equal to number of defined SM
    if (nSM > EC_MAXSM)
      nSM = EC_MAXSM; // limit to maximum number of SM defined, if true the slave can't be configured

    for (int iSM = 2; iSM <= nSM; iSM++) // iterate for every SM type defined
    {
      rdl = sizeof(tSM);
      tSM = 0;
      wkc = ec_SDOread(iSlave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM,
                       EC_TIMEOUTRXM); // read SyncManager Communication Type
      if (wkc <= 0)
      {
        printf("Error in readind the register %s0x%x:%d%s", BOLDRED(), ECT_SDO_SMCOMMTYPE, iSM + 1, RESET());
        continue;
      }
      if ((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
      {
        SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
        printf("Error in accessing the SM configuration, module '%s%d%s'. Activated SM type workaround, possible "
               "incorrect mapping",
               BOLDRED(), iSlave, RESET());
      }

      // only add if SMt > 0
      if (tSM)
        tSM += SMt_bug_add; // only add if SMt > 0

      //     // outputs
      //     if (tSM == 3) ok |= coe_core::get_pdo_map_through_sdo(rx_pdo, iSlave, ECT_SDO_PDOASSIGN + iSM,
      //     (std::size_t)(ec_slave[iSlave].outputs - (uint8 *)&IOmap[0]), 0);
      //
      //     // inputs
      //     if (tSM == 4) ok |= coe_core::get_pdo_map_through_sdo(tx_pdo, iSlave, ECT_SDO_PDOASSIGN + iSM,
      //     (std::size_t)(ec_slave[iSlave].inputs - (uint8 *)&IOmap[0], 0) );

      // outputs
      if (tSM == 3)
        ok |= get_pdo_map_through_sdo(rx_pdo, iSlave, ECT_SDO_PDOASSIGN + iSM, 0, 0);

      // inputs
      if (tSM == 4)
        ok |= get_pdo_map_through_sdo(tx_pdo, iSlave, ECT_SDO_PDOASSIGN + iSM, 0, 0);
    }

    return ok;
  }

  bool get_pdo_map_through_sii(const uint16_t &iSlave, coe_core::Pdo &rx_pdo, coe_core::Pdo &tx_pdo, char *IOmap)
  {
    bool ret = false;
    if (IOmap == NULL)
    {
      printf("IOmap null. Abort.");
      return false;
    }
    //   ret |= get_pdo_map_through_sii( rx_pdo, iSlave, 1, (int)(ec_slave[iSlave].outputs - (uint8 *)&IOmap[0]), 0 );
    //   ret |= get_pdo_map_through_sii( tx_pdo, iSlave, 0, (int)(ec_slave[iSlave].inputs  - (uint8 *)&IOmap[0]), 0 );
    ret |= get_pdo_map_through_sii(rx_pdo, iSlave, 1, 0, 0);
    ret |= get_pdo_map_through_sii(tx_pdo, iSlave, 0, 0, 0);

    return ret;
  }

  std::string sdo2string(uint16_t slave, uint16_t index, uint8_t subidx, uint16_t dtype)
  {
    char usdo[128] = {0};
    char hstr[1024] = {0};
    int l = sizeof(usdo) - 1, i;
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
    if (EcatError)
    {
      return ec_elist2string();
    }
    else
    {
      switch (dtype)
      {
      case ECT_BOOLEAN:
        u8 = (uint8 *)&usdo[0];
        if (*u8)
          sprintf(hstr, "TRUE");
        else
          sprintf(hstr, "FALSE");
        break;
      case ECT_INTEGER8:
        i8 = (int8 *)&usdo[0];
        sprintf(hstr, "0x%2.2x %d", *i8, *i8);
        break;
      case ECT_INTEGER16:
        i16 = (int16 *)&usdo[0];
        sprintf(hstr, "0x%4.4x %d", *i16, *i16);
        break;
      case ECT_INTEGER32:
      case ECT_INTEGER24:
        i32 = (int32 *)&usdo[0];
        sprintf(hstr, "0x%8.8x %d", *i32, *i32);
        break;
      case ECT_INTEGER64:
        i64 = (int64 *)&usdo[0];
        sprintf(hstr, "0x%16.16" PRIx64 " %" PRId64, *i64, *i64);
        break;
      case ECT_UNSIGNED8:
        u8 = (uint8 *)&usdo[0];
        sprintf(hstr, "0x%2.2x %u", *u8, *u8);
        break;
      case ECT_UNSIGNED16:
        u16 = (uint16_t *)&usdo[0];
        sprintf(hstr, "0x%4.4x %u", *u16, *u16);
        break;
      case ECT_UNSIGNED32:
      case ECT_UNSIGNED24:
        u32 = (uint32 *)&usdo[0];
        sprintf(hstr, "0x%8.8x %u", *u32, *u32);
        break;
      case ECT_UNSIGNED64:
        u64 = (uint64 *)&usdo[0];
        sprintf(hstr, "0x%16.16" PRIx64 " %" PRIu64, *u64, *u64);
        break;
      case ECT_REAL32:
        sr = (float *)&usdo[0];
        sprintf(hstr, "%f", *sr);
        break;
      case ECT_REAL64:
        dr = (double *)&usdo[0];
        sprintf(hstr, "%f", *dr);
        break;
      case ECT_BIT1:
      case ECT_BIT2:
      case ECT_BIT3:
      case ECT_BIT4:
      case ECT_BIT5:
      case ECT_BIT6:
      case ECT_BIT7:
      case ECT_BIT8:
        u8 = (uint8 *)&usdo[0];
        sprintf(hstr, "0x%x", *u8);
        break;
      case ECT_VISIBLE_STRING:
        strcpy(hstr, usdo);
        break;
      case ECT_OCTET_STRING:
        hstr[0] = 0x00;
        for (i = 0; i < l; i++)
        {
          sprintf(es, "0x%2.2x ", usdo[i]);
          strcat(hstr, es);
        }
        break;
      default:
        sprintf(hstr, "Unknown type");
      }
      return hstr;
    }
  }

  std::string slave2string(int cnt, bool verbose)
  {
    std::string ret = "\n";
    ret += " - Name: " + std::string(ec_slave[cnt].name) + "\n";
    ret += " - Output size: " + std::to_string(ec_slave[cnt].Obits) + "bits\n";
    ret += " - Input size: " + std::to_string(ec_slave[cnt].Ibits) + "bits\n";
    ret += " - State: " + std::to_string(ec_slave[cnt].state) + "\n";
    ret += " - Delay: " + std::to_string(ec_slave[cnt].pdelay) + "ns\n";
    ret += " - Has DC: " + std::to_string(ec_slave[cnt].hasdc) + "\n";
    if (ec_slave[cnt].hasdc)
      ret += " - DCParentport:" + std::to_string(ec_slave[cnt].parentport) + "\n";
    ret += " - Activeports: ";
    ret += std::to_string((ec_slave[cnt].activeports & 0x01) > 0) + ".";
    ret += std::to_string((ec_slave[cnt].activeports & 0x02) > 0) + ".";
    ret += std::to_string((ec_slave[cnt].activeports & 0x04) > 0) + ".";
    ret += std::to_string((ec_slave[cnt].activeports & 0x08) > 0) + "\n";

    ret += " - Configured address: " + coe_core::to_string_hex(ec_slave[cnt].configadr) + " (" +
           std::to_string(ec_slave[cnt].configadr) + ")\n";

    // printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id,
    // (int)ec_slave[cnt].eep_rev);
    if (verbose)
    {
      ret += " - SyncManager:\n";
      for (int nSM = 0; nSM < EC_MAXSM; nSM++)
      {
        if (ec_slave[cnt].SM[nSM].StartAddr > 0)
        {
          ret += "   SM" + std::to_string(nSM) + "# ";
          ret += " StartAddr: " + coe_core::to_string_hex(ec_slave[cnt].SM[nSM].StartAddr) + ",";
          ret += " SMlength: " + std::to_string(ec_slave[cnt].SM[nSM].SMlength) + ",";
          ret += " SMflags: " + coe_core::to_string_hex((int)ec_slave[cnt].SM[nSM].SMflags) + ",";
          ret += " SMlength: " + std::to_string(ec_slave[cnt].SMtype[nSM]) + "\n";
        }
      }
      ret += " - FMM Unused:\n";
      for (int j = 0; j < ec_slave[cnt].FMMUunused; j++)
      {
        ret += "   FMMU" + std::to_string(j) + "# ";
        ret += " LogStart: " + coe_core::to_string_hex(ec_slave[cnt].FMMU[j].LogStart) + ",";
        ret += " LogLength: " + std::to_string(ec_slave[cnt].FMMU[j].LogLength) + ",";
        ret += " LogStartbit: " + std::to_string(ec_slave[cnt].FMMU[j].LogStartbit) + ",";
        ret += " LogEndbit: " + std::to_string(ec_slave[cnt].FMMU[j].LogEndbit) + ",";
        ret += " PhysStart: " + coe_core::to_string_hex(ec_slave[cnt].FMMU[j].PhysStart) + ",";
        ret += " PhysStartBit: " + std::to_string(ec_slave[cnt].FMMU[j].PhysStartBit) + ",";
        ret += " FMMUtype: " + coe_core::to_string_hex(ec_slave[cnt].FMMU[j].FMMUtype) + ",";
        ret += " FMMAct: " + coe_core::to_string_hex(ec_slave[cnt].FMMU[j].FMMUactive) + "\n";
      }
      ret += "   FMMUfunc 0: " + std::to_string(ec_slave[cnt].FMMU0func) + ", ";
      ret += " 1: " + std::to_string(ec_slave[cnt].FMMU1func) + ", ";
      ret += " 2: " + std::to_string(ec_slave[cnt].FMMU2func) + ", ";
      ret += " 3: " + std::to_string(ec_slave[cnt].FMMU3func) + "\n";

      ret += " - MBX length wr: " + std::to_string(ec_slave[cnt].mbx_l) + ", ";
      ret += " rd: " + std::to_string(ec_slave[cnt].mbx_rl) + ", ";
      ret += " MBX protocols : " + coe_core::to_string_hex(ec_slave[cnt].mbx_proto) + "\n";

      uint16 ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
      /* SII general section */
      if (ssigen)
      {
        ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
        ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
        ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
        ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
        if ((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
        {
          ec_slave[cnt].blockLRW = 1;
          ec_slave[0].blockLRW++;
        }
        ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
        ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
        ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
      }
      ret += " - CoE details: " + coe_core::to_string_hex(ec_slave[cnt].CoEdetails) + ",";
      ret += " FoE details: " + coe_core::to_string_hex(ec_slave[cnt].FoEdetails) + ",";
      ret += " EoE details: " + coe_core::to_string_hex(ec_slave[cnt].EoEdetails) + ",";
      ret += " SoE details: " + coe_core::to_string_hex(ec_slave[cnt].SoEdetails) + "\n";
      ret += " - Ebus current: " + std::to_string(ec_slave[cnt].Ebuscurrent) + "[mA]\n";
      ret += " - Only LRD/LWR:" + std::to_string(ec_slave[cnt].blockLRW) + "\n";
    }

    return ret;
  }

}
