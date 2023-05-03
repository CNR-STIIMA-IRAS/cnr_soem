#ifndef COE_SOEM_UTILITIES__COE_SOEM_UTILITIES__H
#define COE_SOEM_UTILITIES__COE_SOEM_UTILITIES__H

#include <coe_core/coe_pdo.h>
#include <coe_core/coe_sdo.h>
#include <cstdint>
#include <map>

#include <ethercat.h>
#include <ethercatmain.h>
#include <ethercatprint.h>

namespace coe_soem_utilities
{

  typedef int (*PO2SOconfigFcn)(uint16_t slave);

  bool soem_init(const std::string &adapter_name, const double timeout_s = 10.0);
  bool soem_init(const std::string &adapter_name, const double timeout_s,
                 const std::map<int, std::string> ordered_list_of_devices);

  uint32_t soem_check_network_configuration(const std::map<int, std::string> ordered_list_of_devices);

  char *soem_config(const double timeout_s, bool config_dc,
                    const std::map<int, bool> &disable_sdoca = std::map<int, bool>(),
                    const std::map<int, PO2SOconfigFcn> &config_fcn = std::map<int, PO2SOconfigFcn>());
  bool soem_wait_for_state(const ec_state &target_state);
  bool soem_reset_to_operational_state();

  std::vector<ec_errort> soem_errors();
  std::string to_string(ec_errort);
  std::string to_string(const ec_err_type &t);

  int get_node_information(const uint16_t addresses);

  int get_object_description_list(uint16_t slave, const double timeout_s);

  bool get_cob_via_sdo(uint16_t addr, coe_core::BaseDataObjectEntry *in);
  bool set_cob_via_sdo(uint16_t addr, const coe_core::BaseDataObjectEntry *in);

  bool get_pdo_map_through_sdo(coe_core::Pdo &pdo, const uint16_t &iSlave, const uint16_t &PDOassign,
                               const size_t mapoffset, const size_t bitoffset = 0);
  bool get_pdo_map_through_sii(coe_core::Pdo &pdo, const uint16_t &iSlave, const uint16_t &SIIcategory,
                               const size_t mapoffset, const size_t bitoffset = 0);

  bool get_pdo_map_through_sdo(const uint16_t &iSlave, coe_core::Pdo &rx_pdo, coe_core::Pdo &tx_pdo,
                               char *IOmap = NULL);
  bool get_pdo_map_through_sii(const uint16_t &iSlave, coe_core::Pdo &rx_pdo, coe_core::Pdo &tx_pdo,
                               char *IOmap = NULL);

  std::string sdo2string(uint16_t slave, uint16_t index, uint8_t subidx, uint16_t dtype);
  std::string slave2string(int cnt, bool verbose);

}

#endif // COE_SOEM_UTILITIES__COE_SOEM_UTILITIES__H
