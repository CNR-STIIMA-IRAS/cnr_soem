#ifndef SRC_CNR_SOEM_COE_SOEM_UTILITIES_INCLUDE_COE_SOEM_UTILITIES_COE_SOEM_UTILITIES
#define SRC_CNR_SOEM_COE_SOEM_UTILITIES_INCLUDE_COE_SOEM_UTILITIES_COE_SOEM_UTILITIES

#include <cstdint>
#include <map>

#include <coe_core/coe_pdo.h>
#include <coe_core/coe_sdo.h>

#include <ethercat.h>
#include <ethercatmain.h>
#include <ethercatprint.h>

namespace coe_soem_utilities
{


typedef int (*PO2SOconfigFcn)(uint16_t slave);

bool soem_init(const std::string &adapter_id, const double &timeout_s = 10.0);
bool soem_init(const std::string &adapter_id,
               const double &timeout_s,
               const std::map<int, std::string> &list_of_slaves);

uint32_t soem_check_network_configuration(const std::map<int, std::string> list_of_slaves);

char *soem_config(const double timeout_s,
                  bool config_dc,
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

/**
 * @brief Get the pdo map through sdo object
 * 
 * @param iSlave 
 * @param rx_pdo 
 * @param tx_pdo 
 * @param IOmap 
 * @param what 
 * @return int  <0 ERROR, =0 OK, >1 WARNING
 */
int get_pdo_map_through_sdo(
    const uint16_t &iSlave, coe_core::RxPdo &rx_pdo, coe_core::TxPdo &tx_pdo, char *IOmap, std::string &what);

int get_pdo_map_through_sii(
    const uint16_t &iSlave, coe_core::RxPdo &rx_pdo, coe_core::TxPdo &tx_pdo, char *IOmap, std::string &what);

std::string sdo2string(uint16_t slave, uint16_t index, uint8_t subidx, uint16_t dtype);
std::string slave2string(int cnt, bool verbose);

}  // namespace coe_soem_utilities

// IMPL

namespace coe_soem_utilities
{

}

#endif  /* SRC_CNR_SOEM_COE_SOEM_UTILITIES_INCLUDE_COE_SOEM_UTILITIES_COE_SOEM_UTILITIES */
