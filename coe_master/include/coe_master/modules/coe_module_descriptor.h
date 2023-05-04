
#ifndef COE_MASTER__MODULES__COE_MODULE_DESCRIPTION_H
#define COE_MASTER__MODULES__COE_MODULE_DESCRIPTION_H

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

#include <cnr_param/cnr_param.h>
#include <coe_core/coe_pdo.h>
#include <coe_core/coe_utilities.h>
#include <tuple>

namespace coe_master
{

inline std::string uniqueId(const std::string &model, const int address, const std::string &separator = "__")
{
  return model + separator + std::to_string(address);
}

class ModuleDescriptor
{
public:
  typedef std::shared_ptr<ModuleDescriptor> Ptr;

  struct AxisDataEntry
  {
    coe_core::BaseDataObjectEntryPtr entry;
    double nominal_motor_torque;
    double counter_per_motor_round;
    double gear_ratio;
    uint8_t pdo_subindex;
  };

  struct WordDataEntry
  {
    coe_core::BaseDataObjectEntryPtr entry;
    uint8_t pdo_subindex;
  };

  struct AnalogDataEntry
  {
    coe_core::BaseDataObjectEntryPtr entry;
    double scale;
    double offset;
    uint8_t pdo_subindex;
  };

  struct DigitalDataEntry
  {
    coe_core::BaseDataObjectEntryPtr entry;
    uint8_t pdo_subindex;
    uint8_t bit;
  };

public:
  ModuleDescriptor(const std::string root_param, const std::string &model_name,
                    const int &address, const bool default_config)
      : root_param_(root_param), param_label_(model_name), unique_id_(uniqueId(model_name, address)),
        default_config_(default_config), configurated_names_(false), configurated_pdo_sdo_(default_config),
        connected_(false), address_(address), rx_pdo_(ECT_SDO_RXPDOASSIGN), tx_pdo_(ECT_SDO_TXPDOASSIGN)
  {
    // NOTE: IF DEFAULT CONFIG, THEN ALSO THE CONFIGURATION SDO ARE DEFAULT
  }

  bool initNodeConfigurationFromParams(const cnr::param::node_t &node);
  bool initNodeCoeConfigurationFromParams(const cnr::param::node_t &node, bool configure_sdo);
  bool initNodeCoeConfigurationFromSoem(const uint16_t iSlave, std::string slave_name, bool support_sdoca);

  bool initHandles();
  bool connectHandles();
  bool updateROSParamServer();

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::size_t sizeInputs() const;
  std::size_t sizeOutputs() const;
  void updateInputs(const uint8_t *inputs, bool prepended_time);
  void updateOutputs(const uint8_t *outputs, bool prepended_time);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  const int &getAddress() const { return address_; }
  const std::string &getDescription() const { return description_; }
  const std::string &getModel() const { return model_; }
  const std::string &getIdentifier() const { return unique_id_; }
  bool isDefaultConfig() const { return default_config_; }
  bool isDcEnabled() const { return enable_dc_; }
  bool isSdoCaSupported() const { return support_sdoca_; }
  int getLoopRateDecimation() const { return loop_rate_decimation_; }
  int getWatchdogDecimation() const { return watchdog_decimation_; }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  const std::map<std::string, AxisDataEntry> &getAxisCommand() const { return axis_command_; }
  const std::map<std::string, AxisDataEntry> &getAxisFeedback() const { return axis_feedback_; }
  const std::map<std::string, AnalogDataEntry> &getAnalogInputs() const { return analog_inputs_; }
  const std::map<std::string, AnalogDataEntry> &getAnalogOutputs() const { return analog_outputs_; }
  const std::map<std::string, DigitalDataEntry> &getDigitalInputs() const { return digital_inputs_; }
  const std::map<std::string, DigitalDataEntry> &getDigitalOutputs() const { return digital_outputs_; }
  const std::map<std::string, WordDataEntry> &getWordInputs() const { return word_inputs_; }
  const std::map<std::string, WordDataEntry> &getWordOutputs() const { return word_outputs_; }

  const AxisDataEntry &getAxisCommand(const std::string &k) const { return axis_command_.at(k); }
  const AxisDataEntry &getAxisFeedback(const std::string &k) const { return axis_feedback_.at(k); }
  const AnalogDataEntry &getAnalogInputs(const std::string &k) const { return analog_inputs_.at(k); }
  const AnalogDataEntry &getAnalogOutputs(const std::string &k) const { return analog_outputs_.at(k); }
  const DigitalDataEntry &getDigitalInputs(const std::string &k) const { return digital_inputs_.at(k); }
  const DigitalDataEntry &getDigitalOutputs(const std::string &k) const { return digital_outputs_.at(k); }
  const WordDataEntry &getWordInputs(const std::string &k) const { return word_inputs_.at(k); }
  const WordDataEntry &getWordOutputs(const std::string &k) const { return word_outputs_.at(k); }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  const std::string getSoemInputId() const { return unique_id_ + "_txpdo"; }
  const std::string getSoemOutputId() const { return unique_id_ + "_rxpdo"; }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  const coe_core::Pdo &getRxPdo() const { return rx_pdo_; }
  const coe_core::Pdo &getTxPdo() const { return tx_pdo_; }
  const coe_core::Sdo &getConfigurationSdo() const { return configuration_sdo_; }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  coe_core::Pdo &getRxPdo() { return rx_pdo_; }
  coe_core::Pdo &getTxPdo() { return tx_pdo_; }

  // utilities
  const std::string to_string(const std::string what = "input");
  //void to_param(cnr::param::node_t &config) const;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void setConfiguratedSdo(bool val) { configurated_pdo_sdo_ = val; }
  bool getConfiguratedSdo() const { return configurated_pdo_sdo_; }

private:
  cnr::param::node_t root_param_;
  const std::string param_label_;
  const std::string unique_id_;
  const bool default_config_;
  bool configurated_names_;
  bool configurated_pdo_sdo_;
  bool connected_;
  int address_;
  std::string description_;
  std::string model_;

  bool enable_dc_;
  bool support_sdoca_;
  int loop_rate_decimation_;
  int watchdog_decimation_;
  coe_core::Sdo configuration_sdo_;

  coe_core::Pdo rx_pdo_;
  coe_core::Pdo tx_pdo_;

  std::map<std::string, AxisDataEntry> axis_feedback_;
  std::map<std::string, AxisDataEntry> axis_command_;

  std::map<std::string, AnalogDataEntry> analog_inputs_;
  std::map<std::string, AnalogDataEntry> analog_outputs_;

  std::map<std::string, DigitalDataEntry> digital_inputs_;
  std::map<std::string, DigitalDataEntry> digital_outputs_;

  std::map<std::string, WordDataEntry> word_inputs_;
  std::map<std::string, WordDataEntry> word_outputs_;
};

typedef ModuleDescriptor::Ptr ModuleDescriptorPtr;

}  // namespace coe_master

#endif  // COE_MASTER__MODULES__COE_MODULE_DESCRIPTION_H
