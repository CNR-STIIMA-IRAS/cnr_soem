/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */
#include <Eigen/Dense>
#include <boost/interprocess/shared_memory_object.hpp>
#include <regex>

#include <cnr_param/cnr_param.h>
#include <coe_core/coe_base.h>
#include <coe_core/coe_pdo.h>
#include <coe_core/coe_sdo.h>

#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_soem_utilities/coe_soem_utilities.h>

#include <coe_master/modules/coe_module_descriptor_data.h>
#include <coe_master/modules/coe_pdo_data.h>
#include <coe_master/modules/coe_sdo_data.h>

using namespace coe_core;

#define HDR                                                                           \
  std::string(std::string(RESET()) + std::string("[-----]") + "[ " +                  \
              (BOLDCYAN() + std::to_string(address_) + "# " + unique_id_ + RESET()) + \
              (BOLDCYAN() + std::string(default_config_ ? " - DEFAULT CFG" : " - PARAMS CFG") + RESET()) + " ]")

namespace coe_master
{

bool ModuleDescriptor::initHandles()
{
  try
  {
    if (!cnr::param::has(root_param_, param_label_))
    {
      printf("The module '%s' has not parameters in the ros parame server.", param_label_.c_str());
      return false;
    }

    cnr::param::node_t node = cnr::param::extract<cnr::param::node_t>(root_param_, param_label_);
    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::AXIS_FEEDBACK]))
    {
      coe_master::AxisData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::AXIS_FEEDBACK]], axis_feedback_, "axis feedback");
    }
    else
      printf("[CHECK] None Axis Feedback Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::AXIS_COMMAND]))
    {
      coe_master::AxisData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::AXIS_COMMAND]], axis_command_, "axis command");
    }
    else
      printf("[CHECK] None Axis Feedback Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::ANALOG_INPUTS]))
    {
      coe_master::AnalogData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::ANALOG_INPUTS]], analog_inputs_, "analog inputs");
    }
    else
      printf("[CHECK] None Analog Input Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::ANALOG_OUTPUTS]))
    {
      coe_master::AnalogData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::ANALOG_OUTPUTS]], analog_outputs_, "analog outputs");
    }
    else
      printf("[CHECK] None Analog Output Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::DIGITAL_INPUTS]))
    {
      coe_master::DigitalData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::DIGITAL_INPUTS]], digital_inputs_, "digital inputs");
    }
    else
      printf("[CHECK] None Digital Input Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::DIGITAL_OUTPUTS]))
    {
      coe_master::DigitalData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::DIGITAL_OUTPUTS]], digital_outputs_, "digital outputs");
    }
    else
      printf("[CHECK] None Digital Output Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::WORD_INPUTS]))
    {
      coe_master::WordData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::WORD_INPUTS]], word_inputs_, "word inputs");
    }
    else
      printf("[CHECK] None Digital Input Handle in rosparam server");

    if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::WORD_OUTPUTS]))
    {
      coe_master::WordData::set(
          node[ModuleData::KeysId[ModuleData::KeysCode::WORD_OUTPUTS]], word_outputs_, "word outputs");
    }
    else
      printf("[CHECK] None Digital Output Handle in rosparam server");
  }
  catch (std::exception& e)
  {
    printf("%s Exception: %s", HDR.c_str(), e.what());
    return false;
  }

  return true;
}

bool ModuleDescriptor::initNodeConfigurationFromParams(const cnr::param::node_t& node)
{
  if (configurated_names_)
  {
    printf("The module labelled with '%s' has already the names configured. Check.", unique_id_.c_str());
    return false;
  }

  root_param_ = node;
  try
  {
    if (!cnr::param::has(root_param_, param_label_))
    {
      printf("The module '%s' has not parameters in the ros parame server.", param_label_.c_str());
      return false;
    }
  }
  catch (std::exception& e)
  {
    printf("%s Exception: %s", HDR.c_str(), e.what());
    return false;
  }

  description_ = "TODO";
  model_ = "TODO";
  model_ = "TODO";
  loop_rate_decimation_ = 1;
  watchdog_decimation_ = 1;
  try
  {
    cnr::param::node_t node = cnr::param::extract<cnr::param::node_t>(root_param_, param_label_);

    description_ = cnr::param::extract<std::string>(
        node, ModuleData::KeysId[ModuleData::KeysCode::DESCRIPTION], "description");  // exception if error
    model_ = cnr::param::extract<std::string>(
        node, ModuleData::KeysId[ModuleData::KeysCode::MODEL], "model");  // exception if error
    loop_rate_decimation_ = cnr::param::extract<int>(node,
                                                     ModuleData::KeysId[ModuleData::KeysCode::LOOP_RATE_DECIMATION],
                                                     "loop rate decimation");  // exception if error
    watchdog_decimation_ = cnr::param::extract<double>(
        node, ModuleData::KeysId[ModuleData::KeysCode::WATCHDOG_DECIMATION], "Wathcdog");  // exception if error
  }
  catch (std::exception& e)
  {
    printf("%s %s", HDR.c_str(), e.what());
  }

  return true;
}

bool ModuleDescriptor::initNodeCoeConfigurationFromParams(const cnr::param::node_t& node, bool configure_sdo)
{
  if (configurated_names_)
  {
    printf("The module labelled with '%s' has already the names configured. Check.", unique_id_.c_str());
    return false;
  }

  root_param_ = node;
  try
  {
    if (!cnr::param::has(root_param_, param_label_))
    {
      printf("The module '%s' has not parameters in the ros parame server.", param_label_.c_str());
      return false;
    }

    cnr::param::node_t node = cnr::param::extract<cnr::param::node_t>(root_param_, param_label_);

    enable_dc_ = cnr::param::extract<bool>(
        node, ModuleData::KeysId[ModuleData::KeysCode::ENABLE_DC], "enable dc");  // exception if error
    support_sdoca_ = cnr::param::extract<bool>(
        node, ModuleData::KeysId[ModuleData::KeysCode::SUPPORT_SDO_CA], "support sdoca");  // exception if error

    if (!default_config_)
    {
      if (configure_sdo)
      {
        if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::SDO]))
        {
          coe_core::SdoData::get(node[ModuleData::KeysId[ModuleData::KeysCode::SDO]], configuration_sdo_, "sdo");
        }
      }

      if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::RXPDO]))
      {
        coe_core::PdoData::get(node[ModuleData::KeysId[ModuleData::KeysCode::RXPDO]], rx_pdo_, "rxpdo");
      }

      if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::TXPDO]))
      {
        coe_core::PdoData::get(node[ModuleData::KeysId[ModuleData::KeysCode::TXPDO]], tx_pdo_, "txpdo");
      }

      tx_pdo_.finalize();
      rx_pdo_.finalize();

      if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::TXPDO_SIZE]))
      {
        int sz = cnr::param::extract<int>(
            node, ModuleData::KeysId[ModuleData::KeysCode::TXPDO_SIZE], "txpdo packed size");  // exception if error
        tx_pdo_.setPackedBytesLenght(sz);
      }
      if (cnr::param::has(node, ModuleData::KeysId[ModuleData::KeysCode::RXPDO_SIZE]))
      {
        int sz = cnr::param::extract<int>(
            node, ModuleData::KeysId[ModuleData::KeysCode::RXPDO_SIZE], "rxpdo packed size");  // exception if error
        rx_pdo_.setPackedBytesLenght(sz);
      }
    }

    configuration_sdo_.finalize();
    configurated_pdo_sdo_ = true;
  }
  catch (std::exception& e)
  {
    printf("%s Exception: %s", HDR.c_str(), e.what());
    return false;
  }

  return configurated_pdo_sdo_;
}

bool ModuleDescriptor::connectHandles()
{
  try
  {
    for (auto& e : axis_command_) e.second.entry = rx_pdo_.subindex(e.second.pdo_subindex - 1);
    for (auto& e : axis_feedback_) e.second.entry = tx_pdo_.subindex(e.second.pdo_subindex - 1);

    for (auto& e : analog_outputs_) e.second.entry = rx_pdo_.subindex(e.second.pdo_subindex - 1);
    for (auto& e : analog_inputs_) e.second.entry = tx_pdo_.subindex(e.second.pdo_subindex - 1);

    for (auto& e : digital_outputs_) e.second.entry = rx_pdo_.subindex(e.second.pdo_subindex - 1);
    for (auto& e : digital_inputs_) e.second.entry = tx_pdo_.subindex(e.second.pdo_subindex - 1);

    for (auto& e : word_outputs_) e.second.entry = rx_pdo_.subindex(e.second.pdo_subindex - 1);
    for (auto& e : word_inputs_) e.second.entry = tx_pdo_.subindex(e.second.pdo_subindex - 1);

    for (auto& e : axis_command_)
      printf(" Axis Command  Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for (auto& e : axis_feedback_)
      printf(" Axis Feedback Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());

    for (auto& e : analog_outputs_)
      printf(" Axis AO       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for (auto& e : analog_inputs_)
      printf(" Axis AI       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());

    for (auto& e : digital_outputs_)
      printf(" Axis DO       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for (auto& e : digital_inputs_)
      printf(" Axis DI       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());

    for (auto& e : word_outputs_)
      printf(" Word Output   Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for (auto& e : word_inputs_)
      printf(" Word Input    Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());

    connected_ = true;
  }
  catch (std::exception& e)
  {
    printf("%s Exception: %s", HDR.c_str(), e.what());
    return false;
  }

  return connected_;
}

bool ModuleDescriptor::initNodeCoeConfigurationFromSoem(const uint16_t iSlave,
                                                        std::string slave_name,
                                                        bool support_sdoca)
{
  std::string module_identifier;
  std::regex allowed_chars_in_name("^[A-Za-z]");

  module_identifier = slave_name;
  if (!std::regex_match(module_identifier, allowed_chars_in_name))
  {
    module_identifier = std::regex_replace(module_identifier, std::regex(R"([^A-Za-z\d])"), "");
  }
  std::string::iterator end_pos = std::remove(module_identifier.begin(), module_identifier.end(), ' ');
  module_identifier.erase(end_pos, module_identifier.end());

  module_identifier += "_" + std::to_string(iSlave);

  address_ = iSlave;
  model_ = module_identifier;
  support_sdoca_ = support_sdoca;
  enable_dc_ = false;
  configurated_pdo_sdo_ = false;

  return true;
}

bool ModuleDescriptor::updateROSParamServer()
{
  ModuleData::get(*this, root_param_);
  if (cnr::param::has(root_param_, param_label_))
  {
  }
  std::string what;
  cnr::param::set(param_label_, root_param_, what);

  return true;
}

std::size_t ModuleDescriptor::sizeInputs() const { return tx_pdo_.nBytes(true); }

std::size_t ModuleDescriptor::sizeOutputs() const { return rx_pdo_.nBytes(true); }

void ModuleDescriptor::updateInputs(const uint8_t* inputs, bool prepended_time)
{
  if (tx_pdo_.nEntries())
  {
    tx_pdo_.update(inputs, prepended_time);
  }
}

void ModuleDescriptor::updateOutputs(const uint8_t* outputs, bool prepended_time)
{
  if (rx_pdo_.nEntries())
  {
    assert(outputs);
    rx_pdo_.update(outputs, prepended_time);
  }
}

const std::string ModuleDescriptor::to_string(const std::string what)
{
  std::stringstream ret;
  ret << getIdentifier() << "\n";
  if ((what == "input") || (what == "tx_pdo") || (what == "txpdo") || (what == "all") || (what == "input-output") ||
      (what == "both"))
  {
    ret << tx_pdo_.to_string();
  }

  if ((what == "output") || (what == "rx_pdo") || (what == "rxpdo") || (what == "all") || (what == "input-output") ||
      (what == "both"))
  {
    ret << rx_pdo_.to_string();
  }
  return ret.str();
}

}  // namespace coe_master
