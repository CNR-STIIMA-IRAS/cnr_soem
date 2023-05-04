#ifndef SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_MODULE_DESCRIPTOR_DATA
#define SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_MODULE_DESCRIPTOR_DATA

#include <tuple>

#include <cnr_param/cnr_param.h>
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

#include <coe_core/coe_pdo.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_pdo_data.h>
#include <coe_master/modules/coe_sdo_data.h>

namespace coe_master
{

namespace AxisData
{

static const char *KeysId[] = {"name", "nominal_motor_torque", "counter_per_motor_round", "gear_ratio", "pdo_subindex"};

enum KeysCode
{
  NAME = 0,
  NOMINAL_MOTOR_TORQUE,
  COUNTER_PER_MOTOR_ROUND,
  GEAR_RATIO,
  PDO_IDX
};

inline void set(const cnr::param::node_t &node,
                std::map<std::string, coe_master::ModuleDescriptor::AxisDataEntry> &entries,
                const std::string &log)
{
  cnr::param::node_t config(node);
  if (!cnr::param::is_sequence(config))
  {
    throw std::runtime_error("The node  is not of type array");
  }

  entries.clear();
  for (std::size_t i = 0; i < cnr::param::size(config); i++)
  {
    coe_master::ModuleDescriptor::AxisDataEntry entry;
    std::string name =
        cnr::param::extract<std::string>(config[i], KeysId[NAME], log + ", " + std::to_string(i) + "# name");

    entry.nominal_motor_torque =
        cnr::param::has(config[i], KeysId[NOMINAL_MOTOR_TORQUE])
            ? cnr::param::extract<double>(
                  config[i], KeysId[NOMINAL_MOTOR_TORQUE], log + ", " + std::to_string(i) + "# NOMINAL_MOTOR_TORQUE")
            : 0;

    entry.counter_per_motor_round =
        cnr::param::has(config[i], KeysId[COUNTER_PER_MOTOR_ROUND])
            ? cnr::param::extract<double>(config[i],
                                          KeysId[COUNTER_PER_MOTOR_ROUND],
                                          log + ", " + std::to_string(i) + "# COUNTER_PER_MOTOR_ROUND")
            : 0;

    entry.gear_ratio =
        cnr::param::extract<double>(config[i], KeysId[GEAR_RATIO], log + ", " + std::to_string(i) + "# GEAR_RATIO");
    entry.pdo_subindex =
        cnr::param::extract<uint8_t>(config[i], KeysId[PDO_IDX], log + ", " + std::to_string(i) + "# pdo subindex");
    entries.insert(std::make_pair(name, entry));
  }
}

inline cnr::param::node_t makeNode(const std::map<std::string, coe_master::ModuleDescriptor::AxisDataEntry> &entries)
{
  cnr::param::node_t data;
  for (auto const &entry : entries)
  {
    cnr::param::node_t node;
    cnr::param::insert(node, KeysId[NAME], (std::string)entry.first);
    cnr::param::insert(node, KeysId[COUNTER_PER_MOTOR_ROUND], (double)entry.second.counter_per_motor_round);
    cnr::param::insert(node, KeysId[GEAR_RATIO], (double)entry.second.gear_ratio);
    cnr::param::insert(node, KeysId[PDO_IDX], (int)entry.second.pdo_subindex);
    data.push_back(node);
  }
  return data;
}

}  // namespace AxisData

namespace AnalogData
{

static const char *KeysId[5] = {"name", "scale", "offset", "pdo_subindex"};

enum KeysCode
{
  NAME = 0,
  SCALE,
  OFFSET,
  PDO_IDX
};

inline void set(const cnr::param::node_t &node,
                std::map<std::string, coe_master::ModuleDescriptor::AnalogDataEntry> &entries,
                const std::string &log)
{
  cnr::param::node_t config(node);
  if (!cnr::param::is_sequence(config))
  {
    throw std::runtime_error("The node  is not of type array");
  }

  entries.clear();
  for (std::size_t i = 0; i < cnr::param::size(config); i++)
  {
    coe_master::ModuleDescriptor::AnalogDataEntry entry;
    std::string name =
        cnr::param::extract<std::string>(config[i], KeysId[NAME], log + ", " + std::to_string(i) + "# name");
    entry.scale = cnr::param::extract<double>(config[i], KeysId[SCALE], log + ", " + std::to_string(i) + "# scale");
    entry.offset = cnr::param::extract<double>(config[i], KeysId[OFFSET], log + ", " + std::to_string(i) + "# offset");
    entry.pdo_subindex =
        cnr::param::extract<uint8_t>(config[i], KeysId[PDO_IDX], log + ", " + std::to_string(i) + "# pdo subindex");
    entries.insert(std::make_pair(name, entry));
  }
}

inline cnr::param::node_t makeNode(const std::map<std::string, coe_master::ModuleDescriptor::AnalogDataEntry> &entries)
{
  cnr::param::node_t data;
  for (auto const &entry : entries)
  {
    cnr::param::node_t node;
    cnr::param::insert(node, KeysId[NAME], (std::string)entry.first);
    cnr::param::insert(node, KeysId[SCALE], (double)entry.second.scale);
    cnr::param::insert(node, KeysId[OFFSET], (double)entry.second.offset);
    cnr::param::insert(node, KeysId[PDO_IDX], (int)entry.second.pdo_subindex);
    data.push_back(node);
  }
  return data;
}

/*
    inline void set( const cnr::param::node_t& node,
  std::map<std::string,coe_master::ModuleDescriptor::AnalogDataEntry>& entries, const std::string& log )
  {
    std::map<std::string,coe_master::ModuleDescriptor::AxisDataEntry>  entries_;
    coe_master::AxisData::set( node, entries_, log );

    entries.clear();
    for( auto const entry : entries_ )
      entries.insert( std::make_pair( entry.first, *(coe_master::ModuleDescriptor::AnalogDataEntry*)&(entry.second)
  ) );

  }

  inline void get( const std::map<std::string,coe_master::ModuleDescriptor::AnalogDataEntry>& entries,
  cnr::param::node_t& data  )
  {
    std::map<std::string,coe_master::ModuleDescriptor::AxisDataEntry>  entries_;
    for( auto const entry : entries )
      entries_.insert( std::make_pair( entry.first, *(coe_master::ModuleDescriptor::AxisDataEntry*)&(entry.second) )
  );

    return coe_master::AxisData::get( entries_, data );
  }
          */
};  // namespace AnalogData

namespace DigitalData
{

static const char *KeysId[3] = {"name", "pdo_subindex", "bit"};

enum KeysCode
{
  NAME = 0,
  PDO_IDX,
  BIT
};

inline void set(const cnr::param::node_t &node,
                std::map<std::string, coe_master::ModuleDescriptor::DigitalDataEntry> &entries,
                const std::string &log)
{
  cnr::param::node_t config(node);
  if (!cnr::param::is_sequence(config))
  {
    throw std::runtime_error("The node  is not of type array");
  }

  entries.clear();
  for (std::size_t i = 0; i < cnr::param::size(config); i++)
  {
    coe_master::ModuleDescriptor::DigitalDataEntry entry;
    std::string name =
        cnr::param::extract<std::string>(config[i], KeysId[NAME], log + ", " + std::to_string(i) + "# name");
    entry.pdo_subindex =
        cnr::param::extract<uint8_t>(config[i], KeysId[PDO_IDX], log + ", " + std::to_string(i) + "# pdo subindex");
    entry.bit = cnr::param::extract<uint8_t>(config[i], KeysId[BIT], log + ", " + std::to_string(i) + "# bit");

    entries.insert(std::make_pair(name, entry));
  }
}

inline cnr::param::node_t makeNode(const std::map<std::string, coe_master::ModuleDescriptor::DigitalDataEntry> &entries)
{
  cnr::param::node_t data;
  for (auto const &entry : entries)
  {
    cnr::param::node_t node;
    cnr::param::insert(node, KeysId[NAME], (std::string)entry.first);
    cnr::param::insert(node, KeysId[PDO_IDX], (int)entry.second.pdo_subindex);
    cnr::param::insert(node, KeysId[BIT], (int)entry.second.bit);
    data.push_back(node);
  }
  return data;
}

};  // namespace DigitalData

namespace WordData
{

static const char *KeysId[5] = {"name", "pdo_subindex"};

enum KeysCode
{
  NAME = 0,
  PDO_IDX
};

inline void set(const cnr::param::node_t &node,
                std::map<std::string, coe_master::ModuleDescriptor::WordDataEntry> &entries,
                const std::string &log)
{
  cnr::param::node_t config(node);
  if (!cnr::param::is_sequence(config))
  {
    throw std::runtime_error("The node  is not of type array");
  }

  entries.clear();
  for (std::size_t i = 0; i < cnr::param::size(config); i++)
  {
    coe_master::ModuleDescriptor::WordDataEntry entry;
    std::string name =
        cnr::param::extract<std::string>(config[i], KeysId[NAME], log + ", " + std::to_string(i) + "# name");
    entry.pdo_subindex =
        cnr::param::extract<uint8_t>(config[i], KeysId[PDO_IDX], log + ", " + std::to_string(i) + "# pdo subindex");
    entries.insert(std::make_pair(name, entry));
  }
}

inline cnr::param::node_t makeNode(const std::map<std::string, coe_master::ModuleDescriptor::WordDataEntry> &entries)
{
  cnr::param::node_t data;
  for (auto const &entry : entries)
  {
    cnr::param::node_t node;
    cnr::param::insert(node, KeysId[NAME], (std::string)entry.first);
    cnr::param::insert(node, KeysId[PDO_IDX], (int)entry.second.pdo_subindex);
    data.push_back(node);
  }
  return data;
}

};  // namespace WordData

namespace ModuleData
{

static const char *KeysId[] = {"model",
                               "description",
                               "enable_dc",
                               "sdo_complete_access",
                               "loop_rate_decimation",
                               "watchdog_decimation",
                               "axis_feedback",
                               "axis_command",
                               "analog_inputs",
                               "analog_outputs",
                               "digital_inputs",
                               "digital_outputs",
                               "word_inputs",
                               "word_outputs",
                               "rxpdo",
                               "rxpdo_packed_size",
                               "txpdo",
                               "txpdo_packed_size",
                               "sdo"};
enum KeysCode
{
  MODEL = 0,
  DESCRIPTION,
  ENABLE_DC,
  SUPPORT_SDO_CA,
  LOOP_RATE_DECIMATION,
  WATCHDOG_DECIMATION,
  AXIS_FEEDBACK,
  AXIS_COMMAND,
  ANALOG_INPUTS,
  ANALOG_OUTPUTS,
  DIGITAL_INPUTS,
  DIGITAL_OUTPUTS,
  WORD_INPUTS,
  WORD_OUTPUTS,
  RXPDO,
  RXPDO_SIZE,
  TXPDO,
  TXPDO_SIZE,
  SDO
};

inline void set(const cnr::param::node_t &node, coe_master::ModuleDescriptor &module)
{
  module.initNodeConfigurationFromParams(node);
  module.initNodeCoeConfigurationFromParams(node, true);  // force the sdo configuration if in the ros-param server
}

inline void get(const coe_master::ModuleDescriptor &module, cnr::param::node_t &data)
{
  cnr::param::insert(data, KeysId[KeysCode::DESCRIPTION], (std::string)module.getDescription());
  cnr::param::insert(data, KeysId[KeysCode::MODEL], (std::string)module.getIdentifier());
  cnr::param::insert(data, KeysId[KeysCode::ENABLE_DC], (bool)module.isDcEnabled());
  cnr::param::insert(data, KeysId[KeysCode::SUPPORT_SDO_CA], (bool)module.isSdoCaSupported());
  cnr::param::insert(data, KeysId[KeysCode::LOOP_RATE_DECIMATION], (int)module.getLoopRateDecimation());
  cnr::param::insert(data, KeysId[KeysCode::WATCHDOG_DECIMATION], (int)module.getWatchdogDecimation());

  std::string log;
  cnr::param::insert(data, KeysId[KeysCode::AXIS_COMMAND], AxisData::makeNode(module.getAxisCommand()));
  cnr::param::insert(data, KeysId[KeysCode::AXIS_FEEDBACK], AxisData::makeNode(module.getAxisFeedback()));

  cnr::param::insert(data, KeysId[KeysCode::ANALOG_INPUTS], AnalogData::makeNode(module.getAnalogInputs()));
  cnr::param::insert(data, KeysId[KeysCode::ANALOG_OUTPUTS], AnalogData::makeNode(module.getAnalogOutputs()));

  cnr::param::insert(data, KeysId[KeysCode::DIGITAL_INPUTS], DigitalData::makeNode(module.getDigitalInputs()));
  cnr::param::insert(data, KeysId[KeysCode::DIGITAL_OUTPUTS], DigitalData::makeNode(module.getDigitalOutputs()));

  cnr::param::insert(data, KeysId[KeysCode::WORD_INPUTS], WordData::makeNode(module.getWordInputs()));
  cnr::param::insert(data, KeysId[KeysCode::WORD_OUTPUTS], WordData::makeNode(module.getWordOutputs()));

  cnr::param::insert(data, KeysId[KeysCode::TXPDO], coe_core::PdoData::makeNode(module.getTxPdo()));
  cnr::param::insert(data, KeysId[KeysCode::RXPDO], coe_core::PdoData::makeNode(module.getRxPdo()));

  cnr::param::insert(data, KeysId[KeysCode::TXPDO_SIZE], (int)module.sizeInputs());
  cnr::param::insert(data, KeysId[KeysCode::RXPDO_SIZE], (int)module.sizeOutputs());

  if (module.getConfigurationSdo().nEntries() > 0)
  {
    cnr::param::insert(data, KeysId[KeysCode::SDO], coe_core::SdoData::makeNode(module.getConfigurationSdo()));
  }
}

};  // namespace ModuleData

}  // namespace coe_master

#endif /* SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_MODULE_DESCRIPTOR_DATA */
