#ifndef COE_MASTER_MODULES_COE_SDO_DATA
#define COE_MASTER_MODULES_COE_SDO_DATA

#include <cnr_param/cnr_param.h>
#include <coe_core/coe_pdo.h>
#include <coe_core/coe_sdo.h>

namespace coe_core
{

namespace SdoData
{

static const char* KeysId[] = {"value_id", "value_index", "value_subindex", "value_type", "value_access", "value"};

enum KeysCode
{
  VALUE_NAME,
  VALUE_INDEX,
  VALUE_SUBINDEX,
  VALUE_TYPE,
  VALUE_ACCESS,
  VALUE
};

inline void get(const cnr::param::node_t& node, coe_core::Sdo& sdo, const std::string& log)
{
  cnr::param::node_t config(node);
  if (!cnr::param::is_sequence(config))
  {
    throw std::runtime_error("The node is not of type array");
  }

  sdo.clear();
  for (size_t i = 0; i < size_t(config.size()); i++)
  {
    coe_core::BaseDataObjectEntryPtr obj;
    std::string name =
        cnr::param::extract<std::string>(config[i], KeysId[VALUE_NAME], log + ", " + std::to_string(i) + "# name");
    int value_index =
        cnr::param::extract<int>(config[i], KeysId[VALUE_INDEX], log + ", " + std::to_string(i) + "# value_index");
    int value_subindex = cnr::param::extract<int>(
        config[i], KeysId[VALUE_SUBINDEX], log + ", " + std::to_string(i) + "# value_subindex");
    std::string value_type = cnr::param::extract<std::string>(
        config[i], KeysId[VALUE_TYPE], log + ", " + std::to_string(i) + "# value_type");
    std::string value_access = cnr::param::extract<std::string>(
        config[i], KeysId[VALUE_ACCESS], log + ", " + std::to_string(i) + "# value_access");
    long int value =
        (value_access == "write")
            ? cnr::param::extract<long int>(config[i], KeysId[VALUE], log + ", " + std::to_string(i) + "# value")
            : 0;

    switch (coe_core::getType(value_type))
    {
      case ECT_BOOLEAN:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_INTEGER8:
        obj.reset(new coe_core::DataObjectEntry<int8_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_INTEGER16:
        obj.reset(new coe_core::DataObjectEntry<int16_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_INTEGER32:
        obj.reset(new coe_core::DataObjectEntry<int32_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_INTEGER64:
        obj.reset(new coe_core::DataObjectEntry<int64_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_UNSIGNED8:
        obj.reset(new coe_core::DataObjectEntry<uint8_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_UNSIGNED16:
        obj.reset(new coe_core::DataObjectEntry<uint16_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_UNSIGNED32:
        obj.reset(new coe_core::DataObjectEntry<uint32_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_UNSIGNED64:
        obj.reset(new coe_core::DataObjectEntry<uint64_t>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_REAL32:
        obj.reset(new coe_core::DataObjectEntry<double>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_REAL64:
        obj.reset(new coe_core::DataObjectEntry<long double>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT1:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT2:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT3:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT4:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT5:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT6:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT7:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      case ECT_BIT8:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str(), value));
        break;
      default:
        throw std::runtime_error("Type not yet implemented.");
    }
    std::string what;
    if(!sdo.push_back(obj, what))
    {
      throw std::runtime_error(what.c_str());
    }

    sdo.write_access[obj->address()] = (value_access == "write");
  }
}

inline cnr::param::node_t makeNode(const coe_core::Sdo& sdo)
{
  cnr::param::node_t data;
  for (size_t i = 0; i < sdo.nEntries(); i++)
  {
    auto const& cob = sdo.at(i);
    cnr::param::node_t node;
    node[SdoData::KeysId[VALUE_NAME]] = cob->name();
    node[SdoData::KeysId[VALUE_INDEX]] = coe_core::to_string_hex(cob->index());
    node[SdoData::KeysId[VALUE_SUBINDEX]] = coe_core::to_string_hex(cob->subindex());
    node[SdoData::KeysId[VALUE_TYPE]] = coe_core::getType(cob->type());

    if (sdo.write_access.at(cob->address()))
    {
      node[SdoData::KeysId[VALUE_ACCESS]] = std::string("write");
      int value = 0;
      switch (cob->type())
      {
        case ECT_BOOLEAN:
          value = (int)cob->get<bool>();
          break;
        case ECT_INTEGER8:
          value = (int)cob->get<int8_t>();
          break;
        case ECT_INTEGER16:
          value = (int)cob->get<int16_t>();
          break;
        case ECT_INTEGER32:
          value = (int)cob->get<int32_t>();
          break;
        case ECT_INTEGER64:
          value = (int)cob->get<int64_t>();
          break;
        case ECT_UNSIGNED8:
          value = (int)cob->get<uint8_t>();
          break;
        case ECT_UNSIGNED16:
          value = (int)cob->get<uint16_t>();
          break;
        case ECT_UNSIGNED32:
          value = (int)cob->get<uint32_t>();
          break;
        case ECT_UNSIGNED64:
          value = (int)cob->get<uint64_t>();
          break;
        case ECT_REAL32:
          value = (int)cob->get<double>();
          break;
        case ECT_REAL64:
          value = (int)cob->get<long double>();
          break;
        case ECT_BIT1:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT2:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT3:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT4:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT5:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT6:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT7:
          value = (int)cob->get<bool>();
          break;
        case ECT_BIT8:
          value = (int)cob->get<bool>();
          break;
        default:
          throw std::runtime_error("Type not yet implemented.");
      }
      node[SdoData::KeysId[VALUE]] = value;
    }
    else
    {
      node[SdoData::KeysId[VALUE_ACCESS]] = std::string("read");
    }
    data.push_back(node);
  }
  return data;
}

}  // namespace SdoData
}  // namespace coe_core

#endif // COE_MASTER_MODULES_COE_SDO_DATA
