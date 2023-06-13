#ifndef SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_PDO_DATA
#define SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_PDO_DATA

#include <cnr_param/cnr_param.h>
#include <coe_core/coe_pdo.h>
#include <coe_core/coe_string_utilities.h>

namespace coe_core
{

namespace PdoData
{

  static const char *KeysId[7] = {"pdo_subindex", "value_id",    "value_index", "value_subindex",
                                  "value_type",   "byte_offest", "bit_offest"};

  enum KeysCode
  {
    PDO_SUBINDEX = 0,
    VALUE_NAME,
    VALUE_INDEX,
    VALUE_SUBINDEX,
    VALUE_TYPE,
    BYTE_OFFSET,
    BIT_OFFSET
  };

  inline void get(const cnr::param::node_t &node, coe_core::Pdo &pdo, const std::string &log)
  {

    cnr::param::node_t config(node);
    if(!cnr::param::is_sequence(config))
    {
      throw std::runtime_error("The node  is not of type array");
    }

    pdo.clear();

    std::map<uint16_t, coe_core::DataObject> objs;
    for(size_t i = 0; i < size_t(config.size()); i++)
    {
      coe_core::BaseDataObjectEntryPtr obj;
      std::string name =
          cnr::param::extract<std::string>(config[i], KeysId[VALUE_NAME], log+", "+std::to_string(i)+"# name");
      int value_index =
          cnr::param::extract<int>(config[i], KeysId[VALUE_INDEX], log+", "+std::to_string(i)+"# value_index");
      int value_subindex = 
        cnr::param::extract<int>(config[i], KeysId[VALUE_SUBINDEX], log+", "+std::to_string(i)+"# value_subindex");
      std::string value_type =
        cnr::param::extract<std::string>(config[i], KeysId[VALUE_TYPE], log+", "+std::to_string(i)+"# value_type");

      switch(coe_core::getType(value_type))
      {
      case ECT_BOOLEAN:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_INTEGER8:
        obj.reset(new coe_core::DataObjectEntry<int8_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_INTEGER16:
        obj.reset(new coe_core::DataObjectEntry<int16_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_INTEGER32:
        obj.reset(new coe_core::DataObjectEntry<int32_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_INTEGER64:
        obj.reset(new coe_core::DataObjectEntry<int64_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_UNSIGNED8:
        obj.reset(new coe_core::DataObjectEntry<uint8_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_UNSIGNED16:
        obj.reset(new coe_core::DataObjectEntry<uint16_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_UNSIGNED32:
        obj.reset(new coe_core::DataObjectEntry<uint32_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_UNSIGNED64:
        obj.reset(new coe_core::DataObjectEntry<uint64_t>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_REAL32:
        obj.reset(new coe_core::DataObjectEntry<double>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_REAL64:
        obj.reset(new coe_core::DataObjectEntry<long double>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT1:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT2:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT3:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT4:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT5:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT6:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT7:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      case ECT_BIT8:
        obj.reset(new coe_core::DataObjectEntry<bool>(value_index, value_subindex, name.c_str()));
        break;
      default:
        throw std::runtime_error("Type not yet implemented.");
      }
      std::string what;
      if(!pdo.push_back(obj, what))
      {
        printf("%s:%u Error: %s", __PRETTY_FUNCTION__, __LINE__, what.c_str());
      }

      if((cnr::param::has(config[i],KeysId[BIT_OFFSET])) &&(cnr::param::has(config[i],KeysId[BYTE_OFFSET])))
      {
        uint32_t wd = 0;
        wd =((uint32_t)value_index << 16) |(uint32_t)value_subindex;
        pdo.start_bytes_map_[wd] = cnr::param::extract<std::size_t>(config[i][KeysId[BYTE_OFFSET]]);
        pdo.start_bits_map_[wd] = cnr::param::extract<std::size_t>(config[i][KeysId[BIT_OFFSET]]);
        pdo.size_bits_map_[wd] = obj->sizeBits();
      }
    }
  }

  inline cnr::param::node_t makeNode(const coe_core::Pdo &pdo)
  {
    cnr::param::node_t value;
    size_t pdo_subindex = 0;
    for(auto cob = pdo.begin(); cob != pdo.end(); cob++, pdo_subindex++)
    {
      uint32_t wd = 0;
      wd =((uint32_t)(*cob)->index() << 16) |(uint32_t)(*cob)->subindex();

      cnr::param::node_t node;
      node[PdoData::KeysId[PDO_SUBINDEX]] = (int)(pdo_subindex+1);
      node[PdoData::KeysId[VALUE_NAME]] = (*cob)->name();
      node[PdoData::KeysId[VALUE_INDEX]] = coe_core::to_string_hex((*cob)->index());
      node[PdoData::KeysId[VALUE_SUBINDEX]] = coe_core::to_string_hex((*cob)->subindex());
      node[PdoData::KeysId[VALUE_TYPE]] = coe_core::dtype2string((*cob)->type(), false);
      node[PdoData::KeysId[BYTE_OFFSET]] = coe_core::to_string_hex((int)(pdo.start_bytes_map_.at(wd)));
      node[PdoData::KeysId[BIT_OFFSET]] = (int)(pdo.start_bits_map_.at(wd));
      value.push_back(node);
    }
    return value;
  }

};

}
#endif  /* SRC_CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_PDO_DATA */
