
#ifndef CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_NETWORK_DESCRIPTOR_DATA
#define CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_NETWORK_DESCRIPTOR_DATA

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <tuple>

#include <coe_core/coe_pdo.h>
#include <coe_core/coe_utilities.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_module_descriptor_data.h>
#include <coe_master/modules/coe_network_descriptor.h>

namespace coe_master
{

namespace NetworkData
{

static const char *KeysId[9] = {"label", "address", "mapped", "default"};
enum KeysCode
{
  LABEL = 0,
  ADDRESS,
  MAPPED,
  DEFAULT
};

typedef std::tuple<std::string, uint16_t, bool, bool> ModuleListData;

inline bool get(const ModuleListData &t, cnr::param::node_t &data)
{
  data[KeysId[KeysCode::LABEL]] = std::get<KeysCode::LABEL>(t);
  data[KeysId[KeysCode::ADDRESS]] = std::get<KeysCode::ADDRESS>(t);
  data[KeysId[KeysCode::MAPPED]] = std::get<KeysCode::MAPPED>(t);
  data[KeysId[KeysCode::DEFAULT]] = std::get<KeysCode::DEFAULT>(t);
  return true;
}

// template <typename T> void set(const std::string &key, const std::vector<T> &vec)
// {

//   // Note: the XmlRpcValue starts off as "invalid" and assertArray turns it
//   // into an array type with the given size
//   cnr::param::node_t data;
//   data.setSize(vec.size());

//   // Copy the contents into the XmlRpcValue
//   for (std::size_t i = 0; i < vec.size(); i++)
//   {
//     get(vec.at(i), data[i]);
//   }
//   cnr::param::set(key, data);
// }

}

}

#endif  /* CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_COE_NETWORK_DESCRIPTOR_DATA */
