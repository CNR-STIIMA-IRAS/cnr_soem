#ifndef CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_UTILITIES_COMMON
#define CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_UTILITIES_COMMON

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <array>
#include <boost/array.hpp>
#include <coe_core/coe_string_utilities.h>

namespace coe_master 
{


inline std::string to_string( const uint16_t& index, const uint8_t& subindex, const uint8_t& /*sdotype*/)
{
  std::string s = coe_core::to_string_hex( index ) + ":" + std::to_string( subindex );
  return s;  
}
inline std::string to_string( const uint8_t sdotype, const boost::array<uint8_t,8>& value )
{
  std::string s; 
  switch(sdotype) 
  {
    case 0: { s += coe_core::to_string_hex( *(uint8_t*  )&value[0] ); s += " [ UINT8  ]"; } break;
    case 1: { s += coe_core::to_string_hex( *(uint16_t* )&value[0] ); s += " [ UINT16 ]"; } break;
    case 2: { s += coe_core::to_string_hex( *(uint32_t* )&value[0] ); s += " [ UINT32 ]"; } break;
    case 3: { s += coe_core::to_string_hex( *(uint64_t* )&value[0] ); s += " [ UINT64 ]"; } break;
    case 4: { s += coe_core::to_string_hex( *(int8_t*   )&value[0] ); s += " [ INT8   ]"; }break;
    case 5: { s += coe_core::to_string_hex( *(int16_t*  )&value[0] ); s += " [ INT16  ]"; }break;
    case 6: { s += coe_core::to_string_hex( *(int32_t*  )&value[0] ); s += " [ INT32  ]"; }break;
    case 7: { s += coe_core::to_string_hex( *(int64_t*  )&value[0] ); s += " [ INT64  ]"; }break;
  }
  
  return s;  
}

inline std::string to_string( const uint8_t sdotype, const std::array<uint8_t,8>& value )
{
  std::string s; 
  switch(sdotype) 
  {
    case 0: { s += coe_core::to_string_hex( *(uint8_t*  )&value[0] ); s += " [ UINT8  ]"; } break;
    case 1: { s += coe_core::to_string_hex( *(uint16_t* )&value[0] ); s += " [ UINT16 ]"; } break;
    case 2: { s += coe_core::to_string_hex( *(uint32_t* )&value[0] ); s += " [ UINT32 ]"; } break;
    case 3: { s += coe_core::to_string_hex( *(uint64_t* )&value[0] ); s += " [ UINT64 ]"; } break;
    case 4: { s += coe_core::to_string_hex( *(int8_t*   )&value[0] ); s += " [ INT8   ]"; }break;
    case 5: { s += coe_core::to_string_hex( *(int16_t*  )&value[0] ); s += " [ INT16  ]"; }break;
    case 6: { s += coe_core::to_string_hex( *(int32_t*  )&value[0] ); s += " [ INT32  ]"; }break;
    case 7: { s += coe_core::to_string_hex( *(int64_t*  )&value[0] ); s += " [ INT64  ]"; }break;
  }
  
  return s;  
}

inline std::string to_string( const uint16_t index, const uint8_t subindex, const uint8_t sdotype, boost::array<uint8_t,8>& value, const double timeout )
{
  std::string s = coe_core::to_string_hex( index ) + ":" + std::to_string( subindex ) + ", value: " + to_string( sdotype, value ) +", timeout: " + std::to_string( timeout );
  
  return s;  
}
 

/**
 * @brief format the Module ID
 *
 * @param model
 * @param address
 * @param separator
 * @return std::string
 */
inline std::string uniqueId(const std::string &model, const int address, const std::string &separator = "__")
{
  return model + separator + std::to_string(address);
}

/**
 * @brief Get the Node Unique I Ds object
 *
 * @param module_addresses_map
 * @return std::vector<std::string>
 */
inline std::vector<std::string> getNodeUniqueIDs(const std::map<int, std::string> &module_addresses_map)
{
  std::vector<std::string> mapped_names;
  mapped_names.resize(module_addresses_map.size());
  std::transform(module_addresses_map.begin(),
                 module_addresses_map.end(),
                 mapped_names.begin(),
                 [](const std::pair<int, std::string> &m) { return uniqueId(m.second, m.first); });

  return mapped_names;
}


inline std::map<int, std::string> getAddressUniqueIdMap(const std::map<int, std::string> &module_addresses_map)
{
  std::map<int, std::string> sorted_names;
  for (auto const &module : module_addresses_map)
  {
    sorted_names[module.first] = uniqueId(module.second, module.first);
  }

  return sorted_names;
}

inline bool checkAddress(const std::map<int, std::string> &module_addresses_map, int addr)
{
  return module_addresses_map.find(addr) != module_addresses_map.end();
  return false;
}

inline int checkModuleName(const std::map<int, std::string> &module_addresses_map,
                           const std::string &id,
                           std::string &what)
{
  int ret = -1;

  auto it = std::find_if(module_addresses_map.begin(),
                         module_addresses_map.end(),
                         [&id](const std::pair<int, std::string> &m) { return uniqueId(m.second, m.first) == id; });

  ret = (it != module_addresses_map.end()) ? it->first : -1;

  what = "";
  if (ret == -1)
  {
    what = __PRETTY_FUNCTION__ + ("Requested: '" + id + "', while the name available are:\n");
    for (auto const &s : getAddressUniqueIdMap(module_addresses_map))
    {
      what += "- " + std::to_string(s.first) + "# '" + s.second + "'\n";
    }
  }
  return ret;
}

}  // namespace coe_master

#endif  /* CNR_SOEM_COE_MASTER_INCLUDE_COE_MASTER_MODULES_UTILITIES_COMMON */
