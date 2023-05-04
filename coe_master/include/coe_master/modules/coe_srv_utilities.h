
#ifndef __coe_srv_utilities__h__
#define __coe_srv_utilities__h__

#include <coe_core/coe_string_utilities.h>

namespace coe_master 
{


inline std::string to_string( const uint16_t index, const uint8_t subindex, const uint8_t sdotype)
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

inline std::string to_string( const uint16_t index, const uint8_t subindex, const uint8_t sdotype, boost::array<uint8_t,8>& value, const double timeout )
{
  std::string s = coe_core::to_string_hex( index ) + ":" + std::to_string( subindex ) + ", value: " + to_string( sdotype, value ) +", timeout: " + std::to_string( timeout );
  
  return s;  
}
 


}

#endif
