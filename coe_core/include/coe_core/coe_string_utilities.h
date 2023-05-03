#ifndef COE_CORE_INCLUDE_COE_CORE_COE_STRING_UTILITIES
#define COE_CORE_INCLUDE_COE_CORE_COE_STRING_UTILITIES

#include <bitset>
#include <iomanip>
#include <string>
#include <sstream>
#include <ethercattype.h>


namespace coe_core 
{
  

std::string to_string    ( const ec_datatype&          in, bool fill_space );
std::string to_string    ( const ec_state&             in );
std::string to_vstring   ( const ec_state&             in );
std::string dtype2string ( const uint16_t&             dtype, bool fill_spaces );

template< typename T >
std::string to_string_hex( const T& i )
{
//   size_t sz  = sizeof( T );
//   size_t u8  = sizeof( uint8_t );
//   size_t u16 = sizeof( uint16_t );
//   size_t u32 = sizeof( uint32_t );
//   size_t u64 = sizeof( uint64_t );
//   

//   if     ( sz == u8 )  stream << "cosa succede? " << i;
//   else if( sz == u16 ) stream << "0x" << std::hex << std::setfill ( '0' ) << std::setw(2) << (*(const uint16_t*)&i);
//   else if( sz == u32 ) stream << "0x" << std::hex << std::setfill ( '0' ) << std::setw(4) << (*(const uint32_t*)&i);
//   else if( sz == u64 ) stream << "0x" << std::hex << std::setfill ( '0' ) << std::setw(8) << (*(const uint64_t*)&i);
//   else stream << "pippo!!! "<< sz << " i:" << i ;
    std::stringstream stream;
    stream << "0x" << std::hex << std::setfill ( '0' ) << std::setw( sizeof(T) ) << i;
    return stream.str();
}

template<>
inline std::string to_string_hex( const uint8_t& i )
{
    char buffer[16] = {0};
    sprintf(buffer,"0x%2x", i );
    return std::string(buffer);
}


template<>
inline std::string to_string_hex( const int8_t& i )
{
    char buffer[16] = {0};
    sprintf(buffer,"0x%02x", i );
    return std::string(buffer);
}

template< typename C >
std::string to_string_bin( const C& val_int )
{
  size_t sz  = sizeof( C );
  size_t u8  = sizeof( uint8_t );
  size_t u16 = sizeof( uint16_t );
  size_t u32 = sizeof( uint32_t );
  size_t u64 = sizeof( uint64_t );
  
  std::string ret;
  if( sz == u8 )
  {
    std::bitset< sizeof(uint8_t)*8 > val ( *(uint8_t*)&val_int );
    ret = "0b" + val.to_string();
  } 
  else if( sz == u16 )
  {
    std::bitset< sizeof(uint16_t)*8 > val( *(uint16_t*)&val_int );
    ret = "0b" + val.to_string();
  }
  else if( sz == u32 )
  {
    std::bitset< sizeof(uint32_t)*8> val( *(uint32_t*)&val_int );
    ret = "0b" + val.to_string();
  }
  else if( sz == u64 )
  {
    std::bitset< sizeof(uint64_t)*8> val( *(uint64_t*)&val_int );
    ret = "0b" + val.to_string();
  }

  return ret;
}

}  // namespace coe_core 

#endif  /* COE_CORE_INCLUDE_COE_CORE_COE_STRING_UTILITIES */
