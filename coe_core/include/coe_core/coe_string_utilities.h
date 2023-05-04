#ifndef COE_CORE_INCLUDE_COE_CORE_COE_STRING_UTILITIES
#define COE_CORE_INCLUDE_COE_CORE_COE_STRING_UTILITIES

#include <bitset>
#include <iomanip>
#include <string>
#include <sstream>
#include <ethercattype.h>


namespace coe_core 
{

const char* RESET() { return "\033[0m"; }
const char* BLACK() { return "\033[30m"; }
const char* RED() { return "\033[31m"; }
const char* GREEN() { return "\033[32m"; }
const char* YELLOW() { return "\033[33m"; }
const char* BLUE() { return "\033[34m"; }
const char* MAGENTA() { return "\033[35m"; }
const char* CYAN() { return "\033[36m"; }
const char* WHITE() { return "\033[37m"; }
const char* BOLDBLACK() { return "\033[1m\033[30m"; }
const char* BOLDRED() { return "\033[1m\033[31m"; }
const char* BOLDGREEN() { return "\033[1m\033[32m"; }
const char* BOLDYELLOW() { return "\033[1m\033[33m"; }
const char* BOLDBLUE() { return "\033[1m\033[34m"; }
const char* BOLDMAGENTA() { return "\033[1m\033[35m"; }
const char* BOLDCYAN() { return "\033[1m\033[36m"; }
const char* BOLDWHITE() { return "\033[1m\033[37m"; }

const char* RST() { return RESET(); }
const char* BLK() { return BLACK(); }
const char* R() { return RED(); }
const char* G() { return GREEN(); }
const char* Y() { return YELLOW(); }
const char* BLE() { return BLUE(); }
const char* M() { return MAGENTA(); }
const char* C() { return CYAN(); }
const char* W() { return WHITE(); }
const char* BBLK() { return BOLDBLACK(); }
const char* BR() { return BOLDRED(); }
const char* BG() { return BOLDGREEN(); }
const char* BY() { return BOLDYELLOW(); }
const char* BBLE() { return BOLDBLUE(); }
const char* BM() { return BOLDMAGENTA(); }
const char* BC() { return BOLDCYAN(); }
const char* BW() { return BOLDWHITE(); }

std::string to_string    ( const ec_datatype&          in, bool fill_space );
std::string to_string    ( const ec_state&             in );
std::string to_vstring   ( const ec_state&             in );
std::string dtype2string ( const uint16_t&             dtype, bool fill_spaces );

template< typename T >
std::string to_string_hex( const T& i )
{
//   std::size_t sz  = sizeof( T );
//   std::size_t u8  = sizeof( uint8_t );
//   std::size_t u16 = sizeof( uint16_t );
//   std::size_t u32 = sizeof( uint32_t );
//   std::size_t u64 = sizeof( uint64_t );
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
  std::size_t sz  = sizeof( C );
  std::size_t u8  = sizeof( uint8_t );
  std::size_t u16 = sizeof( uint16_t );
  std::size_t u32 = sizeof( uint32_t );
  std::size_t u64 = sizeof( uint64_t );
  
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
