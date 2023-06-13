/**
 *
 * @file coe_dictionary.h
 * @brief Namespace storing the structures to describe the dictionary according to CiA CANopen specifications
 *
 */

#ifndef COE_CORE__COE_COB_TYPES__H
#define COE_CORE__COE_COB_TYPES__H

#include <boost/algorithm/string.hpp>

#include <typeinfo>
#include <typeindex>
#include <tuple>
#include <map>
#include <algorithm>
#include <string>
#include <exception>
#include <cstdint>
#include <memory>
#include <ethercattype.h>

namespace coe_core 
{
  
struct uint24_t
{
    static constexpr std::size_t NBITS = std::numeric_limits<unsigned char>::digits;
    static_assert( NBITS == 8, "byte must be an octet" ) ;

    uint24_t() = default ;
    uint24_t(const char32_t& value )
    {
        *this = value;
    }

    const uint24_t& operator=(const char32_t& value)
    {
        char32_t _value = value;
        // assert value within range of uint24_t
        lsb = _value & 0xff ;
        _value >>= NBITS ;
        midb = _value & 0xff ;
        msb = _value >> NBITS ;
        return *this;
    }

    operator char32_t() const { return ( msb << (NBITS*2) ) + ( midb << NBITS ) + lsb ; }

    std::uint8_t msb = 0 ;
    std::uint8_t midb = 0 ;
    std::uint8_t lsb = 0 ;
};


template<ec_datatype ECT_TYPE> struct std_type 
{ 
  using type = 
  typename std::conditional<ECT_TYPE==ECT_BOOLEAN    , bool        ,
  typename std::conditional<ECT_TYPE==ECT_INTEGER8   , int8_t      ,
  typename std::conditional<ECT_TYPE==ECT_INTEGER16  , int16_t     ,
  typename std::conditional<ECT_TYPE==ECT_INTEGER32  , int32_t     ,
  typename std::conditional<ECT_TYPE==ECT_INTEGER64  , int64_t     ,
  typename std::conditional<ECT_TYPE==ECT_UNSIGNED8  , uint8_t     ,
  typename std::conditional<ECT_TYPE==ECT_UNSIGNED16 , uint16_t    ,
  typename std::conditional<ECT_TYPE==ECT_UNSIGNED24 , uint24_t    ,
  typename std::conditional<ECT_TYPE==ECT_UNSIGNED32 , uint32_t    ,
  typename std::conditional<ECT_TYPE==ECT_UNSIGNED64 , uint64_t    ,
  typename std::conditional<ECT_TYPE==ECT_REAL32     , double      ,
  typename std::conditional<ECT_TYPE==ECT_REAL64     , long double ,
  typename std::conditional<ECT_TYPE==ECT_BIT1       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT2       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT3       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT4       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT5       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT6       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT7       , bool        ,
  typename std::conditional<ECT_TYPE==ECT_BIT8       , bool        , void
  >::type >::type >::type >::type >::type >::type >::type >::type >::type >::type 
  >::type >::type >::type >::type >::type >::type >::type >::type >::type >::type;
  
  static constexpr type value = 0x0;
};

template<ec_datatype data>
constexpr std::tuple<std::type_index, std::size_t, std::size_t, std::string> make_info(std::size_t superside_size_bit = 0) 
{
  return std::make_tuple( std::type_index( typeid(typename std_type<data>::type) ), 
    (superside_size_bit != 0 ? superside_size_bit : 8 * sizeof(typename std_type<data>::type) ),
    (superside_size_bit != 0 ? (1 + superside_size_bit / 8u) : 8 * sizeof(typename std_type<data>::type) ), typeid(typename std_type<data>::type).name() );
}

struct EcTypes
{

  typedef std::tuple<std::type_index, std::size_t, std::size_t, std::string>  info; 
  typedef std::map<ec_datatype, info>                               map;
  typedef std::tuple<std::string, std::string, std::string>         stringequivalent; 
  typedef std::map<std::string,stringequivalent>                    stringmap; 

  static map& Map()
  {
    static map allowed= { { ECT_BOOLEAN   , make_info<ECT_BOOLEAN   >(1) }
                        , { ECT_UNSIGNED8 , make_info<ECT_UNSIGNED8 >() }
                        , { ECT_UNSIGNED16, make_info<ECT_UNSIGNED16>() }
                        , { ECT_UNSIGNED32, make_info<ECT_UNSIGNED32>() }
                        , { ECT_UNSIGNED64, make_info<ECT_UNSIGNED64>() }
                        , { ECT_INTEGER8  , make_info<ECT_INTEGER8  >() }
                        , { ECT_INTEGER16 , make_info<ECT_INTEGER16 >() }
                        , { ECT_INTEGER32 , make_info<ECT_INTEGER32 >() }
                        , { ECT_INTEGER64 , make_info<ECT_INTEGER64 >() } 
                        , { ECT_UNSIGNED24, make_info<ECT_UNSIGNED24>() } 
    };       
    return allowed;
  }
  
  static stringmap& StringMap() 
  {
    static stringmap eq={ { typeid(bool     ).name(), std::make_tuple( "bool"     , "boolean"    , "bool"   ) }
                        , { typeid(uint8_t  ).name(), std::make_tuple( "uint8_t"  , "uint8" , "unsigned8"   ) }
                        , { typeid(uint16_t ).name(), std::make_tuple( "uint16_t" , "uint16", "unsigned16"  ) }
                        , { typeid(uint32_t ).name(), std::make_tuple( "uint32_t" , "uint32", "unsigned32"  ) }
                        , { typeid(uint64_t ).name(), std::make_tuple( "uint64_t" , "uint64", "unsigned64"  ) }
                        , { typeid(int8_t   ).name(), std::make_tuple( "int8_t"   , "int8"  , "integer8"    ) }
                        , { typeid(int16_t  ).name(), std::make_tuple( "int16_t"  , "int16" , "integer16"   ) }
                        , { typeid(int32_t  ).name(), std::make_tuple( "int32_t"  , "int32" , "integer32"   ) }
                        , { typeid(int64_t  ).name(), std::make_tuple( "int64_t"  , "int64" , "integer64"   ) } 
                        , { typeid(uint24_t ).name(), std::make_tuple( "uint24_t" , "uint24" ,"unsigned24"  ) } };
    return eq;
  }
};

inline ec_datatype getType( const std::string& s )
{
  std::string ss = s;
  boost::to_lower( ss );
  const EcTypes::stringmap&   eq  = EcTypes::StringMap();
  const EcTypes::map&         map = EcTypes::Map();
  auto et = eq.end();
  for( EcTypes::stringmap::const_iterator jt = eq.begin(); jt != eq.end(); jt++  ) 
  {
    if( ( std::get<0>( jt->second ) == ss )
      ||( std::get<1>( jt->second ) == ss )
      ||( std::get<2>( jt->second ) == ss ) )
    {
      et = jt;
      break;
    }
  }
  if( et != eq.end() )
  {
    for( auto mt = map.begin(); mt != map.end(); mt++ )
    {
      if( et->first == std::get<3>(mt->second) )
        return mt->first;
    }
  }
  
  throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
      "Type '"+ss+"' is not supported. Abort.").c_str());

}
  

template<typename T>
inline ec_datatype getType( ) 
{
  for( auto const & it : EcTypes::Map() )
  {
    if( std::type_index( typeid(T) ) == std::get<0>( it.second ) )
      return it.first;
  }
  throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
    "Extract Type Failed: asked for type: '" + typeid( T ).name() + std::string("' Abort.") ).c_str() );
}


inline std::string getType(  const ec_datatype& t )
{
  const EcTypes::map& map = EcTypes::Map();
  auto  mt = map.find(t);
  if( mt == map.end() )
  {
      throw std::runtime_error(std::string(std::string(__PRETTY_FUNCTION__) + "@" + std::to_string(__LINE__) +
      "Extract Type Failed. Asked for type: 'ec_datatype'. Abort.") .c_str() );
  }

  const EcTypes::stringmap&   eq  = EcTypes::StringMap();
  auto et = eq.find( std::get< 3 >(mt->second) );
  return std::string( std::get< 2 >( et->second ) );
}


}  // namespace coe_core 

#endif  // COE_CORE__COE_COB_TYPES__H
