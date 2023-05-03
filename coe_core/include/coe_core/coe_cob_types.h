/**
 *
 * @file coe_dictionary.h
 * @brief Namespace storing the structures to describe the dictionary according to CiA CANopen specifications
 *
 */

#ifndef CPE_CORE__COE_COB_TYPES__H
#define CPE_CORE__COE_COB_TYPES__H

#include <typeinfo>
#include <typeindex>
#include <tuple>
#include <map>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <string>
#include <assert.h>
#include <exception>
#include <cstring>
#include <memory>
#include <ethercattype.h>

namespace coe_core 
{
  
struct uint24_t
{
    static constexpr std::size_t NBITS = std::numeric_limits<unsigned char>::digits ;
    static_assert( NBITS == 8, "byte must be an octet" ) ;

    uint24_t() = default ;
    uint24_t( char32_t value )
    {
        // assert value within range of uint24_t
        lsb = value & 0xff ;
        value >>= NBITS ;
        midb = value & 0xff ;
        msb = value >> NBITS ;
    }

    operator char32_t() const { return ( msb << (NBITS*2) ) + ( midb << NBITS ) + lsb ; }

    std::uint8_t msb = 0 ;
    std::uint8_t midb = 0 ;
    std::uint8_t lsb = 0 ;
};

struct EcTypes
{

  typedef std::tuple<std::type_index, size_t, size_t, std::string>  info; 
  typedef std::map<ec_datatype, info>                               map;
  typedef std::tuple<std::string, std::string, std::string>         stringequivalent; 
  typedef std::map<std::string,stringequivalent>                    stringmap; 

  static map& Map()
  {
    static map allowed= { { ECT_BOOLEAN   , std::make_tuple( std::type_index( typeid(bool     ) ), 1                   , sizeof(bool    ), typeid(bool     ).name() ) }
                        , { ECT_UNSIGNED8 , std::make_tuple( std::type_index( typeid(uint8_t  ) ), 8 * sizeof(uint8_t ), sizeof(uint8_t ), typeid(uint8_t  ).name() ) }
                        , { ECT_UNSIGNED16, std::make_tuple( std::type_index( typeid(uint16_t ) ), 8 * sizeof(uint16_t), sizeof(uint16_t), typeid(uint16_t ).name() ) }
                        , { ECT_UNSIGNED32, std::make_tuple( std::type_index( typeid(uint32_t ) ), 8 * sizeof(uint32_t), sizeof(uint32_t), typeid(uint32_t ).name() ) }
                        , { ECT_UNSIGNED64, std::make_tuple( std::type_index( typeid(uint64_t ) ), 8 * sizeof(uint64_t), sizeof(uint64_t), typeid(uint64_t ).name() ) }
                        , { ECT_INTEGER8  , std::make_tuple( std::type_index( typeid(int8_t   ) ), 8 * sizeof(int8_t)  , sizeof(int8_t)  , typeid(int8_t   ).name() ) }
                        , { ECT_INTEGER16 , std::make_tuple( std::type_index( typeid(int16_t  ) ), 8 * sizeof(int16_t) , sizeof(int16_t) , typeid(int16_t  ).name() ) }
                        , { ECT_INTEGER32 , std::make_tuple( std::type_index( typeid(int32_t  ) ), 8 * sizeof(int32_t) , sizeof(int32_t) , typeid(int32_t  ).name() ) }
                        , { ECT_INTEGER64 , std::make_tuple( std::type_index( typeid(int64_t  ) ), 8 * sizeof(int64_t) , sizeof(int64_t) , typeid(int64_t  ).name() ) } 
                        , { ECT_UNSIGNED24, std::make_tuple( std::type_index( typeid(uint24_t ) ), 8 * sizeof(uint24_t), sizeof(uint24_t), typeid(uint24_t ).name() ) } 
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

#endif  // CPE_CORE__COE_COB_TYPES__H
