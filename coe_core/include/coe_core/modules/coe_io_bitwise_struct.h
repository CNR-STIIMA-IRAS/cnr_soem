#ifndef COE_CORE__MODULES__COE_IO_BITWISE_STRUCT__H
#define COE_CORE__MODULES__COE_IO_BITWISE_STRUCT__H

#include <cstdint>
#include <string>

namespace coe_core
{
  
namespace modules
{

typedef struct 
{
    int16_t   value[ 8 ];
}__attribute__ ( ( packed ) ) rpdo_ai8_compact_t ;   // INPUT per il PC


typedef union 
{
    uint16_t   value;
    struct 
    {
        uint8_t ch0 : 1 ;
        uint8_t ch1 : 1 ;
        uint8_t ch2 : 1 ;
        uint8_t ch3 : 1 ;
        uint8_t ch4 : 1 ;
        uint8_t ch5 : 1 ;
        uint8_t ch6 : 1 ;
        uint8_t ch7 : 1 ;
        uint8_t ch8 : 1 ;
        uint8_t ch9 : 1 ;
        uint8_t chA : 1 ;
        uint8_t chB : 1 ;
        uint8_t chC : 1 ;
        uint8_t chD : 1 ;
        uint8_t chE : 1 ;
        uint8_t chF : 1 ;
    } bits;
}__attribute__ ( ( packed ) ) rpdo_di16_compact_t;   // INPUT per il PC

typedef rpdo_di16_compact_t  rpdo_do16_compact_t;    // OUTPUT per il PC

typedef union 
{
    uint8_t   value;
    struct 
    {
        uint8_t ch0 : 1 ;
        uint8_t ch1 : 1 ;
        uint8_t ch2 : 1 ;
        uint8_t ch3 : 1 ;
        uint8_t ch4 : 1 ;
        uint8_t ch5 : 1 ;
        uint8_t ch6 : 1 ;
        uint8_t ch7 : 1 ;
    } bits;
} __attribute__ ( ( packed ) ) rpdo_di18_compact_t;   // INPUT per il PC

typedef rpdo_di8_compact_t  rpdo_do8_compact_t;    // OUTPUT per il PC

void copy( const rpdo_ai8_compact_t& rhs, rpdo_ai8_compact_t* lhs );
void copy( const rpdo_ai8_compact_t* rhs, rpdo_ai8_compact_t* lhs );
rpdo_ai8_compact_t& operator<<(rpdo_ai8_compact_t& lhs, const rpdo_ai8_compact_t& rhs);

void copy( const rpdo_di16_compact_t& rhs, rpdo_di16_compact_t* lhs );
void copy( const rpdo_di16_compact_t* rhs, rpdo_di16_compact_t* lhs );
rpdo_di16_compact_t& operator<<(rpdo_di16_compact_t& lhs, const rpdo_di16_compact_t& rhs);

void copy( const rpdo_di8_compact_t& rhs, rpdo_di8_compact_t* lhs );
void copy( const rpdo_di8_compact_t* rhs, rpdo_di8_compact_t* lhs );
rpdo_di8_compact_t& operator<<(rpdo_di8_compact_t& lhs, const rpdo_di8_compact_t& rhs);

void copy( const el1809_rpdo_t& rhs, el1809_rpdo_t* lhs );
void copy( const el1809_rpdo_t* rhs, el1809_rpdo_t* lhs );
el1809_rpdo_t& operator<<(el1809_rpdo_t& lhs, const el1809_rpdo_t& rhs);

void copy( const el2809_tpdo_t& rhs, el2809_tpdo_t* lhs );
void copy( const el2809_tpdo_t* rhs, el2809_tpdo_t* lhs );
el2809_tpdo_t& operator<<(el2809_tpdo_t& lhs, const el2809_tpdo_t& rhs);


double el3008_digit2volts( int16_t i_val);


}  // namespace modules
}  // namespace coe_core



#include <coe_core/modules/impl/coe_io_bitwise_struct_impl.h>

#endif // COE_CORE__MODULES__COE_IO_BITWISE_STRUCT__H
