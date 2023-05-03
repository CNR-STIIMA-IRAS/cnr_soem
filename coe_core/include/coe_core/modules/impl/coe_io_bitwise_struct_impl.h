#ifndef COE_CORE__MODULES__COE_IO_BITWISE_STRUCT_IMPL_H
#define COE_CORE__MODULES__COE_IO_BITWISE_STRUCT_IMPL_H


#include <string>
#include <coe_core/modules/coe_io_bitwise_struct.h>

namespace coe_core
{
  
namespace modules
{

void copy( const rpdo_ai8_compact_t& rhs, rpdo_ai8_compact_t* lhs ) {
    for (int iCh=0;iCh<8;iCh++)
        lhs->value[iCh] = rhs.value[iCh];                        // 0x6041
}
void copy( const rpdo_ai8_compact_t* rhs, rpdo_ai8_compact_t* lhs ) {
    for (int iCh=0;iCh<8;iCh++)
        lhs->value[iCh] = rhs->value[iCh];                        // 0x6041
}
rpdo_ai8_compact_t& operator<<(rpdo_ai8_compact_t& lhs, const rpdo_ai8_compact_t& rhs) {
    copy( rhs, &lhs );
    return lhs;
}
void copy( const rpdo_di16_compact_t& rhs, rpdo_di16_compact_t* lhs ) {
    std::memcpy( lhs, &rhs, sizeof( rpdo_di16_compact_t ));
}
void copy( const rpdo_di16_compact_t* rhs, rpdo_di16_compact_t* lhs ) {
    memcpy( lhs, rhs, sizeof( rpdo_di16_compact_t ));
}
rpdo_di16_compact_t& operator<<(rpdo_di16_compact_t& lhs, const rpdo_di16_compact_t& rhs) {
    memcpy( &lhs, &rhs, sizeof( rpdo_di16_compact_t ));
    return lhs;
}

void copy( const rpdo_di8_compact_t& rhs, rpdo_di8_compact_t* lhs ) {
    memcpy( lhs, &rhs, sizeof( rpdo_di8_compact_t ));
}
void copy( const rpdo_di8_compact_t* rhs, rpdo_di8_compact_t* lhs ) {
    memcpy( lhs, rhs, sizeof( rpdo_di8_compact_t ));
}
rpdo_di8_compact_t& operator<<(rpdo_di8_compact_t& lhs, const rpdo_di8_compact_t& rhs) {
    memcpy( &lhs, &rhs, sizeof( rpdo_di8_compact_t ));
    return lhs;
}


double el3008_digit2volts( int16_t i_val) {

    static const double  d_max_val   =  10.0;
    static const double  d_min_val   = -10.0;
    static const int16_t i_max_val   =  32767;
    static const int16_t i_min_val   = -32768;
    static const double  i_range_val = double(i_max_val) - double(i_min_val);
    double  d_range_val = d_max_val - d_min_val;

    double  perc        = double(i_val - i_min_val) / i_range_val;
    double  ret         = d_min_val + perc * d_range_val;

    return ret;
};


}  // namespace modules
}  // namespace coe_core


#endif  /* COE_CORE__MODULES__COE_IO_BITWISE_STRUCT_IMPL_H */
