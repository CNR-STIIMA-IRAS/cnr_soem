#ifndef COE_CORE__MODULES__COE_DRIVER_BITWISE_STRUCT__H
#define COE_CORE__MODULES__COE_DRIVER_BITWISE_STRUCT__H

#include <stdexcept>
#include <string>
#include <coe_core/ds402/coe_bitwise_struct.h>
#include <coe_core/ds402/coe_xfsm_symbols.h>

namespace coe_core
{
namespace modules 
{
   
    typedef struct {
        uint16_t status_word;                        ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox6041
        int32_t  position_actual_value;              ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox6064
        int32_t  velocity_sensor_actual_value;       ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox6069
        int16_t  torque_actual_value;                ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox6077
        uint32_t digital_input;                      ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox60FD
    }__attribute__ ( ( packed ) ) rpdo_full_t;   

    typedef struct {
        uint16_t control_word;                        ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox6040
        int16_t  target_torque;                       ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox6071
        int32_t  target_position;                     ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox607A
        int32_t  velocity_offset;                     ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox60B1
        int16_t  torque_offset;                       ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox60B2
        int32_t  target_velocity;                     ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox60FF
        uint32_t digital_output;                      ///< Elmo Motion Control CANopen DSP402 Implementation Guide - COB-ID Ox60FE
    }__attribute__ ( ( packed ) ) tpdo_full_t;  
    
    
    //////////////////////////////////////////////////////////////////////////7    
    void copy ( const rpdo_t&  rhs, rpdo_t*  lhs );
    void copy ( const rpdo_t*  rhs, rpdo_t*  lhs );
    rpdo_t&  operator<<(rpdo_t& lhs, const rpdo_t& rhs);
    
    void copy ( const tpdo_t& rhs, tpdo_t* lhs );
    void copy ( const tpdo_t* rhs, tpdo_t* lhs );
    tpdo_t& operator<<(tpdo_t& lhs, const tpdo_t& rhs);
    
}  // modules
}  // coe_core



namespace cnr 
{
namespace coe 
{
namespace driver
{

inline void copy ( const rpdo_t& rhs, rpdo_t* lhs )
{
    memcpy ( lhs, &rhs, sizeof ( rpdo_t ) );
}
inline void copy ( const rpdo_t* rhs, rpdo_t* lhs )
{
    memcpy ( lhs, rhs, sizeof ( rpdo_t ) );
}
inline rpdo_t& operator<< ( rpdo_t& lhs, const rpdo_t& rhs )
{
    copy ( rhs, &lhs );
    return lhs;
}
inline void copy ( const tpdo_t& rhs, tpdo_t* lhs )
{
    memcpy ( lhs, &rhs, sizeof ( tpdo_t ) );
}
inline void copy ( const tpdo_t* rhs, tpdo_t* lhs )
{
    memcpy ( lhs, rhs, sizeof ( tpdo_t ) );
}

}
}
}

#endif  // COE_CORE__MODULES__COE_DRIVER_BITWISE_STRUCT__H