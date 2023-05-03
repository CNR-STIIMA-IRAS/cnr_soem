#ifndef COE_CORE__DS402__SDO_DICTIONARY__H
#define COE_CORE__DS402__SDO_DICTIONARY__H

#include <cstdint>
#include <coe_core/coe_sdo.h> // uint16_t, ...

namespace coe_core
{
namespace ds402
{
    
inline coe_core::DataObjectEntry<uint16_t> CONTROL_WORD                     ( uint16_t value = 0x0 ) { return coe_core::DataObjectEntry<uint16_t> ( 0x6040, 0, "CONTROL WORD" , value )                      ;}
inline coe_core::DataObjectEntry<uint16_t> STATUS_WORD                      ( uint16_t value = 0x0 ) { return coe_core::DataObjectEntry<uint16_t> ( 0x6041, 0, "STATUS WORD"  , value )                      ;}
inline coe_core::DataObjectEntry<uint32_t> DIGITAL_INPUT                    ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x60FD, 0, "DIGITAL INPUT", value )                      ;}
inline coe_core::DataObjectEntry<uint32_t> DIGITAL_OUTPUT                   ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x60FD, 0, "DIGITAL INPUT", value )                      ;}
inline coe_core::DataObjectEntry<int32_t > POSITION_ACTUAL_VALUE            ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x6064, 0, "Position Actual Value", value )              ;}
inline coe_core::DataObjectEntry<uint32_t> SUPPORTED_DRIVE_MODES            ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6502, 0, "SUPPORTED_DRIVE_MODES", value )              ;}
inline coe_core::DataObjectEntry<int8_t  > MODES_OF_OPERATION               ( int8_t   value = 0x0 ) { return coe_core::DataObjectEntry<int8_t  > ( 0x6060, 0, "MODES_OF_OPERATION", value )                 ;}
inline coe_core::DataObjectEntry<int8_t  > MODES_OF_OPERATION_DISPLAY       ( int8_t   value = 0x0 ) { return coe_core::DataObjectEntry<int8_t  > ( 0x6061, 0, "MODES_OF_OPERATION_DISPLAY", value )         ;}
inline coe_core::DataObjectEntry<int32_t > PROFILE_TARGET_POSITION          ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x607A, 0, "PROFILE_TARGET_POSITION", value )            ;}
inline coe_core::DataObjectEntry<uint32_t> FOLLOWING_ERROR_WINDOW           ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6065, 0, "Following error window", value )             ;}
inline coe_core::DataObjectEntry<uint16_t> FOLLOWING_ERROR_WINDOW_TIME_OUT  ( uint16_t value = 0x0 ) { return coe_core::DataObjectEntry<uint16_t> ( 0x6066, 0, "Following error time out", value );}
inline coe_core::DataObjectEntry<uint32_t> POSITION_WINDOW                  ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6067, 0, "POSITION_WINDOW", value );}
inline coe_core::DataObjectEntry<uint16_t> POSITION_WINDOW_TIME             ( uint16_t value = 0x0 ) { return coe_core::DataObjectEntry<uint16_t> ( 0x6068, 0, "POSITION_WINDOW_TIME", value );}
inline coe_core::DataObjectEntry<int32_t > MIN_POSITION_RANGE_LIMIT         ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x607B, 1, "MIN_POSITION_RANGE_LIMIT", value );}
inline coe_core::DataObjectEntry<int32_t > MAX_POSITION_RANGE_LIMIT         ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x607B, 2, "MAX_POSITION_RANGE_LIMIT", value );}
inline coe_core::DataObjectEntry<int32_t > MIN_SOFTWARE_POSITION_LIMIT      ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x607D, 1, "MIN_SOFTWARE_POSITION_LIMIT", value );}
inline coe_core::DataObjectEntry<int32_t > MAX_SOFTWARE_POSITION_LIMIT      ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x607D, 2, "MAX_SOFTWARE_POSITION_LIMIT", value );}
inline coe_core::DataObjectEntry<uint32_t> MAXIMUM_PROFILE_VELOCITY         ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x607F, 0, "MAXIMUM_PROFILE_VELOCITY", value ) ;}
inline coe_core::DataObjectEntry<uint32_t> PROFILED_ACCELERATION            ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6083, 0, "PROFILED_ACCELERATION", value ) ;}
inline coe_core::DataObjectEntry<uint32_t> PROFILED_DECELERATION            ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6084, 0, "PROFILED_DECELERATION", value ) ;}
inline coe_core::DataObjectEntry<int8_t  > HOMING_METHOD                    ( int8_t   value = 0x0 ) { return coe_core::DataObjectEntry<int8_t  > ( 0x6098, 0, "HOMING METHOD", value ) ;}
inline coe_core::DataObjectEntry<uint32_t> HOMING_SPEED_SEARCH_SWITCH       ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6099, 1, "HOMING SPEED DURING SEARCH FOR SWITCH", value ) ;}
inline coe_core::DataObjectEntry<uint32_t> HOMING_SPEED_SEARCH_ZERO         ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6099, 2, "HOMING SPEED DURING SEARCH FOR ZERO", value ) ;}
inline coe_core::DataObjectEntry<uint32_t> HOMING_ACCELERATION              ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x609A, 0, "HOMING ACCELERATION", value ) ;}
inline coe_core::DataObjectEntry<int32_t > HOME_OFFSET                      ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x607C, 0, "HOME OFFSET", value ) ;}
inline coe_core::DataObjectEntry<int16_t > QUICK_STOP_OPTION_CODE           ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x605A, 0, "QUICK_STOP_OPTION_CODE", value ) ;}
inline coe_core::DataObjectEntry<int16_t > SHUTDOWN_OPTION_CODE             ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x605B, 0, "SHUTDOWN_OPTION_CODE", value ) ;}
inline coe_core::DataObjectEntry<int16_t > DISABLE_OPERATION_OPTION_CODE    ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x605C, 0, "DISABLE_OPERATION_OPTION_CODE", value );}
inline coe_core::DataObjectEntry<int16_t > HALT_OPTION_CODE                 ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x605D, 0, "HALT_OPTION_CODE", value );}
inline coe_core::DataObjectEntry<int16_t > FAULT_REACTION_OPTION_CODE       ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x605E, 0, "FAULT_REACTION_OPTION_CODE", value );}
inline coe_core::DataObjectEntry<uint32_t> QUICK_STOP_DECELERATION          ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6085, 0, "QUICK_STOP_DECELERATION", value );}
inline coe_core::DataObjectEntry<int16_t > ANALOG_1                         ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x2205, 1, "Analog 1", value );}
inline coe_core::DataObjectEntry<int16_t > ANALOG_2                         ( int16_t  value = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x2205, 2, "Analog 2", value );}
//inline coe_core::DataObjectEntry<float   > USER_FLOAT_1                   e d<float (  value = 0x0 ) { return coe_core::DataObjectEntry<float     = (  value = 0x0 ) { return coe_core::DataObjectEntry<float   >( 0x2F01, 1, "User Float 1", value ) ;}
inline coe_core::DataObjectEntry<int32_t > CSP_TORQUE_FFWD                  ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x60B2, 0, "CSP Torque offset", value ) ;}
inline coe_core::DataObjectEntry<int32_t > CSP_VELOCITY_FFWD                ( int32_t  value = 0x0 ) { return coe_core::DataObjectEntry<int32_t > ( 0x60B1, 0, "CSP Velocity offset", value ) ;}
inline coe_core::DataObjectEntry<uint32_t> MOTOR_RATE_CURRENT               ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6075, 0, "Motor Rate Current", value );}
inline coe_core::DataObjectEntry<uint32_t> MOTOR_RATE_TORQUE                ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6076, 0, "Motor Rate Torque", value );}
inline coe_core::DataObjectEntry<uint32_t> ENCODER_INCREMENTS               ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x608F, 1, "Encoder Increments", value );}
inline coe_core::DataObjectEntry<uint32_t> MOTOR_REVOLUTIONS                ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x608F, 2, "Motor revolutions", value );}
inline coe_core::DataObjectEntry<uint16_t> MAX_TARGET_TORQUE                ( uint16_t value = 0x0 ) { return coe_core::DataObjectEntry<uint16_t> ( 0x6072, 0, "Max target torque", value );}
inline coe_core::DataObjectEntry<uint32_t> TARGET_TORQUE_SLOPE              ( uint32_t value = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x6087, 0, "target torque slope", value );}
inline coe_core::DataObjectEntry<uint8_t > INTERPOLATION_TIME_PERIOD        ( uint8_t  value = 0x0 ) { return coe_core::DataObjectEntry<uint8_t>  ( 0x60C2, 1, "Interpolation Cycle Time", value );}
inline coe_core::DataObjectEntry<int8_t  > INTERPOLATION_TIME_PERIOD_BASE   ( int8_t   value = 0x0 ) { return coe_core::DataObjectEntry<int8_t>   ( 0x60C2, 2, "Interpolation Cycle Time Base", value );}


}
}

#endif
