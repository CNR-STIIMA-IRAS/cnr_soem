#ifndef COE_CORE__DS301__SDO_DICTIONARY__H
#define COE_CORE__DS301__SDO_DICTIONARY__H

#include <cstdint>
#include <coe_core/coe_sdo.h>

namespace coe_core 
{
namespace ds301
{

inline coe_core::DataObjectEntry<uint8_t > ERROR_REGISTER               ( uint8_t  val = 0x0 ) { return coe_core::DataObjectEntry<uint8_t > ( 0x1001, 0, "Error register"            , val ); }  
inline coe_core::DataObjectEntry<uint32_t> MANUFACTURER_STATUS_REGISTER ( uint32_t val = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x1002, 0, "Man. Status register"      , val ); }  
inline coe_core::DataObjectEntry<uint8_t > PREDEFINED_ERROR_FIELD_N     ( uint8_t  val = 0x0 ) { return coe_core::DataObjectEntry<uint8_t > ( 0x1003, 0, "Number of error stored"    , val ); }  
inline coe_core::DataObjectEntry<uint32_t> PREDEFINED_ERROR_FIELD_1     ( uint32_t val = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x1003, 1, "Predefined Error Field 1"  , val ); }
inline coe_core::DataObjectEntry<uint32_t> PREDEFINED_ERROR_FIELD_2     ( uint32_t val = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x1003, 2, "Predefined Error Field 2"  , val ); }
inline coe_core::DataObjectEntry<uint32_t> PREDEFINED_ERROR_FIELD_3     ( uint32_t val = 0x0 ) { return coe_core::DataObjectEntry<uint32_t> ( 0x1003, 3, "Predefined Error Field 3"  , val ); }
inline coe_core::DataObjectEntry<int16_t > ABORT_CONNECTION_OPTION_CODE ( int16_t  val = 0x0 ) { return coe_core::DataObjectEntry<int16_t > ( 0x6007, 0, "Abort Connection Code"     , val ); }
//inline coe_core::DataObjectEntry<uint16_t> EMERGENCY_EVENTS             ( uint16_t val = 0x0 ) { return coe_core::DataObjectEntry<uint16_t> ( 0x2F21, 0, "Emergency events"          , val ); }

}  // namespace coe_core 
}  // namespace ds301

#endif  // COE_CORE__DS301__SDO_DICTIONARY__H
