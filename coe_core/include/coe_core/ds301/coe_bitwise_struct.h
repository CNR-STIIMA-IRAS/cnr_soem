#ifndef COE_CORE__DS301__COE_BITWISE_STRUCT__H
#define COE_CORE__DS301__COE_BITWISE_STRUCT__H

#include <cstdint>
#include <string>
#include <ostream>
#include <vector>

namespace coe_core
{
namespace ds301
{
  
  /**
    * @struct error_register_t
    * Bit-to-bit sescription of ERROR_REGISTER
    */
  typedef union {
    uint8_t   value;
    struct {
      unsigned short int generic_error            : 1;
      unsigned short int current                  : 1;
      unsigned short int voltage                  : 1;
      unsigned short int temperature              : 1;
      unsigned short int communication_error      : 1;
      unsigned short int device_profile_specific  : 1;
      unsigned short int reserved                 : 1;
      unsigned short int manufacturer_specific    : 1;
    } bits;
  } __attribute__(( packed )) error_register_t;
  
    
    /**
     * @struct emergency_events_t
     * Bit-to-bit sescription of EMERGENCY_EVENTS
     * This object selects events as the cause for transmitting emergency objects.
     * The driving event definition for an emergency is a bit field, as follows:
     */
    typedef union {
      uint16_t   value;
      struct {
        unsigned short int CAN_message_lost                         : 1; ///< (corrupted or overrun) 0x8110 0x11
        unsigned short int protocol_error                           : 1; ///< (unrecognized NMT request) 0x8200 0x11
        unsigned short int attempt_to_access_an_unconfigured_rpdo   : 1; ///< 0x8210 0x21
        unsigned short int heartbeat_event                          : 1; ///< 0x8130 0x11
        unsigned short int fatal_cpu_error                          : 1; ///< CPU error: stack overflow 0x6180 0x81
        unsigned short int user_program_aborted_by_an_error         : 1; ///< 0x6200 0x81
        unsigned short int request_by_user_program__emit_function   : 1; ///< 0xFF01 0x81
        unsigned short int motor_shut_down_by_fault                 : 1; ///< See Table 13-3 See Table 13-3
        unsigned short int rpdo_returned_an_error                   : 1; ///< during interpretation or a referenced motion failed to be performed. 0x6300 0x01
        unsigned short int ds402_ip_underflow                       : 1; ///< 0xFF02 0x21 Note: This object does not control certain emergency messages, such as PVT
        unsigned short int unnused                                  : 6; ///< unused
      } bits;
    } __attribute__((packed)) emergency_events_t;
    
    /**
     * @struct predefined_error_field_t
     * bit-to-bit description of PREDEFINED_ERROR_FIELD_1,PREDEFINED_ERROR_FIELD_2, PREDEFINED_ERROR_FIELD_3
     */
    typedef union {
      uint32_t   value;
      struct {
        unsigned short int error_code                               : 16; ///< (corrupted or overrun) 0x8110 0x11
        unsigned short int error_register                           : 8;  ///< (unrecognized NMT request) 0x8200 0x11
        unsigned short int manufacturer_specific_error_code         : 8;  ///< 0x8210 0x21
      } bits;
    } __attribute__((packed)) predefined_error_field_t;
    
    /**
     * @struct manufacturer_status_register_t
     * bit-to-bit description of MANUFACTURER_STATUS_REGISTER
     */
    typedef union {
      uint32_t   value;
      struct {
        unsigned short int problem                  : 1; //0
        unsigned short int problem_depenent         : 3; //1-3
        unsigned short int motor_on                 : 1; //4
        unsigned short int reference_mode           : 1; //5
        unsigned short int motor_failure_latched    : 1; //6
        unsigned short int unit_mode                : 3; // (UM) 7 - 9
        unsigned short int gain_scheduling_on       : 1; //10;
        unsigned short int homing_being_processed   : 1; //11
        unsigned short int program_running          : 1; //12
        unsigned short int current_limit_on         : 1; //(LC) 13
        unsigned short int motion_status_reflection : 2; // (MS) 14 - 15
        unsigned short int recorder_status          : 2; // 16-17, 0: Recorder inactive, no valid recorded data 1: Recorder waiting for a trigger event2: Recorder finished; valid data ready for use                         3: Recording now
        unsigned short int not_used                 : 6; // 18 - 23
        unsigned short int digital_hall_sensors     : 3; // 24 - 26
        unsigned short int CPU_status_not_ok        : 1; // 27
        unsigned short int stopped_by_a_limit       : 1; // 28 – RLS, FLS, Stop switch – or by a VH[3]/VL[3]
        unsigned short int error_in_user_program    : 1; // 29;
        unsigned short int not_used_2               : 2; // 30-31;
      } bits;
    } __attribute__((packed)) manufacturer_status_register_t;
    
    
    typedef union {
      uint32_t value;
      struct {
        unsigned short int motion_complete              : 1;
        unsigned short int main_homing_complete         : 1;
        unsigned short int auxiliary_homing_complete    : 1;
        unsigned short int motor_shut_down_by_exception : 1;
        unsigned short int motor_started                : 1;
        unsigned short int user_program_emit_command    : 1;
        unsigned short int os_interpreter_exec_complete : 1;
        unsigned short int digital_input                : 1;
        unsigned       int reserved                     : 23;
        unsigned short int binary_interpreter_complete  : 1;
      } bits;
    } __attribute__((packed)) pdo_events_t;
    
    
    std::string to_string(const error_register_t&                  in );
    std::string to_string(const manufacturer_status_register_t&    in );
    std::string to_string(const emergency_events_t&                in );
    std::string to_string(const predefined_error_field_t&          in );
    
    uint8_t&                        operator<<( uint8_t& lhs, const error_register_t& rhs);
    error_register_t&               operator<<( error_register_t& rhs, const uint8_t& lhs);
    uint16_t&                       operator<<( uint16_t& lhs, const emergency_events_t& rhs);
    emergency_events_t&             operator<<( emergency_events_t& rhs, const uint16_t& lhs);
    uint32_t&                       operator<<( uint32_t& lhs, const predefined_error_field_t& rhs);
    predefined_error_field_t&       operator<<( predefined_error_field_t& rhs, const uint32_t& lhs);
    uint32_t&                       operator<<( uint32_t& lhs, const manufacturer_status_register_t& rhs);
    manufacturer_status_register_t& operator<<( manufacturer_status_register_t& rhs, const uint32_t& lhs);
    
    std::ostream& operator<<(std::ostream& str, const error_register_t&                  in );
    std::ostream& operator<<(std::ostream& str, const manufacturer_status_register_t&    in );
    std::ostream& operator<<(std::ostream& str, const emergency_events_t&                in );
    std::ostream& operator<<(std::ostream& str, const predefined_error_field_t&          in );
    
}  // namespace ds301 
}  // namespace coe_core

#endif  // COE_CORE__DS301__COE_BITWISE_STRUCT__H
