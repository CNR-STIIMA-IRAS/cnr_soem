#ifndef COE_CORE__DS402__COE_BITWISE_STRUCT__H
#define COE_CORE__DS402__COE_BITWISE_STRUCT__H

#include <cstdint>
#include <string>

namespace coe_core
{
namespace ds402
{
    /**
    * @struct digital_inputs_t
    * Bit-to-bit sescription of DIGITAL_INPUT
    */
    typedef union {
      uint32_t value;
      struct {
        unsigned short int negative_limit_switch : 1;       ///< bit:0
        unsigned short int positive_limit_switch : 1;       ///< bit:1
        unsigned short int home_switch           : 1;       ///< bit:2
        unsigned short int interlock             : 1;       ///< bit:3
        unsigned short int reserved              : 12;      ///< bit:4-15
        unsigned short int digital_1             : 1;       ///< bit:16
        unsigned short int digital_2             : 1;       ///< bit:17
        unsigned short int digital_3             : 1;       ///< bit:18
        unsigned short int digital_4             : 1;       ///< bit:19
        unsigned short int digital_5             : 1;       ///< bit:20
        unsigned short int digital_6             : 1;       ///< bit:21
        unsigned short int manufacturer_specific : 10;      ///< bit:22-31
      } bits;
    } __attribute__((packed)) digital_inputs_t;
    
    /**
     * @struct status_word_t
     * @brief  Bit-to-bit sescription of STATUS_WORD (General Operation Mode)
     */
    typedef union {
      uint16_t value;
      struct {
        unsigned short int ready_to_switch_on       : 1;        ///< bit:0
        unsigned short int switched_on              : 1;        ///< bit:1
        unsigned short int operation_enabled        : 1;        ///< bit:2
        unsigned short int fault                    : 1;        ///< bit:3
        unsigned short int voltage_enabled          : 1;        ///< bit:4
        unsigned short int quick_stop               : 1;        ///< bit:5
        unsigned short int switch_on_disabled       : 1;        ///< bit:6
        unsigned short int warning                  : 1;        ///< bit:7
        unsigned short int manufacturer_specific    : 1;        ///< bit:8
        unsigned short int remote                   : 1;        ///< bit:9
        unsigned short int target_reached           : 1;        ///< bit:10
        unsigned short int internal_limit_active    : 1;        ///< bit:11
        unsigned short int operation_mode_specific  : 2;        ///< bit:12,13
        unsigned short int manufacturer_specific_2  : 2;        ///< bit:14,15
      } bits;
    } __attribute__((packed)) status_word_t;

    /**
     * @struct control_word_t
     * @brief Bit-to-bit sescription of CONTROL_WORD (General Operation Mode)
     */    
    typedef union {
      uint16_t value;
      struct {
        unsigned short int switch_on                : 1;            ///< bit:0
        unsigned short int enable_voltage           : 1;            ///< bit:1
        unsigned short int quick_stop               : 1;            ///< bit:2
        unsigned short int enable_operation         : 1;            ///< bit:3
        unsigned short int operation_specific       : 3;            ///< bit:4,5,6
        unsigned short int fault_reset              : 1;            ///< bit:7
        unsigned short int halt                     : 1;            ///< bit:8
        unsigned short int reserved                 : 2;            ///< bit:9,10
        unsigned short int manufacturer_specific    : 5;            ///< bit:11,12,13,14,15
      } bits;
    } __attribute__ ((packed)) control_word_t;
    
    
    /**
     * @struct homing_status_word_t
     * @brief Bit-to-bit sescription of HOMING_STATUS_WORD (Status word in the HOMING Operation Mode)
     */
    typedef union {
      uint16_t value;
      struct {
        unsigned short int ready_to_switch_on       : 1;        ///< bit:0
        unsigned short int switched_on              : 1;        ///< bit:1
        unsigned short int operation_enabled        : 1;        ///< bit:2
        unsigned short int fault                    : 1;        ///< bit:3
        unsigned short int voltage_enabled          : 1;        ///< bit:4
        unsigned short int quick_stop               : 1;        ///< bit:5
        unsigned short int switch_on_disabled       : 1;        ///< bit:6
        unsigned short int warning                  : 1;        ///< bit:7
        unsigned short int manufacturer_specific    : 1;        ///< bit:8
        unsigned short int remote                   : 1;        ///< bit:9
        unsigned short int target_reached           : 1;        ///< bit:10
        unsigned short int internal_limit_active    : 1;        ///< bit:11
        unsigned short int homing_attained          : 1;        ///< bit:12
        unsigned short int homing_error             : 1;        ///< bit:13
        unsigned short int manufacturer_specific_2  : 2;        ///< bits:14,15
      } bits;
    } __attribute__ ( ( packed ) ) homing_status_word_t;
    
    /**
     * @struct homing_control_word_t
     * @brief Bit-to-bit sescription of HOMING_CONTROL_WORD (Control word in the HOMING Operation Mode)
     */    
    typedef union {
      uint16_t value;
      struct {
        unsigned short int switch_on                : 1;            ///< bit:0 --- common to all the modality
        unsigned short int enable_voltage           : 1;            ///< bit:1 --- common to all the modality
        unsigned short int quick_stop               : 1;            ///< bit:2 --- common to all the modality
        unsigned short int enable_operation         : 1;            ///< bit:3 --- common to all the modality
        unsigned short int home_operation_start     : 1;            ///< bit:4
        unsigned short int reserved_2               : 2;            ///< bit:5,6
        unsigned short int fault_reset              : 1;            ///< bit:7 --- common to all the modality
        unsigned short int halt                     : 1;            ///< bit:8 
        unsigned short int reserved                 : 2;            ///< bit:9,10 --- common to all the modality
        unsigned short int manufacturer_specific    : 5;            ///< bit:11,12,13,14,15 --- common to all the modality
      } bits;
    } __attribute__ ( ( packed ) ) homing_control_word_t;
    
    /**
     * @struct profiled_vel_status_word_t
     * @brief Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in PROFILED VELOCITY MODE
     */
    typedef union {
      uint16_t value;
      struct {
        unsigned short int ready_to_switch_on       : 1;        ///< bit:0
        unsigned short int switched_on              : 1;        ///< bit:1
        unsigned short int operation_enabled        : 1;        ///< bit:2
        unsigned short int fault                    : 1;        ///< bit:3
        unsigned short int voltage_enabled          : 1;        ///< bit:4
        unsigned short int quick_stop               : 1;        ///< bit:5
        unsigned short int switch_on_disabled       : 1;        ///< bit:6
        unsigned short int warning                  : 1;        ///< bit:7
        unsigned short int manufacturer_specific    : 1;        ///< bit:8
        unsigned short int remote                   : 1;        ///< bit:9
        unsigned short int target_reached           : 1;        ///< bit:10
        unsigned short int internal_limit_active    : 1;        ///< bit:11
        unsigned short int speed                    : 1;        ///< bit:12
        unsigned short int maximum_slippage         : 1;        ///< bit:13
        unsigned short int manufacturer_specific_2  : 2;        ///< bit:14,15
      } bits;
    } __attribute__ ( ( packed ) ) profiled_vel_status_word_t;
    
    
    /**
     * @struct profiled_vel_status_word_t
     * Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in PROFILED VELOCITY MODE
     */    
    typedef union {
      uint16_t value;
      struct {
        unsigned short int switch_on                : 1;            ///< bit:0 --- common to all the modality
        unsigned short int enable_voltage           : 1;            ///< bit:1 --- common to all the modality
        unsigned short int quick_stop               : 1;            ///< bit:2 --- common to all the modality
        unsigned short int enable_operation         : 1;            ///< bit:3  --- common to all the modality
        unsigned short int rfg_enable               : 1;            ///< bit:4
        unsigned short int rfg_unlock               : 1;            ///< bit:5 
        unsigned short int rfg_use_ref              : 1;            ///< bit:6
        unsigned short int fault_reset              : 1;            ///< bit:7 --- common to all the modality
        unsigned short int halt                     : 1;            ///< bit:8 
        unsigned short int reserved                 : 2;            ///< bit:9,10 --- common to all the modality
        unsigned short int manufacturer_specific    : 5;            ///< bit:11,12,13,14,15 --- common to all the modality
      } bits;
    } __attribute__(( packed )) profiled_vel_control_word_t;

  typedef union {
        uint16_t value;
        struct {
          unsigned short int ready_to_switch_on       : 1;        ///< bit:0
          unsigned short int switched_on              : 1;        ///< bit:1
          unsigned short int operation_enabled        : 1;        ///< bit:2
          unsigned short int fault                    : 1;        ///< bit:3
          unsigned short int voltage_enabled          : 1;        ///< bit:4
          unsigned short int quick_stop               : 1;        ///< bit:5
          unsigned short int switch_on_disabled       : 1;        ///< bit:6
          unsigned short int warning                  : 1;        ///< bit:7
          unsigned short int manufacturer_specific    : 1;        ///< bit:8
          unsigned short int remote                   : 1;        ///< bit:9
          unsigned short int target_reached           : 1;        ///< bit:10
          unsigned short int internal_limit_active    : 1;        ///< bit:11
          unsigned short int set_new_point_acknowledge: 1;        ///< bit:12
          unsigned short int following_error          : 1;        ///< bit:13
          unsigned short int manufacturer_specific_2  : 2;        ///< bit:14,15
        } bits;
      } __attribute__ ( ( packed ) ) profiled_pos_status_word_t;


      /**
       * @struct profiled_vel_status_word_t
       * Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in PROFILED VELOCITY MODE
       */
      typedef union {
        uint16_t value;
        struct {
          unsigned short int switch_on                 : 1;   ///< bit:0 --- common to all the modality
          unsigned short int enable_voltage            : 1;   ///< bit:1 --- common to all the modality
          unsigned short int quick_stop                : 1;   ///< bit:2 --- common to all the modality
          unsigned short int enable_operation          : 1;   ///< bit:3  --- common to all the modality
          unsigned short int new_setpoint              : 1;   ///< bit:4
          unsigned short int change_set_immediately    : 1;   ///< bit:5
          unsigned short int absolute_relative_movement: 1;   ///< bit:6
          unsigned short int fault_reset               : 1;   ///< bit:7 --- common to all the modality
          unsigned short int halt                      : 1;   ///< bit:8
          unsigned short int reserved                  : 4;   ///< bit:9,10,11,12 --- common to all the modality
          unsigned short int new_point_is_buffered     : 1;   ///< bit:13
          unsigned short int manufacturer_specific     : 3;   ///< bit:11,12,13,14,15 --- common to all the modality
        } bits;
      } __attribute__(( packed )) profiled_pos_control_word_t;
    
    /**
     * @struct cyclic_vel_status_word_t
     * @brief Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in CYCLIC SYNC VEL MODE
     */
    typedef union {
      uint16_t value;
      struct {
        unsigned short int ready_to_switch_on        : 1;        ///< bit:0
        unsigned short int switched_on               : 1;        ///< bit:1
        unsigned short int operation_enabled         : 1;        ///< bit:2
        unsigned short int fault                     : 1;        ///< bit:3
        unsigned short int voltage_enabled           : 1;        ///< bit:4
        unsigned short int quick_stop                : 1;        ///< bit:5
        unsigned short int switch_on_disabled        : 1;        ///< bit:6
        unsigned short int warning                   : 1;        ///< bit:7
        unsigned short int manufacturer_specific     : 1;        ///< bit:8
        unsigned short int remote                    : 1;        ///< bit:9
        unsigned short int target_reached            : 1;        ///< bit:10
        unsigned short int internal_limit_active     : 1;        ///< bit:11
        unsigned short int drive_follows_the_command : 1;        ///< bit:12 // 0: The drive does not follow the target value; 1: The drive follow the target value
        unsigned short int following_error           : 1;        ///< bit:13 // 0: No following error; 1: Following error
        unsigned short int manufacturer_specific_2   : 2;        ///< bit:14,15
      } bits;
    } __attribute__ ( ( packed ) ) cyclic_vel_status_word_t;
    
    /**
     * @struct cyclic_pos_status_word_t
     * @brief Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in CYCLIC SYNC VEL MODE
     */
    typedef cyclic_vel_status_word_t  cyclic_pos_status_word_t;



    
    
    /**
     * @struct cyclic_vel_control_word_t
     * Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in CYCLIC SYNC POS MODE
     */    
    typedef control_word_t            cyclic_vel_control_word_t;
    
    /**
     * @struct cyclic_pos_control_word_t
     * Bit-to-bit sescription of STATUS_WORD when device is imposed to be controlled in CYCLIC SYNC POS MODE
     */    
    typedef cyclic_vel_control_word_t cyclic_pos_control_word_t;


    typedef union {
      uint16_t value;
      struct {
        unsigned short int ready_to_switch_on       : 1;        ///< bit:0
        unsigned short int switched_on              : 1;        ///< bit:1
        unsigned short int operation_enabled        : 1;        ///< bit:2
        unsigned short int fault                    : 1;        ///< bit:3
        unsigned short int voltage_enabled          : 1;        ///< bit:4
        unsigned short int quick_stop               : 1;        ///< bit:5
        unsigned short int switch_on_disabled       : 1;        ///< bit:6
        unsigned short int warning                  : 1;        ///< bit:7
        unsigned short int manufacturer_specific    : 1;        ///< bit:8
        unsigned short int remote                   : 1;        ///< bit:9
        unsigned short int target_reached           : 1;        ///< bit:10
        unsigned short int internal_limit_active    : 1;        ///< bit:11
        unsigned short int operation_mode_specific  : 2;        ///< bit:12,13
        unsigned short int manufacturer_specific_2  : 2;        ///< bit:14,15
      } bits;
    } __attribute__((packed)) cyclic_torque_status_word_t;

      

    //--
    digital_inputs_t  to_digital_input ( const uint32_t& );
    uint32_t&         operator<<( uint32_t& lhs, const digital_inputs_t& rhs);                     
    digital_inputs_t& operator<<( digital_inputs_t& lhs, const uint32_t& rhs);       
    std::string       to_string(const digital_inputs_t& in );
  
    //---------
    status_word_t     to_status_word ( const uint16_t& );
    uint16_t&         operator<<( uint16_t& lhs, const status_word_t& rhs);
    status_word_t&    operator<<( status_word_t& lhs, const uint16_t& rhs);
    std::string       to_string(const status_word_t&  in, char how );
    
    control_word_t    to_control_word( const uint16_t& );
    control_word_t&   operator<<( control_word_t& lhs, const uint16_t& rhs);
    uint16_t&         operator<<( uint16_t& lhs, const control_word_t& rhs);     
    std::string       to_string(const control_word_t&  in, char how );
    
  
    //---------
    homing_status_word_t          to_homing_status_word ( const uint16_t& );
    homing_status_word_t&         operator<<( homing_status_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const homing_status_word_t& rhs);           
    status_word_t&                operator<<( status_word_t& lhs, const homing_status_word_t& rhs);
    homing_status_word_t&         operator<<( homing_status_word_t& lhs, const status_word_t& rhs);
    std::string                   to_string(const homing_status_word_t&              in, char how );
    
    homing_control_word_t         to_homing_control_word( const uint16_t& );
    homing_control_word_t&        operator<<( homing_control_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const homing_control_word_t& rhs);
    control_word_t&               operator<<( control_word_t& lhs, const homing_control_word_t& rhs);
    homing_control_word_t&        operator<<( homing_control_word_t& lhs, const control_word_t& rhs);
    std::string                   to_string(const homing_control_word_t&             in, char how );

    //---------
    profiled_vel_status_word_t    to_profiled_vel_status_word ( const uint16_t& );
    profiled_vel_status_word_t&   operator<<( profiled_vel_status_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const profiled_vel_status_word_t& rhs);
    status_word_t&                operator<<( status_word_t& lhs, const profiled_vel_status_word_t& rhs);
    profiled_vel_status_word_t&   operator<<( profiled_vel_status_word_t& lhs, const status_word_t& rhs);
    std::string                   to_string(const profiled_vel_status_word_t&        in, char how );
  
    profiled_vel_control_word_t   to_profiled_velcontrol_word( const uint16_t& );
    profiled_vel_control_word_t&  operator<<( profiled_vel_control_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const profiled_vel_control_word_t& rhs);
    control_word_t&               operator<<( control_word_t& lhs, const profiled_vel_control_word_t& rhs);
    profiled_vel_control_word_t&  operator<<( profiled_vel_control_word_t& lhs, const control_word_t& rhs);
    std::string                   to_string(const profiled_vel_control_word_t&       in, char how );
    
    //---------
    profiled_pos_status_word_t    to_profiled_pos_status_word ( const uint16_t& );
    profiled_pos_status_word_t&   operator<<( profiled_pos_status_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const profiled_pos_status_word_t& rhs);
    status_word_t&                operator<<( status_word_t& lhs, const profiled_pos_status_word_t& rhs);
    profiled_pos_status_word_t&   operator<<( profiled_pos_status_word_t& lhs, const status_word_t& rhs);
    std::string                   to_string(const profiled_pos_status_word_t&        in, char how );

    profiled_pos_control_word_t   to_profiled_poscontrol_word( const uint16_t& );
    profiled_pos_control_word_t&  operator<<( profiled_pos_control_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const profiled_pos_control_word_t& rhs);
    control_word_t&               operator<<( control_word_t& lhs, const profiled_pos_control_word_t& rhs);
    profiled_pos_control_word_t&  operator<<( profiled_pos_control_word_t& lhs, const control_word_t& rhs);
    std::string                   to_string(const profiled_pos_control_word_t&       in, char how );

    //---------
    cyclic_vel_status_word_t      to_cyclic_vel_status_word ( const uint16_t& );
    cyclic_vel_status_word_t&     operator<<( cyclic_vel_status_word_t& lhs, const uint16_t& rhs);
    uint16_t&                     operator<<( uint16_t& lhs, const cyclic_vel_status_word_t& rhs);
    status_word_t&                operator<<( status_word_t& lhs, const cyclic_vel_status_word_t& rhs);
    cyclic_vel_status_word_t&     operator<<( cyclic_vel_status_word_t& lhs, const status_word_t& rhs);
    std::string                   to_string(const cyclic_vel_status_word_t&        in, char how );
  
    cyclic_vel_control_word_t     to_cyclic_vel_control_word( const uint16_t& );                             
//     cyclic_vel_control_word_t&    operator<<( cyclic_vel_control_word_t& lhs, const uint16_t& rhs);       // cyclic_vel_control_word_t is alias of control_word_t
//     uint16_t&                     operator<<( uint16_t& lhs, const cyclic_vel_control_word_t& rhs);       // cyclic_vel_control_word_t is alias of control_word_t
//     control_word_t&               operator<<( control_word_t& lhs, const cyclic_vel_control_word_t& rhs); // cyclic_vel_control_word_t is alias of control_word_t
//     cyclic_vel_control_word_t&    operator<<( cyclic_vel_control_word_t& lhs, const control_word_t& rhs); // cyclic_vel_control_word_t is alias of control_word_t
//     std::string                   to_string(const cyclic_vel_control_word_t&       in, char how );        // cyclic_vel_control_word_t is alias of control_word_t

    //---------
    cyclic_pos_status_word_t      to_cyclic_pos_status_word ( const uint16_t& );
//     cyclic_pos_status_word_t&     operator<<( cyclic_pos_status_word_t& lhs, const uint16_t& rhs);         // cyclic_pos_control_word_t is alias of cyclic_vel_control_word_t 
//     uint16_t&                     operator<<( uint16_t& lhs, const cyclic_pos_status_word_t& rhs);         // cyclic_pos_control_word_t is alias of cyclic_vel_control_word_t 
//     status_word_t&                operator<<( status_word_t& lhs, const cyclic_pos_status_word_t& rhs);    // cyclic_pos_control_word_t is alias of cyclic_vel_control_word_t 
//     cyclic_pos_status_word_t&     operator<<( cyclic_pos_status_word_t& lhs, const status_word_t& rhs);    // cyclic_pos_control_word_t is alias of cyclic_vel_control_word_t 
//     std::string                   to_string(const cyclic_pos_status_word_t&        in, char how );         // cyclic_pos_control_word_t is alias of cyclic_vel_control_word_t 
  
    cyclic_pos_control_word_t     to_cyclic_pos_control_word( const uint16_t& );
//     cyclic_pos_control_word_t&    operator<<( cyclic_pos_control_word_t& lhs, const uint16_t& rhs);        // cyclic_pos_control_word_t is alias of control_word_t
//     uint16_t&                     operator<<( uint16_t& lhs, const cyclic_pos_control_word_t& rhs);        // cyclic_pos_control_word_t is alias of control_word_t
//     control_word_t&               operator<<( control_word_t& lhs, const cyclic_pos_control_word_t& rhs);  // cyclic_pos_control_word_t is alias of control_word_t
//     cyclic_pos_control_word_t&    operator<<( cyclic_pos_control_word_t& lhs, const control_word_t& rhs);  // cyclic_pos_control_word_t is alias of control_word_t
//     std::string                   to_string(const cyclic_pos_control_word_t&       in, char how );         // cyclic_pos_control_word_t is alias of control_word_t
    
}
}

#endif
