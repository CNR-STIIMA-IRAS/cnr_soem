#ifndef COE_CORE__DS402__COE_XFSM_SYMBOLS_H
#define COE_CORE__DS402__COE_XFSM_SYMBOLS_H

#include <string>
#include <vector>
//#include <coe_core/coe_utilities.h>
#include <coe_core/coe_sdo.h>
#include <coe_core/ds301/coe_sdo_dictionary.h>
#include <coe_core/ds402/coe_sdo_dictionary.h>
#include <coe_core/ds402/coe_bitwise_struct.h>

namespace coe_core
{
namespace ds402
{


/** 
 * @enum StateID All the different transition for the CANopen FSM (Elmo Motion Control CANopen DSP 402. Implementation Guide, December 2004, pag 21)
 */
enum StateID    { STATE_NOT_READY_TO_SWITCH_ON  = 0
                , STATE_SWITCH_ON_DISABLED      = 1
                , STATE_READY_TO_SWITCH_ON      = 2
                , STATE_SWITCHED_ON             = 3
                , STATE_OPERATION_ENABLED       = 4
                , STATE_QUICK_STOP_ACTIVE       = 5
                , STATE_FAULT_REACTION_ACTIVE   = 6
                , STATE_FAULT                   = 7
                , STATE_NO_STATE                = -1
};
                
enum StateType  { STATE_TYPE_POWER_DISABLED = 0
                , STATE_TYPE_POWER_ENABLED  = 1
                , STATE_TYPE_FAULT          = 2
                , STATE_FOO                 = 3
};

/// @enum TransitionsID All the different transition for the CANopen FSM (Elmo Motion Control CANopen DSP 402. Implementation Guide, December 2004, pp. 22-23)
enum TransitionID   { TRANSITION_LOOPBACK   = 0
                    , TRANSITION_0_INTERNAL 
                    , TRANSITION_1_INTERNAL
                    , TRANSITION_2
                    , TRANSITION_3
                    , TRANSITION_4
                    , TRANSITION_5
                    , TRANSITION_6
                    , TRANSITION_7
                    , TRANSITION_8
                    , TRANSITION_9
                    , TRANSITION_10
                    , TRANSITION_11
                    , TRANSITION_12
                    , TRANSITION_13_INTERNAL
                    , TRANSITION_14_INTERNAL
                    , TRANSITION_15
                    , TRANSITION_16
                    };
enum TransitionType { TRANSITION_INTERNAL = 0
                    , TRANSITION_FWD = 1
                    , TRANSITION_BCK = 2
                    , TRANSITION_ALARM = 3 
                    , TRANSITION_FOO = 4
                    };
                            
/// @enum CommandID All the different command to impose a transition to the CANopen FSM (Elmo Motion Control CANopen DSP 402. Implementation Guide, December 2004, pp. 22-23)
enum CommandID  { CMD_SHUTDOWN             = 0
                , CMD_SWITCH_ON
                , CMD_DISABLE_VOLTAGE
                , CMD_QUICK_STOP
                , CMD_DISABLE
                , CMD_ENABLE
                , CMD_FAULT_RESET
                , CMD_COMPLETE_RESET
                , CMD_INTERNAL
                , CMD_LOOPBACK
                };
                

//! @enum ModeOperationID according to the possible device control modality
enum ModeOperationID  : int8_t  { MOO_ERROR                        = -2
                                , MOO_NO_MODE                      = -1
                                , MOO_FIRST_RESERVED               =  0
                                , MOO_PROFILED_POSITION            =  1
                                , MOO_OPEN_LOOP_VELOCITY           =  2
                                , MOO_PROFILED_VELOCITY            =  3
                                , MOO_PROFILED_TORQUE              =  4
                                , MOO_SECOND_RESERVED              =  5
                                , MOO_HOMING                       =  6
                                , MOO_INTERPOLATED_POSITION        =  7
                                , MOO_CYCLIC_SYNCHRONOUS_POSITION  =  8
                                , MOO_CYCLIC_SYNCHRONOUS_VELOCITY  =  9
                                , MOO_CYCLIC_SYNCHRONOUS_TORQUE    = 10 };
                                

//! @enum ModeOperationID according to the possible device control modality
enum SupportedModeOperationID : int8_t  { SMOO_PROFILED_POSITION                                = 0
                                        , SMOO_OPEN_LOOP_VELOCITY                               = 1
                                        , SMOO_PROFILED_VELOCITY                                = 2
                                        , SMOO_PROFILED_TORQUE                                  = 3
                                        , SMOO_RESERVED                                         = 4
                                        , SMOO_HOMING                                           = 5
                                        , SMOO_INTERPOLATED_POSITION                            = 6
                                        , SMOO_CYCLIC_SYNCHRONOUS_POSITION                      = 7
                                        , SMOO_CYCLIC_SYNCHRONOUS_VELOCITY                      = 8
                                        , SMOO_CYCLIC_SYNCHRONOUS_TORQUE                        = 9
                                        , SMOO_CYCLIC_SYNCHRONOUS_TORQUE_WITH_COMMUTATION_ANGLE = 10
};


//! @enum QuickStopOptionCodeID according to the possible emergqncy stop modalities
enum QuickStopOptionCodeID : int16_t { QS_NOT_SUPPORTED                 = -1
                                        , QS_DISABLE_DRIVER                = 0 
                                        , QS_SLOW_DOWN_SLOW_RAMP_DISABLE   = 1
                                        , QS_SLOW_DOWN_QUICK_STOP_DISABLE  = 2
                                        , QS_SLOW_DOWN_CRNT_LIMIT_DISABLE  = 3
                                        , QS_NOT_SUPPORTED_4               = 4
                                        , QS_SLOW_DOWN_SLOW_RAMP_HELD      = 5
                                        , QS_NOT_SUPPORTED_6               = 6
                                        , QS_SLOW_DOWN_CRNT_LIMIT_HELD     = 7 };


                        
                        
// state & string
typedef std::map< StateID, std::string > StateIDStrings;
static const StateIDStrings STATEID_STRINGS = 
    { {STATE_NOT_READY_TO_SWITCH_ON  , "STATE_NOT_READY_TO_SWITCH_ON"}
    , {STATE_SWITCH_ON_DISABLED      , "STATE_SWITCH_ON_DISABLED"}
    , {STATE_READY_TO_SWITCH_ON      , "STATE_READY_TO_SWITCH_ON"}
    , {STATE_SWITCHED_ON             , "STATE_SWITCHED_ON"}
    , {STATE_OPERATION_ENABLED       , "STATE_OPERATION_ENABLED"}
    , {STATE_QUICK_STOP_ACTIVE       , "STATE_QUICK_STOP_ACTIVE"}
    , {STATE_FAULT_REACTION_ACTIVE   , "STATE_FAULT_REACTION_ACTIVE"}
    , {STATE_FAULT                   , "STATE_FAULT"}
    , {STATE_NO_STATE                , "STATE_UNEXPECTED"} };
    
// transition & string
typedef std::map< TransitionID, std::string >  TransitionIDStrings;
static const TransitionIDStrings TRANSITIONID_STRINGS =
    { {TRANSITION_LOOPBACK      , "TRANSITION_LOOPBACK"  }
    , {TRANSITION_0_INTERNAL    , "TRANSITION_0_INTERNAL"}
    , {TRANSITION_1_INTERNAL    , "TRANSITION_1_INTERNAL"}
    , {TRANSITION_2             , "TRANSITION_2"}
    , {TRANSITION_3             , "TRANSITION_3"}
    , {TRANSITION_4             , "TRANSITION_4"}
    , {TRANSITION_5             , "TRANSITION_5"}
    , {TRANSITION_6             , "TRANSITION_6"}
    , {TRANSITION_7             , "TRANSITION_7"}
    , {TRANSITION_8             , "TRANSITION_8"}
    , {TRANSITION_9             , "TRANSITION_9"}
    , {TRANSITION_10            , "TRANSITION_10"}
    , {TRANSITION_11            , "TRANSITION_11"}
    , {TRANSITION_12            , "TRANSITION_12"}
    , {TRANSITION_13_INTERNAL   , "TRANSITION_13_INTERNAL"}
    , {TRANSITION_14_INTERNAL   , "TRANSITION_14_INTERNAL"}
    , {TRANSITION_15            , "TRANSITION_15"}
    , {TRANSITION_16            , "TRANSITION_16"} };
    
typedef std::map< TransitionID, std::map< std::string,std::string > > TransitionsIDWStrings;
static const TransitionsIDWStrings TRANSITIONID_WSTRINGS =
    { {TRANSITION_LOOPBACK    , { {"Event" , "None"}
                                , {"Action", "None" } } }
    , {TRANSITION_0_INTERNAL  , { {"Event" , "reset"}
                                , {"Action", "The drive self-tests and/or self-initializes" } } }
    , {TRANSITION_1_INTERNAL  , { {"Event" , "The drive has self-tested and/or initialized successfully."}
                                , {"Action", "Activate communication" } }}
    , {TRANSITION_2           , { {"Event" , "Shutdown command received from host."}
                                , {"Action", "None"}}}
    , {TRANSITION_3           , { {"Event" , "Switch On command received from host."}
                                , {"Action", "The power section is switched on if it is not already on"}}}
    , {TRANSITION_4           , { {"Event" , "Enable Operation command received from host."}
                                , {"Action", "The drive function is enabled."}}}
    , {TRANSITION_5           , { {"Event" , "Disable Operation command received from host"}
                                , {"Action", "The drive operation is disabled."}}}
    , {TRANSITION_6           , { {"Event" , "Shutdown command received from host"}
                                , {"Action", "The power section is switched off"}}}
    , {TRANSITION_7           , { {"Event" , "Quick Stop and Disable Voltage command received from host"}
                                , {"Action", "None"}}}
    , {TRANSITION_8           , { {"Event" , "Shutdown command received from host"}
                                , {"Action", "The power section is switched off immediately"}}}
    , {TRANSITION_9           , { {"Event" , "Disable Voltage command received from host."}
                                , {"Action", "The power section is switched off immediately."}}}
    , {TRANSITION_10          , { {"Event" , "Disable Voltage or Quick Stop command received from host."}
                                , {"Action", "The power section is switched off immediately."}}}
    , {TRANSITION_11          , { {"Event" , "Quick Stop command received from host."}
                                , {"Action", "The quick stop function is executed."}}}
    , {TRANSITION_12          , { {"Event" , "This transition is possible if the quick stop option code is higher than 5 (stay in QUICK STOP ACTIVE state)"}
                                , {"Action", "The profile generator finished the deceleration and the motor is disabled."}}}
    , {TRANSITION_13_INTERNAL , { {"Event" , "A fault has occurred in the drive."}
                                , {"Action", "Execute appropriate fault reaction."}}}
    , {TRANSITION_14_INTERNAL , { {"Event" , "The fault reaction is completed.."}
                                , {"Action", "The drive function is disabled. The power section may be switched off."}}}
    , {TRANSITION_15          , { {"Event" , "Fault Reset command received from host."}
                                , {"Action", "The fault condition is reset if no fault currently exists in the drive.\n         NOTE After leaving FAULT state, the Fault Reset bit of the controlword must be cleared by the host. The drive does not monitor this bit in other states. If this bit is not cleared from a previous \n               fault state, when the next fault occurs, the drive automatically enters SWITCH ON DISABLED state with no indications or warning."}}}
    , {TRANSITION_16          , { {"Event" , "Enable Operation command received from host. This transition is possible if the quick stop option code (object 0x605A) is 5, 6.\n"}
                                , {"Action", "The drive function is enabled."}}}
    };
    


typedef std::map< CommandID, std::string >   CommandIDStrings;
static const CommandIDStrings COMMANDID_STRINGS = 
    { {CMD_SHUTDOWN       , "CMD_SHUTDOWN"      }
    , {CMD_SWITCH_ON      , "CMD_SWITCH_ON"     }
    , {CMD_DISABLE_VOLTAGE, "CMD_DISABLE_VOLTAGE"}
    , {CMD_QUICK_STOP     , "CMD_QUICK_STOP"    }
    , {CMD_DISABLE        , "CMD_DISABLE"       }
    , {CMD_ENABLE         , "CMD_ENABLE"        }
    , {CMD_FAULT_RESET    , "CMD_FAULT_RESET"   }
    , {CMD_COMPLETE_RESET , "CMD_COMPLETE_RESET"}
    , {CMD_INTERNAL       , "CMD_INTERNAL"      }
    , {CMD_LOOPBACK       , "CMD_LOOPBACK"      } 
    };


typedef std::map < ModeOperationID, std::string > ModeOperationIDStrings;
static const ModeOperationIDStrings MODEOPERATIONSID_STRINGS = {
  {MOO_ERROR                       , "MOO_ERROR"}
, {MOO_NO_MODE                     , "MOO_NO_MODE"}
, {MOO_FIRST_RESERVED              , "MOO_FIRST_RESERVED"}
, {MOO_PROFILED_POSITION           , "MOO_PROFILED_POSITION"}
, {MOO_OPEN_LOOP_VELOCITY          , "MOO_VELOCITY_MODE_OPEN_LOOP"}
, {MOO_PROFILED_VELOCITY           , "MOO_PROFILED_VELOCITY"}
, {MOO_PROFILED_TORQUE             , "MOO_PROFILED_TORQUE"}
, {MOO_SECOND_RESERVED             , "MOO_SECOND_RESERVED" }
, {MOO_HOMING                      , "MOO_HOMING"}
, {MOO_INTERPOLATED_POSITION       , "MOO_INTERPOLATED_POSITION"}
, {MOO_CYCLIC_SYNCHRONOUS_POSITION , "MOO_CYCLIC_SYNCHRONOUS_POSITION"}
, {MOO_CYCLIC_SYNCHRONOUS_VELOCITY , "MOO_CYCLIC_SYNCHRONOUS_VELOCITY"}
, {MOO_CYCLIC_SYNCHRONOUS_TORQUE   , "MOO_CYCLIC_SYNCHRONOUS_TORQUE"}
};


typedef std::map < QuickStopOptionCodeID, std::string > QuickStopIDStrings;
static const QuickStopIDStrings QUICKSTOPID_STRINGS = {
  {QS_NOT_SUPPORTED                 , "QS_NOT_SUPPORTED"}
, {QS_DISABLE_DRIVER                , "QS_DISABLE_DRIVER"}
, {QS_SLOW_DOWN_SLOW_RAMP_DISABLE   , "QS_SLOW_DOWN_SLOW_RAMP_DISABLE"}
, {QS_SLOW_DOWN_QUICK_STOP_DISABLE  , "QS_SLOW_DOWN_QUICK_STOP_DISABLE"}
, {QS_SLOW_DOWN_CRNT_LIMIT_DISABLE  , "QS_SLOW_DOWN_CRNT_LIMIT_DISABLE"}
, {QS_NOT_SUPPORTED_4               , "QS_NOT_SUPPORTED_4"}
, {QS_SLOW_DOWN_SLOW_RAMP_HELD      , "QS_SLOW_DOWN_SLOW_RAMP_HELD"}
, {QS_NOT_SUPPORTED_6               , "QS_NOT_SUPPORTED_6" }
, {QS_SLOW_DOWN_CRNT_LIMIT_HELD     , "QS_SLOW_DOWN_CRNT_LIMIT_HELD"} };

}  // namespace ds402
}  // namespace coe_core

#endif  // COE_CORE__DS402__COE_XFSM_SYMBOLS_H
