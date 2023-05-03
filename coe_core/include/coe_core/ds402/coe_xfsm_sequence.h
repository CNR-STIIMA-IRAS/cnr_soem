#ifndef COE_CORE__DS402__XFSM_SEQUENCE__H
#define COE_CORE__DS402__XFSM_SEQUENCE__H

#include <cstdint>
#include <map>
#include <coe_core/ds402/coe_xfsm_symbols.h>


namespace coe_core
{
namespace ds402
{

/**
 *
 *
 * FROM Elmo Motion Control, CANopen DSP 402, Implementation Guide,December 2004
 *
 *
 *
 *
 */  
// state and command masks
typedef std::map< StateID,     std::map<std::string,uint16_t> >  StateMask; 
static const  StateMask STATE_MASK =   { // ATTENTION Thes state STATE_NO_STATE has not any bitmask corresponding representation
    {STATE_NOT_READY_TO_SWITCH_ON  , { {"MASK", 0b0000000001001111 },{"VAL", 0b0000000000000000}  } }  // xxxx xxxx x0xx 0000 Not ready to switch on
  , {STATE_SWITCH_ON_DISABLED      , { {"MASK", 0b0000000001001111 },{"VAL", 0b0000000001000000}  } }  // xxxx xxxx x1xx 0000 Switch on disabled
  , {STATE_READY_TO_SWITCH_ON      , { {"MASK", 0b0000000001101111 },{"VAL", 0b0000000000100001}  } }  // xxxx xxxx x01x 0001 Ready to switch on
  , {STATE_SWITCHED_ON             , { {"MASK", 0b0000000001101111 },{"VAL", 0b0000000000100011}  } }  // xxxx xxxx x01x 0011 Switch on
  , {STATE_OPERATION_ENABLED       , { {"MASK", 0b0000000001101111 },{"VAL", 0b0000000000100111}  } }  // xxxx xxxx x01x 0111 Operation enabled
  , {STATE_QUICK_STOP_ACTIVE       , { {"MASK", 0b0000000001101111 },{"VAL", 0b0000000000000111}  } }  // xxxx xxxx x00x 0111 Quick stop active
  , {STATE_FAULT_REACTION_ACTIVE   , { {"MASK", 0b0000000001001111 },{"VAL", 0b0000000000001111}  } }  // xxxx xxxx x0xx 1111 Fault reaction active
  , {STATE_FAULT                   , { {"MASK", 0b0000000001001111 },{"VAL", 0b0000000000001000}  } }  // xxxx xxxx x0xx 1000 Fault 
};

typedef std::map< CommandID, std::map<std::string,uint16_t> > CommandMask; 
static const  CommandMask COMMAND_MASK = { 
//bit                 7               3               2           1           0
//                FaultReset | EnableOperation | QuickStop | EnableVoltage | SwitchOn
//Shutdown            0               X               1           1           0   -- 2, 6, 8
//Switch ON           0               0               1           1           1   -- 3*
//Switch ON           0               1               1           1           1   -- 3**
//Disable Voltage     0               X               X           0           X   -- 7, 9, 10, 12
//Quick Stop          0               X               0           1           X   -- 7, 10, 11
//Disable Operation   0               0               1           1           1   -- 5
//Enable Operation    0               1               1           1           1   -- 4, 16
//Fault Reset         TR              X               X           X           X   -- 15
    {CMD_SHUTDOWN       , { {"MASK", 0b0000000010000111 },{"VAL", 0b0000000000000110}  } } //uint8_t cmd[2], cmd[0] = 0x87, cmd[1] = 0x00
  , {CMD_SWITCH_ON      , { {"MASK", 0b0000000010001111 },{"VAL", 0b0000000000000111}  } } //uint8_t cmd[2], cmd[0] = 0x8f, cmd[1] = 0x00
  , {CMD_DISABLE_VOLTAGE, { {"MASK", 0b0000000010000010 },{"VAL", 0b0000000000000000}  } } //uint8_t cmd[2], cmd[0] = 0x86, cmd[1] = 0x00
  , {CMD_QUICK_STOP     , { {"MASK", 0b0000000010000110 },{"VAL", 0b0000000000000010}  } } //uint8_t cmd[2], cmd[0] = 0x80, cmd[1] = 0x00
  , {CMD_DISABLE        , { {"MASK", 0b0000000010001111 },{"VAL", 0b0000000000000111}  } } //uint8_t cmd[2], cmd[0] = 0x8f, cmd[1] = 0x00
  , {CMD_ENABLE         , { {"MASK", 0b0000000010001111 },{"VAL", 0b0000000000001111}  } } //uint8_t cmd[2], cmd[0] = 0x8f, cmd[1] = 0x00
  , {CMD_FAULT_RESET    , { {"MASK", 0b0000000011111111 },{"VAL", 0b0000000010000000}  } } //uint8_t cmd[2], cmd[0] = 0xff, cmd[1] = 0x00
  , {CMD_COMPLETE_RESET , { {"MASK", 0b1111111111111111 },{"VAL", 0b0000000000000000}  } } // it does reset the control word!
  , {CMD_INTERNAL       , { {"MASK", 0b0000000000000000 },{"VAL", 0b0000000000000000}  } } // it does not change the control word!
  , {CMD_LOOPBACK       , { {"MASK", 0b0000000000000000 },{"VAL", 0b0000000000000000}  } } };


typedef std::map< CommandID, std::vector< TransitionID > >  CommandTansitionFeasibilityMap; // one-to-one
static const  CommandTansitionFeasibilityMap COMMAND_TRANSITION_MAP =  {
//bit                 7               3               2           1           0
//                FaultReset | EnableOperation | QuickStop | EnableVoltage | SwitchOn
//Shutdown            0               X               1           1           0   -- 2, 6, 8
//Switch ON           0               0               1           1           1   -- 3*
//Switch ON           0               1               1           1           1   -- 3**
//Disable Voltage     0               X               X           0           X   -- 7, 9, 10, 12
//Quick Stop          0               X               0           1           X   -- 7, 10, 11
//Disable Operation   0               0               1           1           1   -- 5
//Enable Operation    0               1               1           1           1   -- 4, 16
//Fault Reset         TR              X               X           X           X   -- 15
      {CMD_SHUTDOWN       , {TRANSITION_2,TRANSITION_6,TRANSITION_8}}
    , {CMD_SWITCH_ON      , {TRANSITION_3 }}
    , {CMD_DISABLE_VOLTAGE, {TRANSITION_7,TRANSITION_9,TRANSITION_10,TRANSITION_12 }}
    , {CMD_QUICK_STOP     , {TRANSITION_7,TRANSITION_10,TRANSITION_11 }}
    , {CMD_DISABLE        , {TRANSITION_5 }}
    , {CMD_ENABLE         , {TRANSITION_4,TRANSITION_16 }}
    , {CMD_FAULT_RESET    , {TRANSITION_15 }}
    , {CMD_COMPLETE_RESET , {TRANSITION_0_INTERNAL }}
    , {CMD_INTERNAL       , {TRANSITION_1_INTERNAL, TRANSITION_13_INTERNAL }}
    , {CMD_LOOPBACK       , {TRANSITION_LOOPBACK}}
    };

  
/**
 *
 *
 * END FROM Elmo Motion Control, CANopen DSP 402, Implementation Guide,December 2004
 *
 *
 *
 *
 */
  
  


/**
 *
 *
 * Some properties
 *
 *
 *
 *
 */
typedef std::map<StateID, StateType> StateTypeMap;
static const  StateTypeMap STATE_TYPE_MAP =  { 
    {STATE_NOT_READY_TO_SWITCH_ON,  STATE_TYPE_POWER_DISABLED }
  , {STATE_SWITCH_ON_DISABLED     , STATE_TYPE_POWER_DISABLED }
  , {STATE_READY_TO_SWITCH_ON     , STATE_TYPE_POWER_DISABLED }
  , {STATE_SWITCHED_ON            , STATE_TYPE_POWER_ENABLED  }
  , {STATE_OPERATION_ENABLED      , STATE_TYPE_POWER_ENABLED  }
  , {STATE_QUICK_STOP_ACTIVE      , STATE_TYPE_POWER_ENABLED  }
  , {STATE_FAULT_REACTION_ACTIVE  , STATE_TYPE_FAULT          }
  , {STATE_FAULT                  , STATE_TYPE_FAULT          }
  , {STATE_NO_STATE               , STATE_TYPE_FAULT          }
};

typedef std::map<TransitionID, TransitionType> TransitionTypeMap;
static const  TransitionTypeMap TRANSITION_TYPE_MAP =  {
   { TRANSITION_LOOPBACK      , TRANSITION_FOO      }
  ,{ TRANSITION_0_INTERNAL    , TRANSITION_INTERNAL }
  ,{ TRANSITION_1_INTERNAL    , TRANSITION_INTERNAL }
  ,{ TRANSITION_2             , TRANSITION_FWD      }
  ,{ TRANSITION_3             , TRANSITION_FWD      }
  ,{ TRANSITION_4             , TRANSITION_FWD      }
  ,{ TRANSITION_5             , TRANSITION_BCK      }
  ,{ TRANSITION_6             , TRANSITION_BCK      }
  ,{ TRANSITION_7             , TRANSITION_BCK      }
  ,{ TRANSITION_8             , TRANSITION_BCK      }
  ,{ TRANSITION_9             , TRANSITION_BCK      }
  ,{ TRANSITION_10            , TRANSITION_ALARM    }
  ,{ TRANSITION_11            , TRANSITION_ALARM    }
  ,{ TRANSITION_12            , TRANSITION_ALARM    }
  ,{ TRANSITION_13_INTERNAL   , TRANSITION_INTERNAL }
  ,{ TRANSITION_14_INTERNAL   , TRANSITION_INTERNAL }
  ,{ TRANSITION_15            , TRANSITION_ALARM    }
  ,{ TRANSITION_16            , TRANSITION_FWD      } };
            


typedef std::vector< std::pair< CommandID, StateID > >    FeasibleStateCommands;     // one-to-many

typedef std::vector< std::pair< TransitionID, StateID > > FeasibleStateTransitions;     

typedef std::map< StateID, FeasibleStateTransitions >     StateTransitionFeasibilityMap;
static const StateTransitionFeasibilityMap STATE_TRANSITIONS_MAP = { 
    {STATE_NO_STATE                , { std::make_pair(TRANSITION_LOOPBACK   , STATE_NO_STATE              )   
                                   ,   std::make_pair(TRANSITION_0_INTERNAL , STATE_SWITCH_ON_DISABLED    ) } }
  , {STATE_NOT_READY_TO_SWITCH_ON  , { std::make_pair(TRANSITION_LOOPBACK   , STATE_NOT_READY_TO_SWITCH_ON)
                                   ,   std::make_pair(TRANSITION_1_INTERNAL , STATE_SWITCH_ON_DISABLED    ) } }
  , {STATE_SWITCH_ON_DISABLED      , { std::make_pair(TRANSITION_LOOPBACK   , STATE_SWITCH_ON_DISABLED    )
                                   ,   std::make_pair(TRANSITION_2          , STATE_READY_TO_SWITCH_ON    ) } }
  , {STATE_READY_TO_SWITCH_ON      , { std::make_pair(TRANSITION_LOOPBACK   , STATE_READY_TO_SWITCH_ON    )
                                   ,   std::make_pair(TRANSITION_3          , STATE_SWITCHED_ON           ) } }
  , {STATE_SWITCHED_ON             , { std::make_pair(TRANSITION_LOOPBACK   , STATE_SWITCHED_ON           )
                                   ,   std::make_pair(TRANSITION_4          , STATE_OPERATION_ENABLED     ) 
                                   ,   std::make_pair(TRANSITION_6          , STATE_READY_TO_SWITCH_ON    ) 
                                   ,   std::make_pair(TRANSITION_10         , STATE_SWITCH_ON_DISABLED    ) } }
  , {STATE_OPERATION_ENABLED       , { std::make_pair(TRANSITION_LOOPBACK   , STATE_OPERATION_ENABLED     )
                                   ,   std::make_pair(TRANSITION_5          , STATE_SWITCHED_ON           ) 
                                   ,   std::make_pair(TRANSITION_8          , STATE_READY_TO_SWITCH_ON    ) 
                                   ,   std::make_pair(TRANSITION_9          , STATE_SWITCH_ON_DISABLED    ) 
                                   ,   std::make_pair(TRANSITION_11         , STATE_QUICK_STOP_ACTIVE     ) } }
  , {STATE_QUICK_STOP_ACTIVE       , { std::make_pair(TRANSITION_LOOPBACK   , STATE_QUICK_STOP_ACTIVE     )
                                   ,   std::make_pair(TRANSITION_12         , STATE_SWITCH_ON_DISABLED    )
                                   ,   std::make_pair(TRANSITION_16         , STATE_OPERATION_ENABLED     ) } }
  , {STATE_FAULT_REACTION_ACTIVE   , { std::make_pair(TRANSITION_LOOPBACK   , STATE_FAULT_REACTION_ACTIVE )
                                   ,   std::make_pair(TRANSITION_14_INTERNAL, STATE_NOT_READY_TO_SWITCH_ON) } }
  , {STATE_FAULT                   , { std::make_pair(TRANSITION_LOOPBACK   , STATE_FAULT                 )
                                   ,   std::make_pair(TRANSITION_15         , STATE_SWITCH_ON_DISABLED    ) } }
  };
/**
 *
 *
 * End - Some P�roperties
 *
 *
 *
 *
 */ 

}  // namespace ds402
}  // namespace coe_core
#endif  // COE_CORE__DS402__XFSM_SEQUENCE__H
