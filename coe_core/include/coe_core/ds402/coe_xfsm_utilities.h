#ifndef COE_CORE__DS402__COE_XFSM_UTILITIES_H
#define COE_CORE__DS402__COE_XFSM_UTILITIES_H

#include <string>
#include <coe_core/ds402/coe_xfsm_symbols.h>
#include <coe_core/ds402/coe_xfsm_sequence.h>


namespace coe_core
{ 
namespace ds402
{

std::string     to_string       ( const StateID& ); 
std::string     to_string       ( const TransitionID&, const char* terminator = "\n" ); 
std::string     to_wstring      ( const TransitionID& );
std::string     to_string       ( const CommandID& );
std::string     to_string       ( const ModeOperationID& );
std::string     to_string       ( const QuickStopOptionCodeID& );

StateID         to_stateid      ( const uint16_t&       status_word );
StateID         to_stateid      ( const status_word_t&  status_word );
StateID         to_stateid      ( const std::string&    str );
std::string     to_stateid_name ( const uint16_t&       status_word );
std::string     what_is_happen  ( const StateID& prev, const StateID& act, int verbose = 0);

CommandID       to_commandid    ( const uint16_t&       control_word );
CommandID       to_commandid    ( const control_word_t& control_word );
CommandID       to_commandid    ( const std::string&    str );
CommandID       to_commandid    ( const TransitionID&   transition );

ModeOperationID to_modeofoperationid ( const int8_t& mode_of_operation );
ModeOperationID to_modeofoperationid ( const std::string& str );

QuickStopOptionCodeID to_quickstopid (const std::string& str );

void to_control_word            ( const CommandID& cmd, uint16_t* );
int  calc_controlword           ( const StateID& act, const CommandID& cmd, StateID* next, uint16_t* controlword );
int  calc_controlword           ( const StateID& act, const StateID& next, uint16_t* controlword );

std::vector< std::pair< StateID, StateID > > get_states_pair(const TransitionID& transition ) ;

FeasibleStateCommands     get_feasible_commands( const StateID& state );
FeasibleStateTransitions  get_feasible_transitions( const StateID& state, const TransitionType& typ  );
StateID                   get_next_state( const StateID& state, const CommandID& command );
std::vector< CommandID >  get_next_fwd_command( const StateID& state );
std::vector< CommandID >  get_next_bck_command( const StateID& state );
bool                      is_fault( const StateID& state ) ;
std::string               echo_feasible_commands( const StateID& state);
std::string               echo_feasible_modeoperation( );
std::string               echo_transition_type( const TransitionID& transition );                             

}  // namespace ds402
}  // namespace coe_core

#endif  // COE_CORE__DS402__COE_XFSM_UTILITIES_H
