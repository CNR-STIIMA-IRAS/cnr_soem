
#include <climits>
#include <coe_core/coe_utilities.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_core/ds301/coe_bitwise_struct.h>
#include <coe_core/ds402/coe_xfsm_symbols.h>
#include <coe_core/ds402/coe_xfsm_sequence.h>
#include <coe_core/ds402/coe_xfsm_utilities.h>

namespace coe_core 
{
namespace ds402
{


std::string to_string ( const StateID& idx)
{
    return STATEID_STRINGS.at(idx);
};
std::string to_string (const ModeOperationID& in )
{
  return MODEOPERATIONSID_STRINGS.at(in);
}
StateID to_stateid ( const uint16_t& status_word )
{
    for ( StateMask::const_iterator it = STATE_MASK.begin(); it != STATE_MASK.end(); it++ )
    {
        if ( ( status_word & it->second.at("MASK") ) == it->second.at("VAL") )
            return it->first;
    }

    return STATE_NO_STATE;
}
StateID to_stateid ( const status_word_t& status_word ) 
{
  uint16_t sw = *(uint16_t*)&status_word;
  return to_stateid( sw );
}
StateID to_stateid(const std::string& str )
{
  StateIDStrings::const_iterator it;
  for( it=STATEID_STRINGS.begin(); it!=STATEID_STRINGS.end(); it++)
  {
    if( it->second == str )
      return it->first;
  }
  throw std::runtime_error(std::string("Input val '"+str+"' is not valid").c_str());
}
std::string to_stateid_name ( const uint16_t& statusword )
{
    return to_string ( to_stateid ( statusword ) );
}
CommandID to_commandid ( const uint16_t& control_word )
{
    for ( CommandMask::const_iterator it = COMMAND_MASK.begin(); it != COMMAND_MASK.end(); it++ )
    {
        if ( ( control_word & it->second.at("MASK") ) == it->second.at("VAL") )
            return it->first;
    }

    return CMD_COMPLETE_RESET;
}
CommandID to_commandid ( const control_word_t& control_word )
{
  uint16_t cw = *(uint16_t*)&control_word;
  return to_commandid( cw );
}
CommandID to_commandid(const std::string& str )
{
  CommandIDStrings::const_iterator it;
  for( it=COMMANDID_STRINGS.begin(); it!=COMMANDID_STRINGS.end(); it++)
  {
    if( it->second == str )
      return it->first;
  }
  throw std::runtime_error(std::string("Command '"+str+"' is not valid").c_str());
}
CommandID to_commandid( const TransitionID& transition )
{
  CommandTansitionFeasibilityMap::const_iterator it;
  for( it = COMMAND_TRANSITION_MAP.begin(); it != COMMAND_TRANSITION_MAP.end(); it++ )
  {
    std::vector< TransitionID >::const_iterator jt = std::find (it->second.begin(), it->second.end(), transition);
    if (jt != it->second.end() )
      return it->first;
  }
  throw std::runtime_error("Command is not valid..wierd error");
}
ModeOperationID to_modeofoperationid( const int8_t& MOF )
{
  for ( ModeOperationIDStrings::const_iterator it = MODEOPERATIONSID_STRINGS.begin()
      ; it != MODEOPERATIONSID_STRINGS.end()
      ; it++ )
  {
    if( int8_t( it->first ) == MOF)
       return it->first;
  }
  throw std::runtime_error(std::string("Invalid input val ").c_str() );
}
ModeOperationID to_modeofoperationid(const std::string& str )
{
  ModeOperationIDStrings::const_iterator it;
  for( it=MODEOPERATIONSID_STRINGS.begin(); it!=MODEOPERATIONSID_STRINGS.end(); it++)
  {
    if( it->second == str )
      return it->first;
  }
  throw std::runtime_error(std::string("Mode of operation '"+str+"' is not valid").c_str());
}
QuickStopOptionCodeID to_quickstopid(const std::string& str )
{
  QuickStopIDStrings::const_iterator it;
  for( it=QUICKSTOPID_STRINGS.begin(); it!=QUICKSTOPID_STRINGS.end(); it++)
  {
    if( it->second == str )
      return it->first;
  }
  throw std::runtime_error(std::string("Quick Stop code '"+str+"' is not valid").c_str());
}
std::string to_string ( const TransitionID& transition, const char* terminator)
{
    std::string ret;
    std::vector< std::pair< StateID, StateID > > v = get_states_pair(transition);
    for( std::size_t i=0;i<v.size();i++)
        ret += TRANSITIONID_STRINGS.at(transition) + std::string(": ") 
            + STATEID_STRINGS.at( v[i].first ) + " => " + STATEID_STRINGS.at( v[i].second ) 
            + std::string(terminator);
    return ret;
}

std::string to_wstring ( const TransitionID& transition )
{
    std::string ret = to_string ( transition, "\n");
    ret += "\t Event:  " + TRANSITIONID_WSTRINGS.at(transition).at("Event") +"\n";
    ret += "\t Action: " + TRANSITIONID_WSTRINGS.at(transition).at("Action") +"\n";
    
    return ret;
}


// ritorna la transizione appena eseguita
std::string what_is_happen ( const StateID& prev_state, const StateID& new_state, int verbose )
{
    if ( prev_state==new_state )
        return "No modification in the state";

    std::string ret;
    std::vector< std::pair< TransitionID, StateID > > possible_states = STATE_TRANSITIONS_MAP.at( prev_state );
    std::vector< std::pair< TransitionID, StateID > >::const_iterator it;
    for( it = possible_states.begin(); it != possible_states.end(); it++ ) 
    {
        if( it->second==new_state )
            ret += ( verbose==0 ) ? to_string ( it->first ) : to_wstring ( it->first ) +"\n";
    }
    return ret;
}
// ritorna una stringa con le etichette dei comandi al drive, normale se eq = 0, allineato se eq =1
std::string  to_string ( const CommandID& idx )
{
    return COMMANDID_STRINGS.at(idx);
}
// dato un comando e lo stato attuale del drive, ritorna la controlword da inviare al drive e lo stato futuro
int calc_controlword( const StateID& act_state, const CommandID& cmd, StateID* next_state_addr, uint16_t* controlword_addr, std::string& what)
{

    int ret = 0;
    
    std::vector<TransitionID>                       possible_transitions = COMMAND_TRANSITION_MAP.at(cmd);
    std::vector<std::pair<TransitionID,StateID> >   possible_next_states = STATE_TRANSITIONS_MAP.at(act_state);
    std::vector<StateID>                            reacheable_states;
    
    for ( std::vector<std::pair<TransitionID,StateID> >::const_iterator itp = possible_next_states.begin(); itp != possible_next_states.end(); itp++)
    {
        for( std::vector<TransitionID>::const_iterator itt = possible_transitions.begin(); itt != possible_transitions.end(); itt ++ )
        {
            if( itp->first == *itt )
                reacheable_states.push_back( itp->second );
        }
    }
    
    bool ok = true;

    if(reacheable_states.size() == 0 || reacheable_states.size() > 1 )
    {
        if( reacheable_states.size() > 1 )
        {
            for(std::size_t i=0;i<reacheable_states.size();i++)
                for(std::size_t j=0;j<reacheable_states.size();j++)
                    ok &= reacheable_states.at(i) == reacheable_states.at(j);
        }
    } else if (reacheable_states.size() == 1 )
        ok = true;
    
    if(!ok)     
    {
        what = "ERROR ******** \n" ;
        what +="       Act state (IN ): " + to_string ( act_state ) +"\n";
        what +="ERROR ******** \n" ;
        what +="         Command (IN ): " + to_string ( cmd )+ "\n";
        what +="ERROR ******** \n" ;
        what +="\t  ctrlword bits  (IN ): ";
        what += to_string_bin( *controlword_addr );
        what += "STATES FOUND "+ std::to_string(reacheable_states.size()) + ": ";
        for(std::size_t i=0;i<reacheable_states.size(); i++)
        {
            what += "\t  FOUND STATE : " + STATEID_STRINGS.at(  reacheable_states.at(i) );
        }

        printf ( "\n" );
        ret = -1;
    } 
    else
    {
        uint16_t new_controlword;
        new_controlword = ( *controlword_addr );    
        new_controlword &= ( ~COMMAND_MASK.at(cmd).at("MASK") );  // the mask indicates what are the bit to set. 
                                                            // Thus, the complement (~) of the mask set to zero the element to be set
        new_controlword |= COMMAND_MASK.at(cmd).at("VAL");        // Since the value of the mask bit have been set to zero, the OR takes the velua in the command
    
        std::memcpy ( controlword_addr, &new_controlword, sizeof ( uint16_t ) );
        *next_state_addr = reacheable_states[0];
        ret = 0;
    }
    
    return ret;
} // end of fun

int calc_controlword ( const StateID&  act_state, const StateID& next_state, uint16_t* controlword_addr, std::string& what)
{
    if ( act_state==next_state )
        return 0;
    
    int ret = 0;
    std::vector<std::pair<TransitionID,StateID> >   possible_states_from_act  = STATE_TRANSITIONS_MAP.at(act_state);
    std::vector<std::pair<TransitionID,StateID> >   possible_states_from_next = STATE_TRANSITIONS_MAP.at(next_state);

    std::vector<TransitionID>                       possible_transitions;
    
    for ( std::vector<std::pair<TransitionID,StateID> >::const_iterator ita = possible_states_from_act.begin(); ita != possible_states_from_act.end(); ita++)
    {
        for ( std::vector<std::pair<TransitionID,StateID> >::const_iterator itn = possible_states_from_next.begin(); itn != possible_states_from_next.end(); itn++)
        {
            if( ita->second== itn->second )
                possible_transitions.push_back( ita->first );
        }
    }
    
    if(possible_transitions.size() == 0 || possible_transitions.size() > 1 )
    {
        what = "ERROR ******** \n";
        what += "       Act state (IN ): " +STATEID_STRINGS.at( act_state ) + "\n";
        what += "ERROR ******** \n";
        what += "       Nex state (IN ):" + STATEID_STRINGS.at( next_state ) + "\n ";
        what += "TRANSITIONS FOUND "+ std::to_string(possible_transitions.size())+": ";
        for(std::size_t i=0;i<possible_transitions.size(); i++)
        {
            what +="\t  FOUND TRANSITIONS : " + TRANSITIONID_STRINGS.at(  possible_transitions.at(i) );
        }
        ret = -1;
    } 
    else if(possible_transitions.size() == 1 )
    {
        uint16_t new_controlword;
        new_controlword = ( *controlword_addr );
        new_controlword &= ( ~COMMAND_MASK.at( to_commandid(possible_transitions[0] ) ).at("MASK") );
        new_controlword |= COMMAND_MASK.at( to_commandid( possible_transitions[0] ) ).at("VAL");
    
        std::memcpy ( controlword_addr, &new_controlword, sizeof ( uint16_t ) );
        ret = 0;
    }
    
    return ret;
}

void to_control_word ( const CommandID& cmd, uint16_t* control_word )
{
  uint16_t new_control_word = *control_word; 
  new_control_word &= ( ~COMMAND_MASK.at(cmd).at("MASK") ); // the mask indicates what are the bit to set. Thus, the complement (~) of the mask set to zero the element to be set
  new_control_word |= COMMAND_MASK.at(cmd).at("VAL");       
  
  *control_word = new_control_word;
}  
std::vector< std::pair< StateID, StateID > > get_states_pair(const TransitionID& transition ) 
{ 
  std::vector< std::pair< StateID, StateID > > ret;
  
  for( StateTransitionFeasibilityMap::const_iterator it = STATE_TRANSITIONS_MAP.begin(); it != STATE_TRANSITIONS_MAP.end(); it++)
  {
    for( FeasibleStateTransitions::const_iterator jt = it->second.begin(); jt != it->second.end(); jt++ )
    {
      if( jt->first == transition )
        ret.push_back( std::make_pair( it->first, jt->second ) );
    }
  }
  return ret;
}  
FeasibleStateCommands get_feasible_commands( const StateID& state )
{
  FeasibleStateCommands ret;
  FeasibleStateTransitions reacheable_states = STATE_TRANSITIONS_MAP.at( state );
  FeasibleStateTransitions::const_iterator it;
  for( it = reacheable_states.begin(); it != reacheable_states.end(); it++ )
    ret.push_back( std::make_pair( to_commandid( it->first ), it->second ) );
        
  return ret;
}
FeasibleStateTransitions get_feasible_transitions( const StateID& state, const TransitionType& typ  )
{
  FeasibleStateTransitions ret;
  FeasibleStateTransitions all_feasible_transitions = STATE_TRANSITIONS_MAP.at( state );
  
  for( FeasibleStateTransitions::const_iterator it = all_feasible_transitions.begin(); it != all_feasible_transitions.end(); it++ )
    if( TRANSITION_TYPE_MAP.at( it->first ) == typ )
      ret.push_back( std::make_pair( it->first, it->second ) );
        
  return ret;
}
StateID get_next_state( const StateID& state, const CommandID& command )
{
    FeasibleStateTransitions reacheable_states = STATE_TRANSITIONS_MAP.at( state );
    for( FeasibleStateTransitions::const_iterator  it = reacheable_states.begin(); it != reacheable_states.end(); it++ )
        if( to_commandid( it->first ) == command ) 
            return it->second;
        
    return state;
}

std::vector< CommandID > get_next_fwd_command( const StateID& state )
{
    std::vector< CommandID > ret;
    FeasibleStateTransitions reacheable_states = STATE_TRANSITIONS_MAP.at( state );
    
    for( FeasibleStateTransitions::const_iterator it = reacheable_states.begin(); it != reacheable_states.end(); it++ )
    {
        if( TRANSITION_TYPE_MAP.at( it->first ) == TRANSITION_FWD ) 
        {
            ret.push_back( to_commandid( it->first ) );
        }
    }
    return ret;
}

std::vector< CommandID > get_next_bck_command( const StateID& state )
{
    std::vector< CommandID > ret;
    FeasibleStateTransitions reacheable_states = STATE_TRANSITIONS_MAP.at( state );
    
    for( FeasibleStateTransitions::const_iterator it = reacheable_states.begin(); it != reacheable_states.end(); it++ )
    {
        if( TRANSITION_TYPE_MAP.at( it->first ) == TRANSITION_BCK ) 
        {
            ret.push_back( to_commandid( it->first ) );
        }
    }
    return ret;
}
bool is_fault( const StateID& state ) 
{
    return STATE_TYPE_MAP.at( state ) == STATE_TYPE_FAULT;
}

std::string echo_feasible_commands( const StateID& state) 
{
  std::string ret;
  ret += "Acutal State     : " + STATEID_STRINGS.at( state )+"\n";
  ret += "Feasible Commands:\n";
  FeasibleStateTransitions feasible_transitions = STATE_TRANSITIONS_MAP.at( state );
  FeasibleStateCommands    feasible_commands = get_feasible_commands( state );
    
  for( FeasibleStateCommands::const_iterator it = feasible_commands.begin(); it != feasible_commands.end(); it++ )
  {
    for( FeasibleStateTransitions::const_iterator jt = feasible_transitions.begin(); jt != feasible_transitions.end(); jt++ )
    {
      if(jt->second == it->second )
      {
        ret += COMMANDID_STRINGS.at( it->first ) + " --> " 
            +  STATEID_STRINGS.at(it->second) + " ( " + echo_transition_type(jt->first) + ")\n";     
      }
    }
  }
  return ret;
}
std::string echo_feasible_modeoperation( ) 
{
  std::string ret;
  ret += "Feasible Mode of operations:\n";
  for( ModeOperationIDStrings::const_iterator jt = MODEOPERATIONSID_STRINGS.begin()
     ; jt != MODEOPERATIONSID_STRINGS.end()
     ; jt++ )
  {
    ret += jt->second +"\n";
  }
  return ret;
}
std::string echo_transition_type( const TransitionID& transition )
{
  std::string ret;
  switch( TRANSITION_TYPE_MAP.at(transition) )
  {
    case TRANSITION_INTERNAL  : ret = "TRANSITION_INTERNAL" ; break;
    case TRANSITION_FWD       : ret = "TRANSITION_FWD"      ; break;
    case TRANSITION_BCK       : ret = "TRANSITION_BCK"      ; break;
    case TRANSITION_ALARM     : ret = "TRANSITION_ALARM"    ; break;
    case TRANSITION_FOO       : ret = "TRANSITION_FOO"    ; break;
  }
  return ret;
}


}
}
