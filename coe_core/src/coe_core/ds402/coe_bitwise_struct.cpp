
#include <string>
#include <vector>
#include <coe_core/coe_utilities.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_core/ds402/coe_xfsm_utilities.h>
#include <coe_core/ds402/coe_bitwise_struct.h>


namespace coe_core
{
namespace ds402
{


uint32_t&  operator<< ( uint32_t& lhs, const digital_inputs_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint32_t ) );
    return lhs;
}
digital_inputs_t&            operator<< ( digital_inputs_t& lhs, const uint32_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint32_t ) );
    return lhs;
}

uint16_t&  operator<< ( uint16_t& lhs, const status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
status_word_t&               operator<< ( status_word_t& lhs, const uint16_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}

//
uint16_t&  operator<< ( uint16_t& lhs, const homing_status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
homing_status_word_t&        operator<< ( homing_status_word_t& lhs, const uint16_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
status_word_t&               operator<< ( status_word_t& lhs, const homing_status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
homing_status_word_t&        operator<< ( homing_status_word_t& lhs, const status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
uint16_t&  operator<< ( uint16_t& lhs, const profiled_vel_status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
profiled_vel_status_word_t&  operator<< ( profiled_vel_status_word_t& lhs, const uint16_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
status_word_t&               operator<< ( status_word_t& lhs, const profiled_vel_status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
profiled_vel_status_word_t&  operator<< ( profiled_vel_status_word_t& lhs, const status_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
uint16_t&  operator<< ( uint16_t& lhs, const control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
control_word_t& operator<< ( control_word_t& lhs, const uint16_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}

void  set ( const control_word_t* in, uint16_t* out )
{
    std::memcpy ( out, in, sizeof ( uint16_t ) );
}
void       set ( const uint16_t* in, control_word_t* out )
{
    std::memcpy ( out, in, sizeof ( uint16_t ) );
}

uint16_t&  operator<< ( uint16_t& lhs, const homing_control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
homing_control_word_t&       operator<< ( homing_control_word_t& lhs, const uint16_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
control_word_t&              operator<< ( control_word_t& lhs, const homing_control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
homing_control_word_t&       operator<< ( homing_control_word_t& lhs, const control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}

uint16_t&  operator<< ( uint16_t& lhs, const profiled_vel_control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
profiled_vel_control_word_t& operator<< ( profiled_vel_control_word_t& lhs, const uint16_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
control_word_t&              operator<< ( control_word_t& lhs, const profiled_vel_control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
profiled_vel_control_word_t& operator<< ( profiled_vel_control_word_t& lhs, const control_word_t& rhs )
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}


digital_inputs_t  to_digital_input ( const uint32_t& in )
{
    digital_inputs_t ret;
    ret.value = in;
    return ret;
}

status_word_t to_status_word ( const uint16_t& in )
{
    status_word_t ret;
    ret.value = in;
    return ret;
}
homing_status_word_t to_homing_status_word ( const uint16_t& in )
{
    homing_status_word_t ret;
    ret.value = in;
    return ret;
}
profiled_vel_status_word_t to_profiled_vel_status_word ( const uint16_t& in )
{
    profiled_vel_status_word_t ret;
    ret.value = in;
    return ret;
}

control_word_t to_control_word ( const uint16_t& in )
{
    control_word_t ret;
    ret.value = in;
    return ret;
}
homing_control_word_t to_homing_control_word ( const uint16_t& in)
{
    homing_control_word_t ret;
    ret.value = in;
    return ret;
}
profiled_vel_control_word_t to_profiled_vel_control_word ( const uint16_t& in )
{
    profiled_vel_control_word_t ret;
    ret.value = in;
    return ret;
}

std::string to_string ( const digital_inputs_t& in )
{
    std::string ret;
    ret +=" Digital Input  : " + coe_core::to_string_hex( in.value ) +", "+ coe_core::to_string_bin ( in.value ) + std::string ( "\n" );
    ret +=" " + ( in.bits.negative_limit_switch   ? std::string ( "1" ) : std::string ( "0" ) ) + ".neg_switch " +  std::string ( "\n" );
    ret +=" " + ( in.bits.positive_limit_switch   ? std::string ( "1" ) : std::string ( "0" ) ) + ".pos_switch " +  std::string ( "\n" );
    ret +=" " + ( in.bits.home_switch             ? std::string ( "1" ) : std::string ( "0" ) ) + ".home_switch" +  std::string ( "\n" );
    ret +=" " + ( in.bits.interlock               ? std::string ( "1" ) : std::string ( "0" ) ) + ".interlock  " +  std::string ( "\n" );
    ret +=" " + ( in.bits.digital_1 ? std::string ( "1" ) : std::string ( "0" ) ) + ".d1" + std::string ( "\n" );
    ret +=" " + ( in.bits.digital_2 ? std::string ( "1" ) : std::string ( "0" ) ) + ".d2" + std::string ( "\n" );
    ret +=" " + ( in.bits.digital_3 ? std::string ( "1" ) : std::string ( "0" ) ) + ".d3" + std::string ( "\n" );
    ret +=" " + ( in.bits.digital_4 ? std::string ( "1" ) : std::string ( "0" ) ) + ".d4" + std::string ( "\n" );
    ret +=" " + ( in.bits.digital_5 ? std::string ( "1" ) : std::string ( "0" ) ) + ".d5" + std::string ( "\n" );
    ret +=" " + ( in.bits.digital_6 ? std::string ( "1" ) : std::string ( "0" ) ) + ".d6" ;
    return ret;
}

std::string to_string ( const status_word_t& in, char how )
{
    std::vector<std::string>    fields;
    std::string                 ret;

    if ( how == 's' )
    {

      ret = "\nStatus word: " + coe_core::to_string_bin ( in.value )
            + ", " + to_string ( to_stateid ( in.value ) ) +  " ( " + coe_core::to_string_hex ( in.value ) + ")\n";

      fields.push_back ( "int_limit    " );
      fields.push_back ( "trg_reached  " );
      fields.push_back ( "remote       " );
      fields.push_back ( "manufacturer " );
      fields.push_back ( "warning      " );
      fields.push_back ( "swt_on_dis   " );
      fields.push_back ( "quick_stop   " );
      fields.push_back ( "volt_enabled " );
      fields.push_back ( "fault        " );
      fields.push_back ( "op_enababled " );
      fields.push_back ( "switched_on  " );
      fields.push_back ( "ready_to_on  " );

      std::vector<std::string> tttmp ( 12,"0" );
      tttmp.at ( 0 ) = ( in.bits.ready_to_switch_on          ? "1" :"0" );
      tttmp.at ( 1 ) = ( in.bits.switched_on                 ? "1" :"0" );
      tttmp.at ( 2 ) = ( in.bits.operation_enabled           ? "1" :"0" );
      tttmp.at ( 3 ) = ( in.bits.fault                       ? "1" :"0" );
      tttmp.at ( 4 ) = ( in.bits.voltage_enabled             ? "1" :"0" );
      tttmp.at ( 5 ) = ( in.bits.quick_stop                  ? "1" :"0" );
      tttmp.at ( 6 ) = ( in.bits.switch_on_disabled          ? "1" :"0" );
      tttmp.at ( 7 ) = ( in.bits.warning                     ? "1" :"0" );
      tttmp.at ( 8 ) = ( in.bits.manufacturer_specific       ? "1" :"0" );
      tttmp.at ( 9 ) = ( in.bits.remote                      ? "1" :"0" );
      tttmp.at ( 10) = ( in.bits.target_reached              ? "1" :"0" );
      tttmp.at ( 11) = ( in.bits.internal_limit_active       ? "1" :"0" );

      for ( int i=11; i>=0; i-- )
          ret += tttmp.at ( i ) + "." + fields.at ( i ) + "\n";
      ret += "\n";

    }
    else
    {
      ret = coe_core::to_string_bin ( in.value );
    }
    return ret;
}

std::string to_string ( const homing_status_word_t& in, char how )
{
    std::string ret;

    if ( how == 's' )
    {
      ret = "\nStatus word: " + coe_core::to_string_bin ( in.value ) + ", "
            + to_string ( to_stateid ( in.value ) ) + " ( " + coe_core::to_string_hex ( in.value ) + ")\n";

      ret += "-- 13 |-- 12 |-- 11 |-- 10 |--- 9 |--- 8 |--- 7 |--- 6 |--- 5 |--- 4 |--- 3 |--- 2 |--- 1 |--- 0 |\n";
      ret += " hmng | hmng | int  |targt |remote| man. | warn |switch|quick | volt |fault | op   |swtchd|ready |\n";
      ret += " err. | att. | limit|reachd|      |      |      |on_dis|stop  | enbl |fault | enbl |   on |to on |\n";
      
      std::vector<std::string> tttmp ( 14,"0" );
      tttmp.at ( 0 ) = ( in.bits.ready_to_switch_on       ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 1 ) = ( in.bits.switched_on              ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 2 ) = ( in.bits.operation_enabled        ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 3 ) = ( in.bits.fault                    ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 4 ) = ( in.bits.voltage_enabled          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 5 ) = ( in.bits.quick_stop               ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 6 ) = ( in.bits.switch_on_disabled       ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 7 ) = ( in.bits.warning                  ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 8 ) = ( in.bits.manufacturer_specific    ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 9 ) = ( in.bits.remote                   ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 10 ) = ( in.bits.target_reached          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 11 ) = ( in.bits.internal_limit_active   ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 12 ) = ( in.bits.homing_attained         ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 13 ) = ( in.bits.homing_error            ? std::string ( "1" ) : std::string ( "0" ) );

      for ( int i=13; i>=0; i-- )
          ret +="    " + tttmp.at ( i ) + " |";
      ret += "\n";

    }
    else
    {
      ret = coe_core::to_string_bin ( in.value );
    }
    return ret;
}

std::string to_string ( const profiled_vel_status_word_t& in, char how )
{
    std::string ret;

    if ( how == 's' )
    {
      ret = "\nStatus word: " + coe_core::to_string_bin ( in.value ) + ", "
            + to_string ( to_stateid ( in.value ) ) + "( " + coe_core::to_string_hex ( in.value ) + " )\n";
      ret += "---13 |---12 |---11 |---10 |--- 9 |--- 8 |--- 7 |--- 6 |--- 5 |--- 4 |--- 3 |--- 2 |--- 1 |--- 0 |\n";
      ret += " max  |speed | int  | targt|remote| man  | warn |switch|quick | volt |fault | op   |swtchd|ready |\n";
      ret += " slip |      | limit|reachd|      |      |      |on dis| stop | enbl |fault | enbl |on    |to on |\n";

      std::vector<std::string> tttmp ( 14,"0" );
      tttmp.at ( 0 ) = ( in.bits.ready_to_switch_on       ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 1 ) = ( in.bits.switched_on              ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 2 ) = ( in.bits.operation_enabled        ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 3 ) = ( in.bits.fault  ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 4 ) = ( in.bits.voltage_enabled          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 5 ) = ( in.bits.quick_stop               ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 6 ) = ( in.bits.switch_on_disabled       ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 7 ) = ( in.bits.warning                  ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 8 ) = ( in.bits.manufacturer_specific    ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 9 ) = ( in.bits.remote                   ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 10 ) = ( in.bits.target_reached           ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 11 ) = ( in.bits.internal_limit_active    ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 12 ) = ( in.bits.speed  ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 13 ) = ( in.bits.maximum_slippage         ? std::string ( "1" ) : std::string ( "0" ) );

      for ( int i=13; i>=0; i-- )
          ret +="    " + tttmp.at ( i ) + " |";
      ret += "\n";
    }
    else
    {
        uint16_t tmp;
        tmp << in;
        ret = coe_core::to_string_bin<uint16_t> ( tmp );
    }
    return ret;
}


std::string to_string ( const control_word_t& in, char how )
{
    std::string ret;

    if ( how == 's' )
    {

      ret  = "\nControl word: " + coe_core::to_string_bin ( in.value ) + "\n";
      ret += "Control word: " + coe_core::to_string_hex ( in.value ) + "\n";
      ret += "halt      |fault rst |enable op |quick stop| enbl volt| switch on|\n";
        
      std::vector<std::string> tttmp ( 6,"0" );
      tttmp.at ( 0 ) = ( in.bits.switch_on           ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 1 ) = ( in.bits.enable_voltage      ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 2 ) = ( in.bits.quick_stop          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 3 ) = ( in.bits.enable_operation    ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 4 ) = ( in.bits.fault_reset         ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 5 ) = ( in.bits.halt                ? std::string ( "1" ) : std::string ( "0" ) );

      for ( int i=5; i>=0; i-- )
          ret +="        " + tttmp.at ( i ) + " |";
      ret += "\n";

    }
    else
    {
      ret = coe_core::to_string_bin( in.value );
    }
    return ret;
}

std::string to_string ( const homing_control_word_t& in, char how )
{
    std::string ret;

    if ( how == 's' )
    {

      ret  = "\nControl word: " + coe_core::to_string_bin ( in.value ) + "\n";
      ret += "Control word: " + coe_core::to_string_hex ( in.value ) + "\n";
      ret += "halt         |fault_reset  | reserved    | reserved    | home start  | enable op   | quick stop  | enable volt | switch on   |\n";

      std::vector<std::string> tttmp ( 9,"0" );
      tttmp.at ( 0 ) = ( in.bits.switch_on           ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 1 ) = ( in.bits.enable_voltage      ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 2 ) = ( in.bits.quick_stop          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 3 ) = ( in.bits.enable_operation     ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 4 ) = ( in.bits.home_operation_start? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 5 ) = ( in.bits.reserved_2          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 6 ) = ( in.bits.reserved_2          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 7 ) = ( in.bits.fault_reset         ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 8 ) = ( in.bits.halt                ? std::string ( "1" ) : std::string ( "0" ) );

      for ( int i=8; i>=0; i-- )
          ret +="           " + tttmp.at ( i ) + " |";
      ret += "\n";
      ret += "-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|\n";
      ret += "            8|            7|            6|            5|            4|            3|            2|            1|            0|\n";
    }
    else
    {
        ret = coe_core::to_string_bin ( in.value );
    }
    return ret;
}


std::string to_string ( const profiled_vel_control_word_t& in, char how )
{
    std::string ret;

    if ( how == 's' )
    {
      ret  = "\nProfiled Vel Control word: 0b" + coe_core::to_string_bin ( in.value ) + "\n";
      ret += "Control word: 0x" + coe_core::to_string_bin ( in.value ) + "\n";
      ret += "halt         |fault_reset  | rfg_use_ref | rfg_unlock  | rfg_enable  | enable op   | quick stop  | enable volt | switch on   |\n";

      std::vector<std::string> tttmp ( 9,"0" );
      tttmp.at ( 0 ) = ( in.bits.switch_on          ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 1 ) = ( in.bits.enable_voltage     ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 2 ) = ( in.bits.quick_stop         ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 3 ) = ( in.bits.enable_operation   ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 4 ) = ( in.bits.rfg_enable         ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 5 ) = ( in.bits.rfg_unlock         ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 6 ) = ( in.bits.rfg_use_ref        ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 7 ) = ( in.bits.fault_reset        ? std::string ( "1" ) : std::string ( "0" ) );
      tttmp.at ( 8 ) = ( in.bits.halt               ? std::string ( "1" ) : std::string ( "0" ) );

      for ( int i=8; i>=0; i-- )
          ret +="           " + tttmp.at ( i ) + " |";
      ret += "\n";
      ret += "\t-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|\n";
      ret += "\t            8|            7|            6|            5|            4|            3|            2|            1|            0|\n";

    }
    else
    {
      ret = coe_core::to_string_bin ( in.value );
    }
    return ret;
}


//---------
cyclic_vel_status_word_t to_cyclic_vel_status_word ( const uint16_t& in )
{
  cyclic_vel_status_word_t ret;
  ret.value = in;
  return ret;
}

cyclic_vel_status_word_t& operator<<( cyclic_vel_status_word_t& lhs, const uint16_t& rhs)
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
uint16_t&  operator<<( uint16_t& lhs, const cyclic_vel_status_word_t& rhs)
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
status_word_t& operator<<( status_word_t& lhs, const cyclic_vel_status_word_t& rhs)
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
cyclic_vel_status_word_t& operator<<( cyclic_vel_status_word_t& lhs, const status_word_t& rhs)
{
    std::memcpy ( &lhs, &rhs, sizeof ( uint16_t ) );
    return lhs;
}
std::string to_string(const cyclic_vel_status_word_t&        in, char how )
{
  std::string ret;

  if ( how == 's' )
  {
    ret = "\nStatus word: " + coe_core::to_string_bin ( in.value ) + ", "
          + to_string ( to_stateid ( in.value ) ) + "( " + coe_core::to_string_hex ( in.value ) + " )\n";
    ret += "---13 |---12 |---11 |---10 |--- 9 |--- 8 |--- 7 |--- 6 |--- 5 |--- 4 |--- 3 |--- 2 |--- 1 |--- 0 |\n";
    ret += " cmd  |fllw  | int  | targt|remote| man  | warn |switch|quick | volt |fault | op   |swtchd|ready |\n";
    ret += " fllw |error | limit|reachd|      |      |      |on dis| stop | enbl |fault | enbl |on    |to on |\n";

    std::vector<std::string> tttmp ( 14,"0" );
    tttmp.at (  0 ) = ( in.bits.ready_to_switch_on         ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  1 ) = ( in.bits.switched_on                ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  2 ) = ( in.bits.operation_enabled          ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  3 ) = ( in.bits.fault                      ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  4 ) = ( in.bits.voltage_enabled            ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  5 ) = ( in.bits.quick_stop                 ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  6 ) = ( in.bits.switch_on_disabled         ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  7 ) = ( in.bits.warning                    ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  8 ) = ( in.bits.manufacturer_specific      ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at (  9 ) = ( in.bits.remote                     ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at ( 10 ) = ( in.bits.target_reached             ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at ( 11 ) = ( in.bits.internal_limit_active      ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at ( 12 ) = ( in.bits.following_error            ? std::string ( "1" ) : std::string ( "0" ) );
    tttmp.at ( 13 ) = ( in.bits.drive_follows_the_command  ? std::string ( "1" ) : std::string ( "0" ) );

    for ( int i=13; i>=0; i-- )
        ret +="    " + tttmp.at ( i ) + " |";
    ret += "\n";
  }
  else
  {
      uint16_t tmp;
      tmp << in;
      ret = coe_core::to_string_bin<uint16_t> ( tmp );
  }
  return ret;
}


cyclic_vel_control_word_t to_cyclic_vel_control_word( const uint16_t& in)
{
  cyclic_vel_control_word_t ret;
  ret.value = in;
  return ret; 
}






//---------
cyclic_pos_status_word_t to_cyclic_pos_status_word ( const uint16_t& in )
{
  cyclic_pos_status_word_t ret;
  ret.value = in;
  return ret;
}

cyclic_pos_control_word_t to_cyclic_pos_control_word( const uint16_t& in)
{
  cyclic_pos_control_word_t ret;
  ret.value = in;
  return ret; 
}







}
}
