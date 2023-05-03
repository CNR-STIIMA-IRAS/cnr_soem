#include <coe_core/coe_utilities.h>
#include <coe_core/coe_string_utilities.h>
#include <boost/algorithm/string.hpp>
#include <inttypes.h>
#include <ethercat.h>

namespace coe_core {
    
      
      



std::string   to_vstring(const ec_state& in) {
  switch (in) 
  {
    case EC_STATE_NONE:         return "|   NO_STATE   |";
    case EC_STATE_INIT:         return "|  INIT_STATE  |";
    case EC_STATE_PRE_OP:       return "| PREOP_STATE  |";
    case EC_STATE_SAFE_OP:      return "| SAFEOP_STATE |";
    case EC_STATE_OPERATIONAL:  return "|   OP_STATE   |";
    case EC_STATE_BOOT:         return "|  STATE_BOOT  |";
    case EC_STATE_ERROR:        return "|  STATE_ERROR |";
  }
  return "EC_STATE_UNKNOWN:" + std::to_string( uint16_t( in ) );
}

std::string to_string ( const ec_datatype& in, bool fill_space )
{
  return dtype2string(in, fill_space);
}

std::string   to_string(const ec_state& in) {
  std::string ret = to_vstring( in );
  boost::algorithm::erase_all(ret, "|" );
  boost::algorithm::erase_all(ret, " " );
 
  return ret;
}



std::string dtype2string(const uint16_t& dtype, bool fill_spaces )
{
  char hstr[1024] = {0};
  
  if( fill_spaces )
  {
    switch(dtype)
    {
      case ECT_BOOLEAN:         sprintf(hstr, "   BOOLEAN");     break;
      case ECT_INTEGER8:        sprintf(hstr, "  INTEGER8");     break;
      case ECT_INTEGER16:       sprintf(hstr, " INTEGER16");     break;
      case ECT_INTEGER32:       sprintf(hstr, " INTEGER32");     break;
      case ECT_INTEGER24:       sprintf(hstr, " INTEGER24");     break;
      case ECT_INTEGER64:       sprintf(hstr, " INTEGER64");     break;
      case ECT_UNSIGNED8:       sprintf(hstr, " UNSIGNED8");     break;
      case ECT_UNSIGNED16:      sprintf(hstr, "UNSIGNED16");     break;
      case ECT_UNSIGNED32:      sprintf(hstr, "UNSIGNED32");     break;
      case ECT_UNSIGNED24:      sprintf(hstr, "UNSIGNED24");     break;
      case ECT_UNSIGNED64:      sprintf(hstr, "UNSIGNED64");     break;
      case ECT_REAL32:          sprintf(hstr, "    REAL32");     break;
      case ECT_REAL64:          sprintf(hstr, "    REAL64");     break;
      case ECT_BIT1:            sprintf(hstr, "      BIT1");     break;
      case ECT_BIT2:            sprintf(hstr, "      BIT2");     break;
      case ECT_BIT3:            sprintf(hstr, "      BIT3");     break;
      case ECT_BIT4:            sprintf(hstr, "      BIT4");     break;
      case ECT_BIT5:            sprintf(hstr, "      BIT5");     break;
      case ECT_BIT6:            sprintf(hstr, "      BIT6");     break;
      case ECT_BIT7:            sprintf(hstr, "      BIT7");     break;
      case ECT_BIT8:            sprintf(hstr, "      BIT8");     break;
      case ECT_VISIBLE_STRING:  sprintf(hstr, "VISIBLE_STRING"); break;
      case ECT_OCTET_STRING:    sprintf(hstr, "OCTET_STRING");   break;
      default:                  sprintf(hstr, "Type 0x%4.4X", dtype);
    }
  }
  else
  {
    switch(dtype)
    {
      case ECT_BOOLEAN:         sprintf(hstr, "BOOLEAN");         break;
      case ECT_INTEGER8:        sprintf(hstr, "INTEGER8");        break;
      case ECT_INTEGER16:       sprintf(hstr, "INTEGER16");       break;
      case ECT_INTEGER32:       sprintf(hstr, "INTEGER32");       break;
      case ECT_INTEGER24:       sprintf(hstr, "INTEGER24");       break;
      case ECT_INTEGER64:       sprintf(hstr, "INTEGER64");       break;
      case ECT_UNSIGNED8:       sprintf(hstr, "UNSIGNED8");       break;
      case ECT_UNSIGNED16:      sprintf(hstr, "UNSIGNED16");      break;
      case ECT_UNSIGNED32:      sprintf(hstr, "UNSIGNED32");      break;
      case ECT_UNSIGNED24:      sprintf(hstr, "UNSIGNED24");      break;
      case ECT_UNSIGNED64:      sprintf(hstr, "UNSIGNED64");      break;
      case ECT_REAL32:          sprintf(hstr, "REAL32");          break;
      case ECT_REAL64:          sprintf(hstr, "REAL64");          break;
      case ECT_BIT1:            sprintf(hstr, "BIT1");            break;
      case ECT_BIT2:            sprintf(hstr, "BIT2");            break;
      case ECT_BIT3:            sprintf(hstr, "BIT3");            break;
      case ECT_BIT4:            sprintf(hstr, "BIT4");            break;
      case ECT_BIT5:            sprintf(hstr, "BIT5");            break;
      case ECT_BIT6:            sprintf(hstr, "BIT6");            break;
      case ECT_BIT7:            sprintf(hstr, "BIT7");            break;
      case ECT_BIT8:            sprintf(hstr, "BIT8");            break;
      case ECT_VISIBLE_STRING:  sprintf(hstr, "VISIBLE_STRING");  break;
      case ECT_OCTET_STRING:    sprintf(hstr, "OCTET_STRING");    break;
      default:                  sprintf(hstr, "Type 0x%4.4X", dtype);
    }
  }
  return hstr;
}






}

