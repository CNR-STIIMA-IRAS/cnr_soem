#ifndef COE_CORE_COE_SDO__H
#define COE_CORE_COE_SDO__H

#include <coe_core/coe_base.h>

namespace coe_core 
{

struct Sdo : public WeakDataObject 
{
  std::map< size_t, bool > write_access;
};

}  // namespace coe_core 

#endif  // COE_CORE_COE_SDO__H
