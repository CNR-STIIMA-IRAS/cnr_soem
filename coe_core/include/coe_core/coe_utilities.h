/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */
#ifndef COE_CORE_COE_UTILITIES
#define COE_CORE_COE_UTILITIES

#include <string>
#include <typeinfo>
#include <bits/stdc++.h>
#include <coe_core/coe_base.h>

namespace coe_core 
{

template<typename R> bool checkType( const BaseDataObjectEntryPtr cob ) 
{
  return( getType<R>() == cob->type() );
}

template<typename R> R get( const BaseDataObjectEntryPtr cob ) 
{
  assert( getType<R>() == cob->type() );
  return ( ( coe_core::DataObjectEntry< R >*)( cob.get() ) )->value();
}

template<typename T> std::string to_string( const ::coe_core::DataObjectEntry<T>& in ) {

    char buffer[2048] = {0};
    sprintf( buffer, " Index: 0x%x, sub-index: %d, Name: %s, Type: %s, dim (bytes) %d "
            , in.addr(), in.subindex(), in.id(), typeid(in.val).name(), sizeof(T) );

    return buffer;
}

}

#endif  /* COE_CORE_COE_UTILITIES */
