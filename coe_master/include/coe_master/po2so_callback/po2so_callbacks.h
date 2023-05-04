#ifndef __STIIMA_SWEEEPE__OPCUA_SERVER__CALLBACK__
#define __STIIMA_SWEEEPE__OPCUA_SERVER__CALLBACK__

#include <stdio.h>
#include <signal.h>
#include <pthread.h>    // if needed to update the values as simulation case
#include <unistd.h>     // if needed to update the values as simulation case

#include <coe_master/po2so_callback/po2so_module.h>

class CallbackBase
{
public:
    // input: pointer to a unique C callback. 
    CallbackBase(PO2SOModuleSetupCallback pCCallback);
    void Free();

    PO2SOModuleSetupCallback Reserve(PPO2SOModuleSetupBase* instance,
                                     PPO2SOModuleSetupBase::PO2SOModuleSetupCallback method);

protected:
    static int StaticInvoke(int context, uint16_t slave);

private:
    PO2SOModuleSetupCallback m_pCCallback;
    PPO2SOModuleSetupBase* m_pClass;
    PPO2SOModuleSetupBase::PO2SOModuleSetupCallback m_pMethod;
};



template <int context> class DynamicCallback : public CallbackBase
{
public:
    DynamicCallback(): CallbackBase(&DynamicCallback<context>::GeneratedStaticFunction) { }

private:
    static int GeneratedStaticFunction ( uint16_t slave )
    {
        return StaticInvoke(context, slave);
    }
};


/**
 * 
 * 
 * 
 * 
 * 
 */
class MemberFunctionCallback
{
public:
    MemberFunctionCallback(PPO2SOModuleSetupBase* instance,
                           PPO2SOModuleSetupBase::PO2SOModuleSetupCallback method);
    ~MemberFunctionCallback();
public:
    operator PO2SOModuleSetupCallback() const
    {
        return m_cbCallback;
    }

    bool IsValid() const;
private:
    PO2SOModuleSetupCallback m_cbCallback;
    int m_nAllocIndex;

private:
//     MemberFunctionCallback( const MemberFunctionCallback& os );
//     MemberFunctionCallback& operator=( const MemberFunctionCallback& os );
};


#endif
