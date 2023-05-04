#include <coe_master/po2so_callback/po2so_callbacks.h>
#include <coe_master/po2so_callback/po2so_module.h>

CallbackBase *AvailableCallbackSlots[] = {
    new DynamicCallback<0x00>(), new DynamicCallback<0x01>(), new DynamicCallback<0x02>(), new DynamicCallback<0x03>(),
    new DynamicCallback<0x04>(), new DynamicCallback<0x05>(), new DynamicCallback<0x06>(), new DynamicCallback<0x07>(),
    new DynamicCallback<0x08>(), new DynamicCallback<0x09>(), new DynamicCallback<0x0A>(), new DynamicCallback<0x0B>(),
    new DynamicCallback<0x0C>(), new DynamicCallback<0x0D>(), new DynamicCallback<0x0E>(), new DynamicCallback<0x0F>(),
};

CallbackBase::CallbackBase(PO2SOModuleSetupCallback pCCallback)
    : m_pCCallback(pCCallback), m_pClass(NULL), m_pMethod(NULL)
{
}

// when done, remove allocation of the callback
void CallbackBase::Free() { m_pClass = NULL; }

// when free, allocate this callback
PO2SOModuleSetupCallback CallbackBase::Reserve(PPO2SOModuleSetupBase *instance,
                                               PPO2SOModuleSetupBase::PO2SOModuleSetupCallback method)
{
  if (m_pClass)
    return NULL;

  m_pClass = instance;
  m_pMethod = method;
  return m_pCCallback;
}

int CallbackBase::StaticInvoke(int context, uint16_t slave)
{
  return ((AvailableCallbackSlots[context]->m_pClass)->*(AvailableCallbackSlots[context]->m_pMethod))(slave);
}

MemberFunctionCallback::MemberFunctionCallback(PPO2SOModuleSetupBase *instance,
                                               PPO2SOModuleSetupBase::PO2SOModuleSetupCallback method)
{
  int imax = sizeof(AvailableCallbackSlots) / sizeof(AvailableCallbackSlots[0]);
  for (m_nAllocIndex = 0; m_nAllocIndex < imax; ++m_nAllocIndex)
  {
    m_cbCallback = AvailableCallbackSlots[m_nAllocIndex]->Reserve(instance, method);
    if (m_cbCallback != NULL)
      break;
  }
}
MemberFunctionCallback::~MemberFunctionCallback()
{
  if (IsValid())
  {
    AvailableCallbackSlots[m_nAllocIndex]->Free();
  }
}

bool MemberFunctionCallback::IsValid() const { return m_cbCallback != NULL; }
