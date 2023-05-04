#ifndef COE_MASTER__PO2SO_MODULE_BASE__H
#define COE_MASTER__PO2SO_MODULE_BASE__H

#include <memory>

typedef int (*PO2SOModuleSetupCallback)(uint16_t slave);

class PPO2SOModuleSetupBase
{

  private:
  public:
  typedef std::shared_ptr<PPO2SOModuleSetupBase> Ptr;
  typedef int (PPO2SOModuleSetupBase::*PO2SOModuleSetupCallback)(uint16_t slave);

  PPO2SOModuleSetupBase() = default;
  virtual ~PPO2SOModuleSetupBase() = default;
  void run();
  void stop();
  virtual int PO2SOSetup(uint16_t slave) { return 0; }
};

typedef PPO2SOModuleSetupBase::Ptr PPO2SOModuleSetupBasePtr;

#endif // COE_MASTER__PO2SO_MODULE_BASE__H
