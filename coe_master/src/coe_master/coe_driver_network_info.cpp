#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <regex>
#include <locale>  
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <realtime_utilities/realtime_utilities.h>
#include <coe_core/coe_sdo.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_core/coe_sdo_xmlrpc.h>
#include <coe_core/coe_pdo_xmlrpc.h>
#include <coe_core/coe_string_utilities.h>
#include <coe_soem_utilities/coe_soem_utilities.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_module_descriptor_xmlrpc.h>
#include <coe_master/modules/coe_network_descriptor_xmlrpc.h>
#include <coe_master/modules/coe_pdo_shared_memory.h>

#include "coe_master_utilities.h"

static const std::size_t         PRE_ALLOCATION_SIZE             = 100*1024*1024;
static const std::size_t         MY_STACK_SIZE                   = 100*1024;
static const std::string    COE_DEVICE_PARAMETER_NAMESPACE  = "/coe";

class CoeController
{
  std::shared_ptr<ros::NodeHandle>  nh_;
  cnr::param::node_t               root_config_;
  std::string                       adapter_;
  std::string                       outfilename_;
  
   SdoManager*                      sdo_;
   coe_master::NetworkDescriptorPtr network_;
    
public:
  
  CoeController()
  {
    // nothing to do so far
  }

  CoeController(ros::NodeHandle& nh, const std::string& coe_device_parameter_namespace) 
  {
    network_.reset( new coe_master::NetworkDescriptor( nh, coe_device_parameter_namespace ) );
    
    if(!init(nh, coe_device_parameter_namespace))
    {
      ROS_ERROR("Failed in ctor. Exit.");
      throw std::runtime_error("Failed in ctor. Exit.");
    }
  }
  
  bool init(ros::NodeHandle& nh, const std::string& coe_device_parameter_namespace)
  {
    nh_.reset( new ros::NodeHandle( nh,  coe_device_parameter_namespace ) );
    
    if( !nh_->getParam(coe_device_parameter_namespace, root_config_ ) )
    {
      ROS_ERROR_STREAM( "The root config is not in the rosparam server. " );
      return false;
    }
    
    if( !nh_->getParam(coe_device_parameter_namespace+"/adapter", adapter_ ) )
    {
      ROS_ERROR_STREAM( "The param "<< coe_device_parameter_namespace<<"/adapter"<<" is not in the rosparam server. " );
      return false;
    }
    ROS_ERROR_STREAM( "The param "<< coe_device_parameter_namespace<<"/adapter"<<" is: " << adapter_ );
    
    sdo_ = new SdoManager( network_, true );
    
    return true;
  }

  void coeFsmThread( )
  {

    if( !realtime_utilities::rt_init_thread( MY_STACK_SIZE, int prio, int sched, period_info*  pinfo, long  period_ns  ) )
    {
      ROS_FATAL("Failed in setting threwad rt properties. Exit. ");
      return;
    }
    
    ros::AsyncSpinner spinner( 4 );
    spinner.start();

    //------------------------------------------------------------------------------------------------------------------------------------------------------
    ROS_INFO("Starting slaveinfo");
    if ( !coe_soem_utilities::soem_init( adapter_, 10.0 ) )
    {
      return;
    }
    ROS_INFO("%d slaves found and configured.",ec_slavecount);
    //------------------------------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------------------------------   
    char* IOmap = coe_soem_utilities::soem_config( 10.0, false );
    
    std::vector< std::tuple<std::string,uint16_t,bool, bool> > module_list; 
    for( int i = 1; i <= ec_slavecount; i++ )
    {
      std::regex  allowed_chars_in_name("^[A-Za-z][\w_]*$");
      std::string device_name_from_coe = ec_slave[i].name;
      if(!std::regex_match(device_name_from_coe ,allowed_chars_in_name))
      {
        device_name_from_coe  = std::regex_replace(device_name_from_coe , std::regex(R"([^A-Za-z\d])"), "");
      }
      boost::to_lower(device_name_from_coe);
      
      std::tuple<std::string,uint16_t,bool, bool> module_basic_info;
      std::get<0>( module_basic_info ) = device_name_from_coe;
      std::get<1>( module_basic_info ) = i;
      std::get<2>( module_basic_info ) = true;
      std::get<3>( module_basic_info ) = true;
      
      module_list.push_back( module_basic_info );
    }
    
    coe_master::NetworkData::setParam(*nh_, "module_list", module_list );

    for( int iSlave = 1; iSlave <= ec_slavecount; iSlave++ )
    {
      ROS_INFO("*************************** Slave %d ******************************* ", iSlave);
      
      coe_master::ModuleDescriptorPtr  module( new coe_master::ModuleDescriptor( *nh_, "", std::get<0>( module_list.at(iSlave-1) ), iSlave, true ) );
      
      module->initNodeConfigurationFromParams( );
      module->initNodeCoeConfigurationFromSoem(iSlave, ec_slave[iSlave].name, (ec_slave[iSlave].CoEdetails & ECT_COEDET_SDOCA ) );
      
      updateNodeConfiguration(module, IOmap );
      
      ROS_INFO("Set params to rosparam server (%s)", module->getIdentifier().c_str() );
      cnr::param::node_t xml_module;
      coe_master::ModuleData::get(*module,xml_module);
      nh_->setParam(module->getIdentifier(),xml_module);
    }
    
    if( !network_->initNetworkNames( ) )
    {
      ROS_FATAL("Fail in extracting the coe configuration information from ros param server. Abort.");
      return;
    }

    while ( !coe_soem_utilities::soem_reset_to_operational_state( ) )
    {
      ROS_INFO("SOEM wait for EC_STATE_OPERATIONAL state ... try again ... ");
      if( !ros::ok() )
      {
        ec_close();
        return;
      }
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
    }
    ROS_INFO("Operational state reached for all slaves.");
    
    while( ros::ok() )
    {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
    }
    ec_close();

    return;
  }
  
  bool run()
  {
    try 
    {
        ROS_INFO("Prepare RT thread.");

        boost::thread::attributes coe_fsm_thread_attr;
        
        coe_fsm_thread_attr.set_stack_size( PTHREAD_STACK_MIN + MY_STACK_SIZE );

        ROS_INFO("Create FSM Coe thread.");
        boost::thread coe_fsm_thread( coe_fsm_thread_attr, boost::bind(&CoeController::coeFsmThread, this) );
        coe_fsm_thread.join();
        
        ROS_INFO("Wait for shutdown..");
        ros::waitForShutdown();
    }
    catch( std::exception& e )
    {
      ROS_ERROR("%s", e.what());
      ROS_ERROR("Abort.");
      return false;
    }
    catch (...)
    {
      ROS_ERROR("Unhandled exception ");
      ROS_ERROR("Abort.");
      return false;
    }
    return true;
  }
  
  
};


std::shared_ptr<CoeController> controller;

int main(int argc, char* argv[])
{
  if( !realtime_utilities::rt_main_init(PRE_ALLOCATION_SIZE) )
  {
    perror("Error in rt_main_init. Exit.");
    return -1;
  }

  ros::init(argc,argv,"coe_master");
  ros::NodeHandle nh("~");
  
  ros::AsyncSpinner spinner( 4 );
  spinner.start();
  
  try
  {
    controller.reset( new CoeController(nh,COE_DEVICE_PARAMETER_NAMESPACE ) );
    
    controller->run();
  }
  catch( std::exception& e )
  {
    ROS_ERROR("%s", e.what());
    ROS_ERROR("Abort.");
    return -1;
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception ");
    ROS_ERROR("Abort.");
    return -1;
  }

return 0;
}
