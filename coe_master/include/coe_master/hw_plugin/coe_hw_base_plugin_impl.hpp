#ifndef __COE__BASE_PLUGINS_ROS__IMPL__H___
#define __COE__BASE_PLUGINS_ROS__IMPL__H___

#include <std_msgs/Int16.h>
#include <realtime_utilities/realtime_utilities.h>
#include "coe_hw_base_plugin.h"

namespace coe_master
{
  
inline void CoeHwPlugin::errorDiagnostic( diagnostic_updater::DiagnosticStatusWrapper &stat )
{
  ros::Time   n  = ros::Time::now();
  std::string st = " [" + boost::posix_time::to_simple_string( n.toBoost() ) +"]";

  if( are_errors_active_ )
  {
    int dim = 0;

    error_mtx_.lock();
    auto const errors_active = errors_active_;
    error_mtx_.unlock();
    for( auto const & err_pair : errors_active )
    {
      std::string  key  = err_pair.first;
      std::string  val = err_pair.second;
      stat.add( key, val );
      dim++;
    }
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, module_->getIdentifier() + " [ " + std::to_string( dim ) + " items ]" + st);
  }
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,  module_->getIdentifier() + "[ None error ]" + st );

}






inline CoeHwPlugin::CoeHwPlugin ( ) 
  : initialized_      ( false )
  , prxpdo_           (nullptr)
  , prxpdo_previous_  (nullptr)
  , ptxpdo_           (nullptr)
  , ptxpdo_previous_  (nullptr)
  , stop_thread_      ( false )
  , are_errors_active_( false )
  , operational_time_ ( 0.001 ) 
{ 
  operation_mode_state_ = CoeHwPlugin::OPERATION_MODE_IDLE;
  bool debug = false;
  if (!nh_.getParam("coe_hw_plugin_debug", debug) )
  {
    ROS_WARN("The param '%s/%s' is not defined", nh_.getNamespace().c_str(), std::string( "coe_hw_plugin_debug" ).c_str() );
  }
  ROS_WARN("[ %s ] CoeHwPlugin::CoeHwPlugin() Verbosity level: '%s'. ", nh_.getNamespace().c_str(), (debug ? "DEBUG" : "INFO" ));

  if (debug)
  {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

}

inline CoeHwPlugin::~CoeHwPlugin ( ) 
{
  ROS_DEBUG("~CoeHwPlugin Plugin Destructor!");
  if(initialized_)
  {
    reset();
  }
  ROS_DEBUG("~CoeHwPlugin Plugin Destructed");
}
  
  
inline bool CoeHwPlugin::initialize(ros::NodeHandle& nh, const std::string& device_coe_parameter, const int address)
{

  nh_ = nh;
  ROS_INFO("CoeHwPlugin::initialize() Start!");
  
  cnr::param::node_t config;
  if( !nh.getParam(device_coe_parameter, config ) )
  {
    ROS_ERROR("Error in extracting data from '%s/%s'", nh.getNamespace().c_str(), device_coe_parameter.c_str() );
    return false;
  }
  if( !nh.getParam(device_coe_parameter+"/module_list", config ) )
  {
    ROS_ERROR("Error in extracting data from '%s/%s'", nh.getNamespace().c_str(), device_coe_parameter.c_str() );
    return false;
  }
  
  //---
  
  bool ok = false;
  std::string label ="na";
  for( int i=0; i<config.size(); i++ )
  {
    cnr::param::node_t module = config[i];
    int addr = rosparam_utilities::toInt   ( module,  coe_master::NetworkData::KeysId[ coe_master::NetworkData::ADDRESS] );
    label    = rosparam_utilities::toString( module,  coe_master::NetworkData::KeysId[ coe_master::NetworkData::LABEL  ] );
    if( addr ==  address )
    {
      ok = true;
      break;
    }    
  }
  if(!ok) 
  {
    ROS_ERROR("Error in extracting data from '%s/%s'. The address '%d'", nh.getNamespace().c_str(), device_coe_parameter.c_str() , address);
    return false;
  }
    
  std::string msg = std::string(RESET()) 
              + std::string("[ " + ( BOLDCYAN()   + label + RESET() ) + " ] " )
              + std::string("[ " + ( BOLDBLUE()   + std::string( "Init Params" ) + RESET() ) + " ] " )
              + std::string("[ " + ( BOLDYELLOW() + std::string( " PARAMS" ) + RESET() ) + " ] " );
  
  module_.reset( new coe_master::ModuleDescriptor( nh, device_coe_parameter, label, address, false ) );
  
  if( !nh.getParam(device_coe_parameter+"/operational_time", operational_time_) )
  {
    ROS_ERROR("Error in extracting data from '%s/%s/operational_time'", nh.getNamespace().c_str(), device_coe_parameter.c_str() );
    return false;
  }
  
  ROS_DEBUG_STREAM(msg << BOLDYELLOW() << "Init Node - Configuration from ROS param server" << RESET() );
  if( !module_->initNodeConfigurationFromParams ( ) )
  {
    ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Basic Configurations from Param Server failed. ");
    return false;
  }
  if( !module_->initNodeCoeConfigurationFromParams ( false ) )
  {
    ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Coe Configurations from Param Server failed. ");
    return false;
  }
  
  ROS_DEBUG_STREAM(msg << BOLDYELLOW() << "Init Handles" << RESET() );
  if( !module_->initHandles( ) )
  {
    ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Handles Configurations from Params failed. ");
    return false;
  }
  
  if( !module_->connectHandles( ) )
  {
    ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Handle Connection failed. ");
    return false;
  }
  // TODO check correctness throu network_
  
  set_sdo_  = nh.serviceClient<coe_ros_msgs::SetSdo>(device_coe_parameter+"/"+SET_SDO_SERVER_NAMESPACE);
  if (!set_sdo_.waitForExistence(ros::Duration(5)))
  {
    ROS_ERROR("No server found for service %s",set_sdo_.getService().c_str());
    return false;
  }
  get_sdo_  = nh.serviceClient<coe_ros_msgs::GetSdo>(device_coe_parameter+"/"+GET_SDO_SERVER_NAMESPACE);
  if (!get_sdo_.waitForExistence(ros::Duration(5)))
  {
    ROS_ERROR("No server found for service %s",get_sdo_.getService().c_str());
    return false;
  }
  ROS_DEBUG_STREAM(msg << BOLDYELLOW() << "Access to Shared Memory '"<<  BOLDCYAN() << module_->getIdentifier() << BOLDYELLOW()<< "' "<< RESET() << " Operational Time: " << operational_time_ << " WatchDog Deciamtion: " << module_->getWatchdogDecimation() );
  pdo_shared_memory_.reset( new coe_master::ModuleIPC( module_->getIdentifier(), operational_time_, module_->getWatchdogDecimation() ) );
  
  std::size_t rxpdo_shdim = pdo_shared_memory_->rx_pdo_.getSize(false);
  std::size_t txpdo_shdim = pdo_shared_memory_->tx_pdo_.getSize(false);
  
  ROS_DEBUG_STREAM(msg << "Allocate the shared memory object for '" << module_->getIdentifier() << "' [I:" << txpdo_shdim <<"B O:"<< rxpdo_shdim  <<"B]");
  
  prxpdo_          = rxpdo_shdim > 0  ? ( new uint8_t[ rxpdo_shdim ] ) : NULL;
  prxpdo_previous_ = rxpdo_shdim > 0  ? ( new uint8_t[ rxpdo_shdim ] ) : NULL;
  
  ptxpdo_          = txpdo_shdim > 0  ? ( new uint8_t[ txpdo_shdim ] ) : NULL;
  ptxpdo_previous_ = txpdo_shdim > 0  ? ( new uint8_t[ txpdo_shdim ] ) : NULL;
  
  if( prxpdo_           != NULL ) std::memset( prxpdo_, 0x0, rxpdo_shdim );
  if( prxpdo_previous_  != NULL ) std::memset( prxpdo_previous_, 0x0, rxpdo_shdim );
  
  if( ptxpdo_           != NULL ) std::memset( ptxpdo_, 0x0, txpdo_shdim );
  if( ptxpdo_previous_  != NULL ) std::memset( ptxpdo_previous_, 0x0, txpdo_shdim );
    
  if( ( !pdo_shared_memory_->rx_pdo_.bond() ) && (rxpdo_shdim > 0) )
  {
    ROS_ERROR ("Bonding failed ...");
    return false;
  }
  
  if( ( !pdo_shared_memory_->tx_pdo_.bond() ) && (txpdo_shdim > 0) )
  {
    ROS_ERROR ("Bonding failed ...");
    return false;
  }
  
  double diagnostic_period_parameter;
  if( !ros::NodeHandle("~").getParam( "diagnostic_period_parameter", diagnostic_period_parameter ) )
  {
    ROS_WARN("~diagnostic_period_parameter not in the ROSPARAM SERVER. Set to 100Hz");
    ros::NodeHandle("~").setParam( "diagnostic_period_parameter", 0.01 );
  }

  updater_.reset( new diagnostic_updater::Updater( ros::NodeHandle("~") ) );
  updater_->setHardwareID( module_->getIdentifier() );
  updater_->add( module_->getIdentifier(), boost::bind(&CoeHwPlugin::errorDiagnostic, this, _1) );
  
  boost::thread::attributes err_thread_attr;
  error_thread_.reset( new boost::thread( err_thread_attr, boost::bind( &CoeHwPlugin::errorThread, this) ) );
  
  clock_gettime(CLOCK_MONOTONIC, &read_time_);
  clock_gettime(CLOCK_MONOTONIC, &write_time_);
  initialized_ = true;

  operation_mode_state_ = CoeHwPlugin::OPERATION_MODE_RUNNING;

  plugin_private_nh_.reset( new ros::NodeHandle(nh.getNamespace() + "/" + module_->getIdentifier() ) );
  return true;
}

inline bool CoeHwPlugin::setHardRT ( )
{
  return pdo_shared_memory_->rx_pdo_.setHardRT() && pdo_shared_memory_->tx_pdo_.setHardRT(); 
}

inline bool CoeHwPlugin::setSoftRT ( )
{
  return pdo_shared_memory_->rx_pdo_.setSoftRT() && pdo_shared_memory_->tx_pdo_.setSoftRT(); 
}

inline CoeHwPlugin::Error CoeHwPlugin::read() 
{
  
  if( pdo_shared_memory_->tx_pdo_.getSize(false) == 0 )
    return CoeHwPlugin::COM_ERROR;

  try
  {
    assert( pdo_shared_memory_->tx_pdo_.getSize( false ) > 0 );
    if( pdo_shared_memory_->tx_pdo_.getSize(false) !=  module_->sizeInputs() )
    {
      ROS_ERROR("Dimension Mismatch. Module Name '%s', Shared Memory Dimension: %zuB, Module Inputs size %zuB", module_->getIdentifier().c_str(), pdo_shared_memory_->tx_pdo_.getSize(false), module_->sizeInputs() );
      assert(0);
    }
    assert( ptxpdo_previous_ != NULL );
    
    std::memcpy( ptxpdo_previous_, ptxpdo_, pdo_shared_memory_->tx_pdo_.getSize( false ) ); 
    
    coe_master::PdoIPC::ErrorCode errorcode;
    double tm;
    double latency;
    errorcode = pdo_shared_memory_->tx_pdo_.flush( &ptxpdo_[0], &tm, &latency, module_->sizeInputs() );
    if( errorcode )
    {
      ROS_ERROR("Broken linkage to the Shared Memory. Error: %s Abort.", pdo_shared_memory_->tx_pdo_.to_string( errorcode ).c_str() );
      return CoeHwPlugin::COM_ERROR;
    }
    
    module_->updateInputs(&ptxpdo_[0], false);

  }
  catch( std::exception& e )
  {
    ROS_ERROR("READ EXCEPTION: %s",e.what());
    return CoeHwPlugin::EXCEPTION_ERROR;
  }
  
  clock_gettime(CLOCK_MONOTONIC, &read_time_);
  return CoeHwPlugin::NONE_ERROR;
}

  

inline CoeHwPlugin::Error CoeHwPlugin::write() 
{
  struct timespec update_time;
 
  try
  {
    if( pdo_shared_memory_->rx_pdo_.getSize(false) == 0 )
      return CoeHwPlugin::NONE_ERROR;
    
    assert( pdo_shared_memory_->rx_pdo_.getSize(false) ==  module_->sizeOutputs() );
    assert( prxpdo_previous_ != NULL );

    std::memcpy( prxpdo_previous_, prxpdo_, pdo_shared_memory_->rx_pdo_.getSize( false ) );   
    
    module_->getRxPdo().flush( prxpdo_, false );

    clock_gettime(CLOCK_MONOTONIC, &update_time );
    double act_time = realtime_utilities::timer_to_s( &update_time ); 
    coe_master::PdoIPC::ErrorCode errorcode;
    errorcode=pdo_shared_memory_->rx_pdo_.update( &prxpdo_[0], act_time, module_->sizeOutputs()  );
    if(errorcode)
    {
      ROS_ERROR("Broken linkage to the Shared Memory. Error: %s Abort.", pdo_shared_memory_->rx_pdo_.to_string( errorcode ).c_str() );
      return CoeHwPlugin::COM_ERROR;
    }
  }
  catch( std::exception& e )
  {
    ROS_ERROR("WRITE EXCEPTION: %s",e.what());
    return CoeHwPlugin::EXCEPTION_ERROR;
  }
  
  clock_gettime(CLOCK_MONOTONIC, &write_time_);
  return CoeHwPlugin::NONE_ERROR;
}


inline CoeHwPlugin::Error CoeHwPlugin::safeWrite()
{
  struct timespec update_time;

  try
  {
    if( pdo_shared_memory_->rx_pdo_.getSize(false) == 0 )
      return CoeHwPlugin::NONE_ERROR;

    assert( pdo_shared_memory_->rx_pdo_.getSize(false) ==  module_->sizeOutputs() );
    assert( prxpdo_previous_ != NULL );

    std::memcpy( prxpdo_previous_, prxpdo_, pdo_shared_memory_->rx_pdo_.getSize( false ) );

    // module_->getRxPdo().flush( prxpdo_, false );
    std::memset( prxpdo_, 0x0, pdo_shared_memory_->rx_pdo_.getSize( false ) );

    clock_gettime(CLOCK_MONOTONIC, &update_time );
    double act_time = realtime_utilities::timer_to_s( &update_time );
    coe_master::PdoIPC::ErrorCode errorcode;
    errorcode=pdo_shared_memory_->rx_pdo_.update( &prxpdo_[0], act_time, module_->sizeOutputs()  );
    if(errorcode)
    {
      ROS_ERROR("Broken linkage to the Shared Memory. Error: %s Abort.", pdo_shared_memory_->rx_pdo_.to_string( errorcode ).c_str() );
      return CoeHwPlugin::COM_ERROR;
    }
  }
  catch( std::exception& e )
  {
    ROS_ERROR("WRITE EXCEPTION: %s",e.what());
    return CoeHwPlugin::EXCEPTION_ERROR;
  }

  clock_gettime(CLOCK_MONOTONIC, &write_time_);
  return CoeHwPlugin::NONE_ERROR;
}
  
inline bool CoeHwPlugin::readSdo ( coe_core::BaseDataObjectEntry* in)
{
  // ROS_INFO("SDO READ REQUEST: %s", in->to_string().c_str() );
  coe_ros_msgs::GetSdo::Request req;
  coe_ros_msgs::GetSdo::Response res;
  
  req.index     = in->index();
  req.subindex  = in->subindex();
  req.sdotype   = in->type() == ECT_UNSIGNED8  ? coe_ros_msgs::SetSdo::Request::TYPE_U8
                : in->type() == ECT_UNSIGNED16 ? coe_ros_msgs::SetSdo::Request::TYPE_U16
                : in->type() == ECT_UNSIGNED32 ? coe_ros_msgs::SetSdo::Request::TYPE_U32
                : in->type() == ECT_UNSIGNED64 ? coe_ros_msgs::SetSdo::Request::TYPE_U64
                : in->type() == ECT_INTEGER8   ? coe_ros_msgs::SetSdo::Request::TYPE_I8
                : in->type() == ECT_INTEGER16  ? coe_ros_msgs::SetSdo::Request::TYPE_I16
                : in->type() == ECT_INTEGER32  ? coe_ros_msgs::SetSdo::Request::TYPE_I32
                : in->type() == ECT_INTEGER64  ? coe_ros_msgs::SetSdo::Request::TYPE_I64
                : 99;

  assert( req.sdotype != 99 );
  
  req.desc      = in->name();
  req.module_id = module_->getIdentifier();
  req.timeout   = 0.1;
  
  if( !set_sdo_.exists() )
  {
    ROS_FATAL_THROTTLE(2,"Error. The service '%s' is not available", set_sdo_.getService().c_str() );
    return false;
  }
  
  if( !get_sdo_.call(req,res) )
  {
    ROS_FATAL_THROTTLE(2,"Error. Server does not answer to the call. ");
    return false;
  }
  if( !res.success )
  {
    ROS_FATAL("Error. Getting the sdo '%x:%u' failed. ", in->index(), in->subindex() );
    return false;
  }
  // ROS_INFO("Read SDO Success, %lu" , *(uint64_t*)&(res.value[0]) );
  fromBoostArray( res.value, in);
  return true;
}
    
inline bool CoeHwPlugin::writeSdo ( const coe_core::BaseDataObjectEntry* in) 
{
  // ROS_DEBUG_STREAM( "[" << BOLDMAGENTA() << " PROCESS "<< RESET() << "] Set COB-ID: " << in->to_string() );
  coe_ros_msgs::SetSdo::Request req;
  coe_ros_msgs::SetSdo::Response res;
  
  req.index     = in->index();
  req.subindex  = in->subindex();

  req.sdotype   = in->type() == ECT_UNSIGNED8  ? coe_ros_msgs::SetSdo::Request::TYPE_U8
                : in->type() == ECT_UNSIGNED16 ? coe_ros_msgs::SetSdo::Request::TYPE_U16
                : in->type() == ECT_UNSIGNED32 ? coe_ros_msgs::SetSdo::Request::TYPE_U32
                : in->type() == ECT_UNSIGNED64 ? coe_ros_msgs::SetSdo::Request::TYPE_U64
                : in->type() == ECT_INTEGER8   ? coe_ros_msgs::SetSdo::Request::TYPE_I8
                : in->type() == ECT_INTEGER16  ? coe_ros_msgs::SetSdo::Request::TYPE_I16
                : in->type() == ECT_INTEGER32  ? coe_ros_msgs::SetSdo::Request::TYPE_I32
                : in->type() == ECT_INTEGER64  ? coe_ros_msgs::SetSdo::Request::TYPE_I64
                : 99;

  assert( req.sdotype != 99 );
  
  toBoostArray(in, req.value) ;     
  
  req.desc      = in->name();
  
  req.module_id = module_->getIdentifier();
  
  req.timeout   = 0.1;
  
  if( !set_sdo_.exists() )
  {
    ROS_FATAL("Error. The service '%s' is not available", set_sdo_.getService().c_str() );
    return false;
  }
  
  if( !set_sdo_.call(req,res) )
  {
    ROS_FATAL("Error. Server does not asnwer to the call. ");
    return false;
  }
  if( !res.success )
  {
    ROS_ERROR_STREAM( "[" << BOLDRED() << "  FAILED "<< RESET() << "] Set COB-ID: " << in->to_string() );
    return false;
  }
  // ROS_DEBUG_STREAM( "[" << BOLDGREEN() << "   OK    "<< RESET() << "] Set COB-ID: " << in->to_string() );
  return true;
}



inline void  CoeHwPlugin::errorThread( )
{
  ros::Rate rt(100);
  ROS_INFO("[ %s%s%s ] The Loggger Thread is now running", BOLDCYAN(), module_->getIdentifier().c_str(), RESET() );
  ros::Publisher state_pub = nh_.advertise<std_msgs::Int16>(module_->getIdentifier()+"_operation_mode_state",1);
  ROS_INFO("[ %s%s%s ] The Operation MOde State publisher is now running", BOLDCYAN(), module_->getIdentifier().c_str(), RESET() );
  std_msgs::Int16 state_msg;
  state_msg.data = operation_mode_state_;
  state_pub.publish(state_msg);

  struct timespec read_time_prev;
  struct timespec write_time_prev;

  read_time_prev = read_time_;
  write_time_prev = write_time_;
  bool are_errors_active_prev = false;
  while( ros::ok() && !stop_thread_ )
  {
    are_errors_active_ = checkErrorsActive();
    if( are_errors_active_ && !are_errors_active_prev )
    {
      std::lock_guard<std::mutex> lock(error_mtx_);  
      getDeviceErrors( errors_active_ );
    }
    updater_->update();
    are_errors_active_prev = are_errors_active_;
    rt.sleep();

    double d1 = realtime_utilities::timer_difference_s( &read_time_, &read_time_prev );
    double d2 = realtime_utilities::timer_difference_s( &write_time_, &write_time_prev );

    if( ( std::fabs(d1) > 5 * pdo_shared_memory_->rx_pdo_.getWatchdog() )
    ||  ( std::fabs(d2) > 5 * pdo_shared_memory_->tx_pdo_.getWatchdog() ) )
    {
      ROS_WARN("PDO commnucation borken %f/%f, %f/%f"
               , std::fabs(d1) , 5 * pdo_shared_memory_->rx_pdo_.getWatchdog()
               , std::fabs(d2) , 5 * pdo_shared_memory_->tx_pdo_.getWatchdog() );
    }

    read_time_prev = read_time_;
    write_time_prev = write_time_;

    std_msgs::Int16 state_msg;
    state_msg.data = operation_mode_state_;
    state_pub.publish(state_msg);
  }
}

inline bool CoeHwPlugin::reset( )
{
    if( !initialized_ )
      return true;
    
  stop_thread_ = true;
  
  ROS_DEBUG("Join the error thread");
  if( error_thread_ )
  {
    stop_thread_ = true;
    error_thread_->join();
    error_thread_.reset();
  }

  ROS_DEBUG("Rx Break Bond!");
  if( pdo_shared_memory_->rx_pdo_.isBonded() )
    pdo_shared_memory_->rx_pdo_.breakBond();
  ROS_DEBUG("Tx Break Bond!");
  if( pdo_shared_memory_->tx_pdo_.isBonded() )
    pdo_shared_memory_->tx_pdo_.breakBond();
  
  ROS_DEBUG("Shutdown set_sdo service");
  set_sdo_.shutdown();
  
  ROS_DEBUG("Shutdown get_sdo service");
  get_sdo_.shutdown();
  
  ROS_DEBUG("Clean mem rxpdo");
  if( prxpdo_ != nullptr )          
  {
    delete [] prxpdo_;
    prxpdo_ = nullptr;
  }
  ROS_DEBUG("Clean mem txpdo");
  if( ptxpdo_ != nullptr )
  {
    delete [] ptxpdo_;
    ptxpdo_ = nullptr;
  }
  
  ROS_DEBUG("Clean mem rxpdo swap");
  if( prxpdo_previous_ != nullptr )
  {
    delete [] prxpdo_previous_;
    prxpdo_previous_ = nullptr;
  }
  
  ROS_DEBUG("Clean mem txpdo swap");
  if( ptxpdo_previous_ != nullptr )
  {
    delete [] ptxpdo_previous_;
    ptxpdo_previous_ = nullptr;
  }
  ROS_DEBUG("OK");  
  
  initialized_ = false;

  return true;
}

}

#endif
