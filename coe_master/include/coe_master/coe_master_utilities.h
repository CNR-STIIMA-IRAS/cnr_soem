#ifndef __COE_DRIVER_UTILITES__H__
#define __COE_DRIVER_UTILITES__H__

#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cinttypes>
#include <csignal>

#include <ethercat.h>
#include <ethercattype.h>
#include <ethercatdc.h>
#include <thread>

//#include <diagnostic_updater/diagnostic_updater.h>

 
#include <cnr_param/cnr_param.hpp>
#include <rtos_utilities/rtos_utilities.h>
#include <rtos_utilities/circular_buffer.h>

#include <coe_core/coe_sdo.h>
#include <coe_core/coe_utilities.h>

#include <coe_soem_utilities/coe_soem_utilities.h>
#include <coe_master/ipc/coe_ipc.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_network_descriptor.h>
#include <coe_master/modules/coe_srv_utilities.h>

class MainThreadSharedData
{
private:
  
  const int         windows_dim_;
  std::mutex        mtx_;
  double            cycle_time_;
  int               expectedWKC_;
  char*             IOmap_;
  rtos_utilities::circ_buffer<double>   latency_msr_;
  rtos_utilities::circ_buffer<double>   cycle_time_msr_;
  rtos_utilities::circ_buffer<double>   calc_time_;
  rtos_utilities::circ_buffer<uint32_t> missed_cycles_;
  rtos_utilities::circ_buffer<int>      wkc_;
  std::map<std::string, bool>         rxbond_;
  std::map<std::string, bool>         txbond_;
  std::map<std::string, bool>         rxhardrt_;
  std::map<std::string, bool>         txhardrt_;
public:
  MainThreadSharedData(const int windows_dim ) 
    : windows_dim_   (windows_dim)
    , cycle_time_    (0)
    , expectedWKC_   (0)
    , IOmap_         (NULL)
    , latency_msr_   (windows_dim)
    , cycle_time_msr_(windows_dim)
    , calc_time_     (windows_dim)
    , missed_cycles_ (windows_dim)
    , wkc_           (windows_dim)
  {
  }
  
  int      getWindowDim        ( ) { return windows_dim_;   }
  double   getCycleTime        ( ) { std::lock_guard<std::mutex> lock(mtx_);  return cycle_time_;   }
  int      getExpectedWKC      ( ) { std::lock_guard<std::mutex> lock(mtx_);  return expectedWKC_;  }
  char*    getIOmap            ( ) { std::lock_guard<std::mutex> lock(mtx_);  return &IOmap_[0];    }

  double   getMeanActCycleTime ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::mean( cycle_time_msr_.get() ) * 1e3;    }
  double   getMeanCalcTime     ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::mean( calc_time_.get() ) * 1e3;    }
  double   getMeanLatencyTime  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::mean( latency_msr_.get() ) * 1e3;    }
  uint32_t getMeanMissedCycles ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::mean( missed_cycles_.get() );}
  int      getMeanWkc          ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::mean( wkc_.get() );          }
  
  double   getMaxActCycleTime  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::max( cycle_time_msr_.get() ) * 1e3;    }
  double   getMaxCalcTime      ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::max( calc_time_.get() ) * 1e3;    }
  double   getMaxLatencyTime   ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::max( latency_msr_.get() ) * 1e3;    }
  uint32_t getMaxMissedCycles  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::max( missed_cycles_.get() );}
  int      getMaxWkc           ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::max( wkc_.get() );          }

  double   getMinActCycleTime  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::min( cycle_time_msr_.get() ) * 1e3;    }
  double   getMinCalcTime      ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::min( calc_time_.get() ) * 1e3;    }
  double   getMinLatencyTime   ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::min( latency_msr_.get() ) * 1e3;    }
  uint32_t getMinMissedCycles  ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::min( missed_cycles_.get() );}
  int      getMinWkc           ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rtos_utilities::min( wkc_.get() );          }

  void     setCycleTime        ( double v ) { std::lock_guard<std::mutex> lock(mtx_);  cycle_time_    = v; }
  void     setExpectedWKC      ( int    v ) { std::lock_guard<std::mutex> lock(mtx_);  expectedWKC_   = v; }
  void     setIOmap            ( char*  v ) { std::lock_guard<std::mutex> lock(mtx_);  IOmap_         = v; }

  void     setActualCycleTime  ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  cycle_time_msr_.push_back( v ); }
  void     setLatencyTime      ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  latency_msr_.push_back( v ); }
  void     setCalcTime         ( double    v ) { std::lock_guard<std::mutex> lock(mtx_);  calc_time_.push_back( v ); }
  void     setMissedCycles     ( uint32_t  v ) { std::lock_guard<std::mutex> lock(mtx_);  missed_cycles_.push_back( v ); }
  int      setWkc              ( int       v ) { std::lock_guard<std::mutex> lock(mtx_);  wkc_.push_back( v ); return v; }
  
  void setRxBonded ( std::string s, bool v ) { std::lock_guard<std::mutex> lock(mtx_);  rxbond_[ s ] = v; }
  std::map<std::string,bool> getRxBonded ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rxbond_ ; }
  
  void setTxBonded ( std::string s, bool v ) { std::lock_guard<std::mutex> lock(mtx_);  txbond_[ s ] = v; }
  std::map<std::string,bool> getTxBonded ( ) { std::lock_guard<std::mutex> lock(mtx_);  return txbond_ ; }

  void setRxHardRT ( std::string s, bool v ) { std::lock_guard<std::mutex> lock(mtx_);  rxhardrt_[ s ] = v; }
  std::map<std::string,bool> getRxHardRT ( ) { std::lock_guard<std::mutex> lock(mtx_);  return rxhardrt_; }
  
  void setTxHardRT ( std::string s, bool v ) { std::lock_guard<std::mutex> lock(mtx_);  txhardrt_[ s ] = v; }
  std::map<std::string,bool> getTxHardRT ( ) { std::lock_guard<std::mutex> lock(mtx_);  return txhardrt_; }

  void     incIOmap            (  )         { std::lock_guard<std::mutex> lock(mtx_);  IOmap_[0]++; }
  
};

static const size_t SDO_STACK_SIZE = 100*1024;
class SdoManager 
{
private:
  
  std::vector< std::tuple<std::string,std::string, bool > >   sdo_queries_;
  coe_master::NetworkDescriptorPtr                     network_;
  std::shared_ptr<ros::ServiceServer>                  set_sdo_;
  std::shared_ptr<ros::ServiceServer>                  get_sdo_;
  ros::CallbackQueue                                   sdo_queue_;
  ros::NodeHandle                                      nh_;
  bool                                                 force_sdo_;
                                                       
  bool syncSdo  ( const std::string&   desc
                , const std::string&   module_id
                , const uint8_t&       sdotype
                , const uint16_t&      index
                , const uint8_t&       subindex
                , const double&        timeout
                , const bool&          read_sdo
                , uint8_t*             value )
  {
    try 
    {
      
      int addr = network_->checkModuleName(module_id, true);
      if( addr < 0  )
      {
        
        ROS_ERROR("Module '%s' is not mapped", module_id.c_str() );
        ROS_INFO( "Mapped Modules are: ");
        for( auto const & n :  network_->getAddressUniqueIdMap() )
        {
          std::cout << n.first << "# '"<< n.second <<"'"<< std::endl;
        }
        return false;
      }
      
      coe_core::BaseDataObjectEntryPtr sdo; 
      int nominal_dim = -1;
      int dim = -1;
      
      switch(sdotype)
      {
        case coe_ros_msgs::SetSdo::Request::TYPE_U8 : dim = 1; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_U16: dim = 2; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_U32: dim = 4; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_U64: dim = 8; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_I8 : dim = 1; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_I16: dim = 2; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_I32: dim = 4; break;
        case coe_ros_msgs::SetSdo::Request::TYPE_I64: dim = 4; break;
        default:
        {
          ROS_ERROR("SDO Type not recognized");
          return false;
        }
      }
      nominal_dim = dim;
      
      int retval = -1;  
      assert( dim <= 8 );
      if( read_sdo )
      {
        uint8_t* buffer = new uint8_t[dim];
        std::memset( &buffer[0], 0x0, sizeof(uint8_t)*dim );

        retval = ec_SDOread   ( addr
                              , index
                              , (uint8)subindex
                              , FALSE
                              , &dim
                              , buffer
                              , EC_TIMEOUTSAFE  );
        
        std::memcpy( value, buffer, dim*sizeof( uint8_t ) );
      }
      else
      {
        uint8_t* buffer = new uint8_t[dim];
        std::memset( &buffer[0], 0x0, sizeof(uint8_t)*dim );
        std::memcpy( buffer, value, sizeof(uint8_t)*dim );
        
        size_t max_trial = 5;
        do
        {
          
          retval = ec_SDOwrite( addr
                              , index
                              , (uint8)subindex
                              , dim
                              , ( (uint8)subindex == 0 ) ? TRUE : FALSE
                              , buffer
                              , EC_TIMEOUTRXM );            
          if( retval > 0 )
            break;
          
          
        } while( max_trial-- );
      }
      
      if( retval <= 0 )
      {
        ROS_ERROR("ec_SDOread failed. See CoE diagnostics for further information.");
      }
            
      if( nominal_dim != dim )
      {
        throw std::runtime_error( ( "Read only " + std::to_string(nominal_dim) + " bytes, while " + std::to_string( dim ) + " bytes were supposed to be read").c_str());
      }
      return retval > 0;
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

  cnr_logger::TraceLoggerPtr logger_;
  
public:
  
  SdoManager(cnr_logger::TraceLoggerPtr logger, coe_master::NetworkDescriptorPtr network,
    bool force_sdo ) : logger_(logger), network_(network) , nh_("~" ), force_sdo_(force_sdo)
  {
    // set_sdo_.reset( new ros::ServiceServer( nh_.advertiseService( "coe/set_sdo", &SdoManager::setSdo, this ) ) );
    // get_sdo_.reset( new ros::ServiceServer( nh_.advertiseService( "coe/get_sdo", &SdoManager::getSdo, this ) ) );    
  }
  
  bool setSdo(coe_ros_msgs::SetSdo::Request& req, coe_ros_msgs::SetSdo::Response& res)
  {
    std::string s1,s2;
    s1 = std::string("SET SDO");
    s2 = coe_master::to_string( req.index, req.subindex, req.sdotype, req.value, req.timeout );
    
    res.success = syncSdo   ( req.desc
                            , req.module_id
                            , req.sdotype
                            , req.index
                            , req.subindex
                            , req.timeout
                            , false
                            , &(req.value[0]) );
    
    s2 +=  std::string(" [") + ( res.success ? "OK" : "FAILED" ) + std::string("]");
    
    sdo_queries_.push_back( std::make_tuple(s1,s2,bool(res.success)) );
    return true;
  }
  
  bool getSdo(coe_ros_msgs::GetSdo::Request& req, coe_ros_msgs::GetSdo::Response& res)
  {
    std::string s1,s2;
    s1 = std::string("GET SDO");
    s2 = req.module_id + ", " + coe_master::to_string( req.index, req.subindex, req.sdotype);
    
    res.success = syncSdo   ( req.desc
                            , req.module_id
                            , req.sdotype
                            , req.index
                            , req.subindex
                            , req.timeout
                            , true
                            , &(res.value[0]) );
    
    s2 += std::string(", value: ") + coe_master::to_string(req.sdotype, res.value) + ", timeout: " + std::to_string( req.timeout ) + std::string(" [") + (res.success ? "OK" : "FAILED" ) + std::string("]");

    sdo_queries_.push_back( std::make_tuple(s1,s2,bool(res.success)) );
    return true;
  }
  
  std::vector< std::tuple<std::string,std::string, bool > > getQueries( )
  {
    std::vector< std::tuple<std::string,std::string, bool > > ret = sdo_queries_;
    sdo_queries_.clear();
    return ret;
  }
  
  
  
  
};

  

inline bool initNodes(ros::NodeHandle& nh, const coe_master::NetworkDescriptorPtr network, std::vector< coe_master::ModuleDescriptorPtr >& nodes )
{
  try 
  {
    ROS_INFO("[%s%s%s] %sInit the nodes of the network", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW() );
    
    XmlRpc::XmlRpcValue param_root;      
    rosparam_utilities::extractParam( nh, network->getNamespace(), param_root );

    std::map<int, std::string> module_addresses_map = network->getAddressLabelsMap( );
    for( auto module_address : module_addresses_map )
    {
      bool default_config = network->hasDefaultConfiguration( module_address.first );
      std::string msg = std::string(RESET()) + "[-----]" 
                      + std::string("[ " + ( BOLDBLUE()   + std::string( "Init Params" ) + RESET() ) + " ] " )
                      + std::string("[ " + ( BOLDCYAN()   + std::to_string(module_address.first) +"# " + module_address.second + RESET() ) + " ] " )
                      + std::string("[ " + ( BOLDYELLOW() + std::string( default_config ? "DEFAULT" : " PARAMS" ) + RESET() ) + " ] " );

      ROS_DEBUG_STREAM( msg << " [ " << BOLDMAGENTA() << "RUNNING" << RESET() <<" ] " );
      coe_master::ModuleDescriptorPtr module( new coe_master::ModuleDescriptor( nh, network->getNamespace(), module_address.second, module_address.first, default_config ) );
      
      if( !module->initNodeConfigurationFromParams( ) )
      {
        ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Basic Configurations from params failed. ");
        return false;
      }

      
      if( network->hasDefaultConfiguration( module_address.first ) )
      {
        if( !module->initNodeCoeConfigurationFromSoem( module_address.first, module_address.second, true ) )
        {
          ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Configurations from SOEM failed. ");
          return false;
        }
      }
      else
      {
        if( !module->initNodeCoeConfigurationFromParams ( true ) ) //force sdo configuration from ROSPARAM
        {
          ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Configurations from Param Server failed. ");
          return false;
        }
      }
      
      auto jt = std::find_if( nodes.begin(), nodes.end(), [&module](const coe_master::ModuleDescriptorPtr& m ){ return m->getIdentifier() == module->getIdentifier(); } );
      if( jt != nodes.end() ) 
      {
        ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] There are two modules with the same 'identifier'. ");
        return false;
      }
      
      if( !module->initHandles( ) )
      {
        ROS_ERROR_STREAM( msg << " [ " << RED() << " ERROR " << RESET() << " ] Configurations from SOEM failed. ");
        return false;
      }

      // 
      nodes.push_back( module );
      
      ROS_DEBUG_STREAM( msg << " [ " << BOLDGREEN() << "OK" << RESET() <<" ]");
    }
    
    if( nodes.size() != nodes.size() )
    {
      ROS_ERROR_STREAM( " [ ERROR : Number of modules configured: " << nodes.size() << "/ Number of modules mapped in the rosparam server: " << module_addresses_map.size() << "]");
      return false;
    }
    
    ROS_INFO("[%s%s%s] %sInit the nodes of the network ", BOLDGREEN(), "  OK ", RESET(), BOLDYELLOW() );
  }
  catch(std::exception& e)
  {
    ROS_FATAL("--------------------------");
    ROS_FATAL("%s",e.what());
    ROS_FATAL("--------------------------");
    return false;
  }
  
  return true;
}

inline bool updateNodeConfiguration ( coe_master::ModuleDescriptorPtr& module, char* IOmap )
{
  bool ret = false;
  std::string HDR =std::string ( std::string(RESET()) + std::string("[-----]") 
              + "[ " 
              + ( BOLDCYAN() + std::to_string(module->getAddress()) +"# " + module->getIdentifier() + RESET() ) 
              + ( BOLDCYAN() + std::string( module->isDefaultConfig() ? " - DEFAULT CFG" : " - PARAMS CFG" ) + RESET() )
              + " ]" );

    
  if (ec_slave[module->getAddress()].mbx_proto & ECT_MBXPROT_COE)
  {
    ROS_DEBUG_STREAM ( HDR << " PDO Mapping through SDO [ " << BOLDMAGENTA() << "RUNNING" << RESET() << "] " );
    ret = coe_soem_utilities::get_pdo_map_through_sdo(module->getAddress(), module->getRxPdo(), module->getTxPdo(), IOmap );
    ROS_DEBUG_STREAM ( HDR << " PDO Mapping through SDO [ " << (ret ? ( BOLDGREEN()   +std::string("  OK   ") + RESET() ) : ( BOLDRED()   +std::string(" CHECK ") + RESET() ) ) << "] " );
  }
  else
  {
    ROS_DEBUG_STREAM ( HDR << " PDO Mapping through SII [ " << BOLDMAGENTA() << "RUNNING" << RESET() << "]" );
    ret = coe_soem_utilities::get_pdo_map_through_sii(module->getAddress(), module->getRxPdo(), module->getTxPdo(), IOmap );
    ROS_DEBUG_STREAM ( HDR << " PDO Mapping through SII [ " << (ret ? ( BOLDGREEN()   +std::string("  OK   ") + RESET() ) : ( BOLDRED()   +std::string(" CHECK ") + RESET() ) ) << "] " );
  }
  
  
  if( ret )
  {
    ROS_DEBUG_STREAM ( HDR << " PDO Mapping Finalization [ " << BOLDMAGENTA() << "RUNNING" << RESET() << "] " );
    module->getTxPdo().finalize();
    module->getRxPdo().finalize();
    ROS_DEBUG_STREAM ( HDR << " PDO Mapping Finalization [ " << BOLDGREEN() << " OK " << RESET() << "] " );
  }  
  
  ROS_DEBUG_STREAM ( HDR << " Handles Connection [ " << BOLDMAGENTA() << "RUNNING" << RESET() << "] " );
  if( !module->connectHandles() )
  {
    ROS_DEBUG_STREAM ( HDR << " Handles Connection [ " << RED() << "ERROR" << RESET() << "] " );
    return false;
  }
  ROS_DEBUG_STREAM ( HDR << " Handles Connection [ " << BOLDGREEN() << "OK" << RESET() << "] " );
  module->setConfiguratedSdo( ret ); 
  return ret;
}

inline bool updateNodes( std::vector< coe_master::ModuleDescriptorPtr >& nodes, char* IOmap, bool update_ros_param_server )
{
  try 
  {
    ROS_INFO("[%s%s%s] %sUpdate the nodes of the network", BOLDMAGENTA(), "START", RESET(), BOLDYELLOW() );
    
    for ( coe_master::ModuleDescriptorPtr module : nodes )
    {
      
      updateNodeConfiguration( module, IOmap );

      if( update_ros_param_server )
      {
        module->updateROSParamServer( );
      }
    }
    
    ROS_INFO("[%s%s%s] %sUpdate the nodes of the network", BOLDGREEN(), "  OK ", RESET(), BOLDYELLOW() );
  }
  catch(std::exception& e)
  {
    ROS_FATAL("--------------------------");
    ROS_FATAL("%s",e.what());
    ROS_FATAL("--------------------------");
    return false;
  }
  
  return true;
}




inline bool configDc  ( const std::vector< coe_master::ModuleDescriptorPtr >& nodes  )  
{ 
  auto it = std::find_if(nodes.begin(), nodes.end(), [](const coe_master::ModuleDescriptorPtr& m)
  {
    return m->isDcEnabled();  
  }); 
  return it != nodes.end(); 
}

inline std::map<int,bool> configSdoCa  ( const std::vector< coe_master::ModuleDescriptorPtr >& nodes  )  
{ 
  std::map<int,bool>  ret;
  for( auto const & module : nodes )
  {
    ret[ module->getAddress() ] = module->isSdoCaSupported();
  }

  return ret;
}

inline std::vector<std::string> getNodeUniqueIDs( const std::vector< coe_master::ModuleDescriptorPtr >& nodes ) 
{
  std::vector<std::string> ret(nodes.size());
  std::transform(nodes.begin(),nodes.end(),ret.begin(),[](coe_master::ModuleDescriptorPtr m) { return m->getIdentifier();} );
  return ret;
}


inline int checkModuleName ( const std::string& id, const std::vector< coe_master::ModuleDescriptorPtr >& nodes, const bool verbose )  
{ 
  int ret = -1;
  
  auto it = std::find_if(nodes.begin(), nodes.end(), [&id  ](const coe_master::ModuleDescriptorPtr& m) { return m->getIdentifier()==id; } );
  ret = ( it != nodes.end() ) ? (*it)->getAddress() : -1;
  
  
  if( (ret==-1) && verbose )
  {
    ROS_WARN("[checkModuleName] Requested: '%s', while the name available are:", id.c_str() );
    for( auto const & s : getNodeUniqueIDs(nodes) )
    {
      std::cout << "- " << s << std::endl;
    }
    
  }
  return ret; 
}






#endif
