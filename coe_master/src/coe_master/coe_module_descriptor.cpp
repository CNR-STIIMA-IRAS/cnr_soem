/**
 *
 * @file coe_utilities.h
 * @brief FIle with some utility for the management of the Can Over Ethercat protocol
 *
 */
#include <Eigen/Dense>
#include <regex>
#include <boost/interprocess/shared_memory_object.hpp>
#include <XmlRpc.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <coe_core/coe_base.h>
#include <coe_core/coe_pdo.h>
#include <coe_core/coe_sdo.h>
#include <coe_core/coe_sdo_xmlrpc.h>
#include <coe_core/coe_pdo_xmlrpc.h>
#include <coe_soem_utilities/coe_soem_utilities.h>
#include <coe_master/modules/coe_module_descriptor.h>
#include <coe_master/modules/coe_module_descriptor_xmlrpc.h>

#define HDR \
  std::string ( std::string(RESET()) + std::string("[-----]") \
              + "[ " \
              + ( BOLDCYAN() + std::to_string(address_) +"# " + unique_id_ + RESET() ) \
              + ( BOLDCYAN() + std::string( default_config_ ? " - DEFAULT CFG" : " - PARAMS CFG" ) + RESET() )\
              + " ]" )

namespace coe_master
{

bool ModuleDescriptor::initHandles(  )
{
  try
  {
    if( !cnr::param::has(root_param_, param_label_) )
    {
      printf("The module '%s' has not parameters in the ros parame server.", param_label_.c_str() );
      return false;
    }

    cnr::param::node_t node = cnr::param::extract(root, param_label_);
    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::AXIS_FEEDBACK ] )  )
    {
       coe_master::AxisData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::AXIS_FEEDBACK] ], axis_feedback_, "axis feedback" );
    }
    else ROS_DEBUG("[CHECK] None Axis Feedback Handle in rosparam server");
    
    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::AXIS_COMMAND ] )  )
    {
       coe_master::AxisData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::AXIS_COMMAND] ], axis_command_, "axis command" );
    }
    else ROS_DEBUG("[CHECK] None Axis Feedback Handle in rosparam server");

    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::ANALOG_INPUTS] )  )
    {
       coe_master::AnalogData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::ANALOG_INPUTS] ], analog_inputs_, "analog inputs" );
    }
    else ROS_DEBUG("[CHECK] None Analog Input Handle in rosparam server");
    
    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::ANALOG_OUTPUTS] )  )
    {
       coe_master::AnalogData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::ANALOG_OUTPUTS] ], analog_outputs_, "analog outputs" );
    }
    else ROS_DEBUG("[CHECK] None Analog Output Handle in rosparam server");
    
    
    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::DIGITAL_INPUTS] )  )
    {
       coe_master::DigitalData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::DIGITAL_INPUTS] ], digital_inputs_, "digital inputs" );
    }
    else ROS_DEBUG("[CHECK] None Digital Input Handle in rosparam server");
    
    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::DIGITAL_OUTPUTS] )  )
    {
       coe_master::DigitalData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::DIGITAL_OUTPUTS] ], digital_outputs_, "digital outputs" );
    }
    else ROS_DEBUG("[CHECK] None Digital Output Handle in rosparam server");

    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::WORD_INPUTS] )  )
    {
       coe_master::WordData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::WORD_INPUTS] ], word_inputs_, "word inputs" );
    }
    else ROS_DEBUG("[CHECK] None Digital Input Handle in rosparam server");
    
    if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::WORD_OUTPUTS] )  )
    {
       coe_master::WordData::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::WORD_OUTPUTS] ], word_outputs_, "word outputs" );
    }
    else ROS_DEBUG("[CHECK] None Digital Output Handle in rosparam server");
    
  }
  catch( XmlRpc::XmlRpcException& e )
  {
    printf("%s XmlRpcException: %s", HDR.c_str(), e.getMessage().c_str() );
    return false;
  }
  catch( std::exception& e )
  {
    printf("%s Exception: %s", HDR.c_str(), e.what() );
    return false;
  }
  
  return true;
}

bool ModuleDescriptor::initNodeConfigurationFromParams(const cnr::param::node_t &node) 
{
  if( configurated_names_ )
  {
    printf("The module labelled with '%s' has already the names configured. Check.", unique_id_.c_str() );
    return false;
  }
  
  cnr::param::node_t root_config();      
  cnr::param::extractParam( nh_, root_param_, root_config );

  try
  {
    
    cnr::param::node_t root( root_config );
    if( !root.hasMember( param_label_.c_str() ) )
    {
      printf("The module labelled with '%s/%s/%s' has not parameters in the ros parame server.", nh_.getNamespace().c_str(), root_param_.c_str(), param_label_.c_str() );
      return false;
    }
  }
  catch( XmlRpc::XmlRpcException& e )
  {
    printf("%s XmlRpcException: %s", HDR.c_str(), e.getMessage().c_str() );
    return false;
  }
  catch( std::exception& e )
  {
    printf("%s Exception: %s", HDR.c_str(),  e.what() );
    return false;
  }
  
  description_          = "TODO";
  model_                = "TODO";
  model_                = "TODO";
  loop_rate_decimation_ = 1;
  watchdog_decimation_  = 1;
  try
  {
    cnr::param::node_t root( root_config );
    cnr::param::node_t& node = root[param_label_.c_str()];

    description_          = rosparam_utilities::toString( node, ModuleData::KeysId[ ModuleData::KeysCode::DESCRIPTION           ] , "description"          ); // exception if error
    model_                = rosparam_utilities::toString( node, ModuleData::KeysId[ ModuleData::KeysCode::MODEL                 ] , "model"                ); // exception if error
    loop_rate_decimation_ = rosparam_utilities::toInt   ( node, ModuleData::KeysId[ ModuleData::KeysCode::LOOP_RATE_DECIMATION  ] , "loop rate decimation" ); // exception if error
    watchdog_decimation_  = rosparam_utilities::toDouble( node, ModuleData::KeysId[ ModuleData::KeysCode::WATCHDOG_DECIMATION   ] , "Wathcdog"             ); // exception if error
  }
  catch( std::exception& e )
  {
    ROS_WARN("%s %s", HDR.c_str(),  e.what() );
  }
  
  return true;

}

bool ModuleDescriptor::initNodeCoeConfigurationFromParams( bool configure_sdo ) 
{
  if( configurated_names_ )
  {
    printf("The module labelled with '%s' has already the names configured. Check.", unique_id_.c_str() );
    return false;
  }
  
  cnr::param::node_t root_config;      
  rosparam_utilities::extractParam( nh_, root_param_, root_config );
  
  try
  {
    ROS_DEBUG("[ %s ] Node Handle '%s', root param: '%s', configure_sdo: '%s' ", "ModuleDescriptor::initNodeCoeConfigurationFromParams", nh_.getNamespace().c_str(), root_param_.c_str(), (configure_sdo ? "TRUE" : "FALSE") );
    cnr::param::node_t root( root_config );
    if( !root.hasMember( param_label_.c_str() ) )
    {
      printf("The module labelled with '%s' has not parameters in the ros parame server.", param_label_.c_str() );
      return false;
    }
    
    cnr::param::node_t& node = root[param_label_.c_str()];

    ROS_DEBUG("[ %s ] Node Handle '%s', root param: '%s', param_label: '%s' ", "ModuleDescriptor::initNodeCoeConfigurationFromParams", nh_.getNamespace().c_str(), root_param_.c_str(), param_label_.c_str() );
    enable_dc_            = rosparam_utilities::toBool  ( node, ModuleData::KeysId[ ModuleData::KeysCode::ENABLE_DC             ] , "enable dc"            ); // exception if error
    support_sdoca_        = rosparam_utilities::toBool  ( node, ModuleData::KeysId[ ModuleData::KeysCode::SUPPORT_SDO_CA        ] , "support sdoca"        ); // exception if error
    
    if( !default_config_ )
    {
      if( configure_sdo  )
      {
        if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::SDO ] )  )
        {
          coe_core::XmlRpcSdo::set(node[ ModuleData::KeysId[ ModuleData::KeysCode::SDO ] ], configuration_sdo_, "sdo" );
        }
      }
      
      if(  cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::RXPDO ] )  )
      {
        coe_core::XmlRpcPdo::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::RXPDO ] ], rx_pdo_, "rxpdo" );
      }
      
      if(  cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::TXPDO ] )  )
      {
        coe_core::XmlRpcPdo::set( node[ ModuleData::KeysId[ ModuleData::KeysCode::TXPDO ] ], tx_pdo_, "txpdo" );
      }
      
      tx_pdo_.finalize();
      rx_pdo_.finalize();
      
      if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::TXPDO_SIZE ] )  )
      {
        int sz = rosparam_utilities::toInt ( node, ModuleData::KeysId[ ModuleData::KeysCode::TXPDO_SIZE ] , "txpdo packed size"       ); // exception if error
        tx_pdo_.setPackedBytesLenght( sz );
      }
      if( cnr::param::has(node, ModuleData::KeysId[ ModuleData::KeysCode::RXPDO_SIZE ] )  )
      {
        int sz = rosparam_utilities::toInt ( node, ModuleData::KeysId[ ModuleData::KeysCode::RXPDO_SIZE ] , "rxpdo packed size"       ); // exception if error
        rx_pdo_.setPackedBytesLenght( sz );
      }
    }

    configuration_sdo_.finalize();
    configurated_pdo_sdo_ = true;
  }
  catch( XmlRpc::XmlRpcException& e )
  {
    printf("%s XmlRpcException: %s", HDR.c_str(), e.getMessage().c_str() );
    return false;
  }
  catch( std::exception& e )
  {
    printf("%s Exception: %s", HDR.c_str(),  e.what() );
    return false;
  }
  
  return configurated_pdo_sdo_;

}
 
bool ModuleDescriptor::connectHandles (  )
{
  try
  {
    for( auto & e : axis_command_     ) e.second.entry  = rx_pdo_.subindex( e.second.pdo_subindex  - 1 );
    for( auto & e : axis_feedback_    ) e.second.entry  = tx_pdo_.subindex( e.second.pdo_subindex - 1 );
    
    for( auto & e : analog_outputs_   ) e.second.entry  = rx_pdo_.subindex( e.second.pdo_subindex - 1 );
    for( auto & e : analog_inputs_    ) e.second.entry  = tx_pdo_.subindex( e.second.pdo_subindex - 1 );
    
    for( auto & e : digital_outputs_  ) e.second.entry  = rx_pdo_.subindex( e.second.pdo_subindex - 1 );
    for( auto & e : digital_inputs_   ) e.second.entry  = tx_pdo_.subindex( e.second.pdo_subindex - 1 );

    for( auto & e : word_outputs_     ) e.second.entry  = rx_pdo_.subindex( e.second.pdo_subindex - 1 );
    for( auto & e : word_inputs_      ) e.second.entry  = tx_pdo_.subindex( e.second.pdo_subindex - 1 );

    for( auto & e : axis_command_     ) ROS_DEBUG(" Axis Command  Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for( auto & e : axis_feedback_    ) ROS_DEBUG(" Axis Feedback Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    
    for( auto & e : analog_outputs_   ) ROS_DEBUG(" Axis AO       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for( auto & e : analog_inputs_    ) ROS_DEBUG(" Axis AI       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    
    for( auto & e : digital_outputs_  ) ROS_DEBUG(" Axis DO       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for( auto & e : digital_inputs_   ) ROS_DEBUG(" Axis DI       Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    
    for( auto & e : word_outputs_     ) ROS_DEBUG(" Word Output   Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
    for( auto & e : word_inputs_      ) ROS_DEBUG(" Word Input    Handle '%s' -> '%s'", e.first.c_str(), e.second.entry->to_string().c_str());
      
    connected_ = true;    
  }
  catch( XmlRpc::XmlRpcException& e )
  {
    printf("%s XmlRpcException: %s", HDR.c_str(),  e.getMessage().c_str() );
    return false;
  }
  catch( std::exception& e )
  {
    printf("%s Exception: %s", HDR.c_str(), e.what() );
    return false;
  }

  return connected_;
}

bool ModuleDescriptor::initNodeCoeConfigurationFromSoem( const uint16_t iSlave, std::string slave_name, bool support_sdoca )
{

  std::string module_identifier;
  std::regex allowed_chars_in_name("^[A-Za-z]");

  module_identifier = slave_name;
  if(!std::regex_match(module_identifier,allowed_chars_in_name))
  {
    module_identifier = std::regex_replace(module_identifier, std::regex(R"([^A-Za-z\d])"), "");
  }
  std::string::iterator end_pos = std::remove(module_identifier.begin(), module_identifier.end(), ' ');
  module_identifier.erase(end_pos, module_identifier.end());
  
  module_identifier += "_" + std::to_string( iSlave );
  
  address_              = iSlave;
  model_                = module_identifier;
  support_sdoca_        = support_sdoca;
  enable_dc_           = false;
  configurated_pdo_sdo_ = false;
  
  return true;
}


bool ModuleDescriptor::updateROSParamServer ( )
{
  cnr::param::node_t node;
  cnr::param::node_t root( nh_.getNamespace()+"/"+root_param_ );
  if( !root.hasMember( param_label_ ) )
  {
    ModuleData::get( *this, node  );
  }
  else
  {
    node = root[ param_label_ ];
    ModuleData::get( *this, node  );
  }
  
  nh_.setParam( nh_.getNamespace()+"/"+root_param_+"/"+param_label_, node );
  
  return true;
}

const std::size_t   ModuleDescriptor::sizeInputs( ) const
{
  return tx_pdo_.nBytes(true);
}

const std::size_t   ModuleDescriptor::sizeOutputs( ) const
{
  return rx_pdo_.nBytes(true);
}

void ModuleDescriptor::updateInputs ( const uint8_t* inputs, bool prepended_time )
{
  if( tx_pdo_.nEntries( ) )
  {
    tx_pdo_.update( inputs, prepended_time );
  }
  
}

void ModuleDescriptor::updateOutputs( const uint8_t* outputs, bool prepended_time )
{
  if( rx_pdo_.nEntries( ) )
  {
    assert( outputs );
    rx_pdo_.update( outputs, prepended_time );
  }
}
  
const std::string ModuleDescriptor::to_string( const std::string what )
{
  std::stringstream ret; 
  ret << getIdentifier() << "\n";
  if( (what=="input") || (what=="tx_pdo") || (what=="txpdo") || (what=="all") || (what=="input-output") || (what=="both" ))
  {
    ret << tx_pdo_.to_string();
  }
  
  if( (what=="output") || (what=="rx_pdo") || (what=="rxpdo") || (what=="all") || (what=="input-output") || (what=="both" ))
  {
    ret << rx_pdo_.to_string();
  }
  return ret.str();
}

}

