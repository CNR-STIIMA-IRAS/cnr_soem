#ifndef __COE__BASE_PLUGINS_ROS__H___
#define __COE__BASE_PLUGINS_ROS__H___

#include <functional>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

#include <rosparam_utilities/rosparam_utilities.h>
#include <realtime_utilities/realtime_utilities.h>
#include <coe_core/ds402/coe_xfsm_utilities.h>
#include <coe_ros_msgs/GetSdo.h>
#include <coe_ros_msgs/SetSdo.h>
#include <coe_master/ipc/coe_pdo_ipc.h>
#include <coe_master/modules/coe_module_descriptor_xmlrpc.h>
#include <coe_master/modules/coe_network_descriptor_xmlrpc.h>

namespace coe_master
{

  static const char* SET_SDO_SERVER_NAMESPACE = "set_sdo";
  static const char* GET_SDO_SERVER_NAMESPACE = "get_sdo";

  inline void toBoostArray( const coe_core::BaseDataObjectEntry* in, boost::array<uint8_t, 8>& data  )
  {
    uint8_t* raw_data = new uint8_t[ in->sizeBytes() ];
    std::memset( &raw_data[0], 0x0, in->sizeBytes() ) ;
    std::memcpy( &raw_data[0], in->data(), in->sizeBytes() ) ;
    std::memcpy( &data[0], &raw_data[0], in->sizeBytes() ) ;
    delete raw_data;
  }

  inline void fromBoostArray( const boost::array<uint8_t, 8>& data, coe_core::BaseDataObjectEntry* in  )
  {
    uint8_t* raw_data = new uint8_t[ in->sizeBytes() ];
    std::memset( &raw_data[0] , 0x0         , in->sizeBytes() ) ;
    std::memcpy( &raw_data[0] , &data[0]    , in->sizeBytes() ) ;
    std::memcpy( in->data()   , &raw_data[0], in->sizeBytes() ) ;
    delete raw_data;
  }

  class CoeHwPlugin
  {

  public:

    enum Error { NONE_ERROR, COM_ERROR, DEVICE_ERROR, EXCEPTION_ERROR };
    enum OperationModeState { OPERATION_MODE_IDLE, OPERATION_MODE_RUNNING, OPERATION_MODE_ERROR, OPERATION_MODE_GOAL_ACHIEVED };


  private:

    bool                                initialized_;
    struct timespec                     read_time_;
    struct timespec                     write_time_;
    // Local pointer to easy manage the access to the shared memory
    uint8_t*                            prxpdo_;
    uint8_t*                            prxpdo_previous_;

    uint8_t*                            ptxpdo_;
    uint8_t*                            ptxpdo_previous_;

    // Structure to manage the protected access to the shared memory
    coe_master::ModuleIPCPtr            pdo_shared_memory_;

    // Methods to access the node through sdo
    ros::ServiceClient                  set_sdo_;
    ros::ServiceClient                  get_sdo_;

    //
    bool                                stop_thread_;

    // Error management
    std::mutex                                        error_mtx_;
    std::vector< std::pair<std::string,std::string> > errors_active_;
    bool                                              are_errors_active_;
    boost::shared_ptr<boost::thread>                  error_thread_;
    boost::shared_ptr<diagnostic_updater::Updater>    updater_;
    void errorThread( );
    void errorDiagnostic( diagnostic_updater::DiagnosticStatusWrapper &stat );

  protected:

    ros::NodeHandle                     nh_;
    std::shared_ptr< ros::NodeHandle >  plugin_private_nh_;
    double                              operational_time_;
    coe_master::ModuleDescriptorPtr     module_;

    virtual bool checkErrorsActive ( ) = 0;

    OperationModeState operation_mode_state_;


  public:

    virtual void getDeviceErrors ( std::vector< std::pair< std::string,std::string> >& errors_map ) = 0;

    CoeHwPlugin ( );
    virtual ~CoeHwPlugin ( );

    virtual bool initialize(ros::NodeHandle& nh, const std::string& device_coe_parameter, int address );
    virtual bool reset( );
    virtual bool setHardRT ( );
    virtual bool setSoftRT ( );
    virtual Error read  ( );
    virtual Error write ( );
    virtual Error safeWrite ( );

    virtual bool readSdo  ( coe_core::BaseDataObjectEntry* in);
    virtual bool writeSdo ( const coe_core::BaseDataObjectEntry* in);

    std::string  getUniqueId       ( ) const  { return module_->getIdentifier(); }
    bool         areErrorsActive   ( ) const  { return are_errors_active_; }

    virtual std::vector<std::string> getStateNames     ( ) const                         = 0;
    virtual std::string              getActualState    ( )                               = 0;
    virtual bool                     setTargetState    ( const std::string& state_name, const bool async ) = 0;

    virtual std::vector<std::string> getBytesInputNames   ( ) const = 0;
    virtual std::vector<std::string> getBytesOutputNames  ( ) const = 0;
    virtual std::vector<std::string> getAnalogInputNames  ( ) const = 0;
    virtual std::vector<std::string> getAnalogOutputNames ( ) const = 0;
    virtual std::vector<std::string> getDigitalInputNames ( ) const = 0;
    virtual std::vector<std::string> getDigitalOutputNames( ) const = 0;

    virtual void      jointStateHandle        ( double** pos,  double** vel, double** eff ) = 0;
    virtual void      jointCommandHandle      ( double** pos,  double** vel, double** eff )  = 0;

    virtual uint64_t* bytesInputValueHandle   ( const std::string& name ) = 0;
    virtual uint64_t* bytesOutputValueHandle  ( const std::string& name ) = 0;

    virtual double*   analogInputValueHandle  ( const std::string& name ) = 0;
    virtual double*   analogOutputValueHandle ( const std::string& name ) = 0;

    virtual bool*     digitalInputValueHandle ( const std::string& name ) = 0;
    virtual bool*     digitalOutputValueHandle( const std::string& name ) = 0;

    virtual bool hasBytesInputs   ( ) = 0;
    virtual bool hasBytesOutputs  ( ) = 0;
    virtual bool hasAnalogInputs  ( ) = 0;
    virtual bool hasAnalogOutputs ( ) = 0;
    virtual bool hasDigitalInputs ( ) = 0;
    virtual bool hasDigitalOutputs( ) = 0;
    virtual bool isActuator       ( ) = 0;

    virtual void getJointState( std::vector<double>&/*pos*/,std::vector<double>&/*vel*/,std::vector<double>&/*eff*/){}
    virtual void getJointCommand( std::vector<double>&/*pos*/,std::vector<double>&/*vel*/,std::vector<double>&/*eff*/){}


  };


}

#include <coe_master/hw_plugin/coe_hw_base_plugin_impl.hpp>



#endif
