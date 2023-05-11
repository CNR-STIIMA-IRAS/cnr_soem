#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

cnr_set_flags()
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(USE_ROS1                 "ROS SUPPORT" OFF)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


#############################
## DEPENDANCIES            ##
#############################
find_package(soem REQUIRED)
if(USE_ROS1)
  find_package( catkin REQUIRED
                COMPONENTS  coe_soem_utilities
                            coe_core
                            rosparam_utilities
                            realtime_utilities
                            coe_ros_msgs
                            pluginlib )

  catkin_package( INCLUDE_DIRS include ${soem_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosparam_utilities realtime_utilities coe_core
  )          
else()
  find_package(coe_core REQUIRED)
  find_package(cnr_param REQUIRED)
  find_package(coe_soem_utilities REQUIRED)
  find_package(realtime_utilities REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(JSONCPP jsoncpp)
endif()
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system filesystem )

cnr_install_directories(USE_ROS1 
  CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
#############################
## END DEPENDANCIES        ##
#############################




#################
## Build       ##
#################
list(APPEND DEPENDENCIES_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
if(USE_ROS1)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
else()
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${coe_core_INCLUDE_DIRS}
    ${coe_soem_utilities_INCLUDE_DIRS} ${realtime_utilities_INCLUDE_DIRS} ${cnr_param_INCLUDE_DIRS})
endif()

list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS} include)

set( rt_LIBRARIES soem Boost::system pthread rt ${JSONCPP_LIBRARIES} ${BOOST_LIBRARIES})
#####################################################################
add_library(${PROJECT_NAME}_ipc
              src/${PROJECT_NAME}/ipc/sdo_ipc.cpp
)

target_include_directories(${PROJECT_NAME}_ipc PUBLIC
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

if(USE_ROS1)
  target_link_libraries(${PROJECT_NAME}_ipc ${catkin_LIBRARIES} ${rt_LIBRARIES}
  )
else()
  target_link_libraries(${PROJECT_NAME}_ipc cnr_param_utilities 
     ${coe_core_LIBRARIES} ${coe_soem_utilities_LIBRARIES}
      ${realtime_utilities_LIBRARIES}  ${rt_LIBRARIES} 
  )
endif()

add_executable(sdo_server_client_example examples/sdo_server_client_example.cpp)
target_link_libraries(sdo_server_client_example ${PROJECT_NAME}_ipc)
#####################################################################

# add_library(${PROJECT_NAME}
#               src/${PROJECT_NAME}/coe_master.cpp
#                   src/${PROJECT_NAME}/coe_module_descriptor.cpp
#                     src/${PROJECT_NAME}/coe_network_descriptor.cpp
#                       src/${PROJECT_NAME}/ipc/coe_pdo_ipc.cpp
#                         src/${PROJECT_NAME}/ipc/coe_ipc.cpp
#                           src/${PROJECT_NAME}/po2so_callback/po2so_callbacks.cpp
# )


#################
## END - Build ##
#################


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}_ipc
        DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}_ipc
        EXPORT export_coe_master
        ARCHIVE DESTINATION ${CNR_INSTALL_LIB_DIR}
        LIBRARY DESTINATION ${CNR_INSTALL_LIB_DIR}
        RUNTIME DESTINATION ${CNR_INSTALL_BIN_DIR}
)
###################
## END - Install ##
###################

##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################
if(NOT USE_ROS1) 
  set(export_targets ${export_targets};${PROJECT_NAME}_ipc)
  export(EXPORT export_coe_master
    FILE "${PROJECT_BINARY_DIR}/export_coe_master.cmake")

  # Create the coe_masterConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(coe_masterConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/coe_masterConfig.cmake" @ONLY)

  # Create the coe_masterConfigVersion.cmake
  configure_file(coe_masterConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/coe_masterConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/coe_masterConfig.cmake"
    "${PROJECT_BINARY_DIR}/coe_masterConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_coe_master
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_coe_master.cmake
    COMPONENT dev
  )
endif()


##########################
## Testing              ##
##########################
if(ENABLE_TESTING)
  include(GoogleTest)
  add_executable(coe_master_ipc_test 
     test/ipc/test_ipc.cpp
  )
  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
    target_link_libraries(coe_master_ipc_test 
      ${PROJECT_NAME}_ipc Threads::Threads GTest::gtest_main)
  else()
    target_link_libraries(coe_master_ipc_test ${PROJECT_NAME} GTest::gtest_main)
    if(THREADS_HAVE_PTHREAD_ARG)
      target_compile_options(coe_master_ipc_test PUBLIC "-pthread")
    endif()
    if(CMAKE_THREAD_LIBS_INIT)
      target_link_libraries(coe_master_ipc_test "${CMAKE_THREAD_LIBS_INIT}")
    endif()
  endif()
  target_compile_definitions(coe_master_ipc_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")  
  gtest_discover_tests(coe_master_ipc_test)
 endif(ENABLE_TESTING)


##########################
## Coverage Testing     ##
##########################
# if( ENABLE_TESTING AND ENABLE_COVERAGE_TESTING AND NOT WIN32 )
#   if(USE_ROS)
#     message(STATUS "The coverage testing is enabled")
#     set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
#     add_code_coverage(
#       NAME coverage_report
#       DEPENDENCIES coe_master_server_test
#     )
#   else()
#     ## NOTHING TO DO, SINCE GTEST ALREADY CREATES THE COVERAGE
#     ## IF THE COMPILING OPTIONS ARE SET
#   endif()
#endif()



