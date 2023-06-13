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

#############################
## DEPENDANCIES            ##
#############################
if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS roscpp
  ) 
endif()

find_package(soem REQUIRED)

cnr_install_directories(USE_ROS1 
  CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
#############################
## END DEPENDANCIES        ##
#############################


#################
## Build       ##
#################
if(USE_ROS1)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
endif()

list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS} include)

# ================================================
add_library(${PROJECT_NAME}
              src/${PROJECT_NAME}/ds301/coe_bitwise_struct.cpp
                src/${PROJECT_NAME}/ds402/coe_xfsm_utilities.cpp      
                  src/${PROJECT_NAME}/ds402/coe_bitwise_struct.cpp  
                    src/${PROJECT_NAME}/coe_base.cpp
                      src/${PROJECT_NAME}/coe_string_utilities.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

target_link_libraries(${PROJECT_NAME} PUBLIC soem)
#################
## END - Build ##
#################

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
        FILES_MATCHING 
          PATTERN "*.h"
          PATTERN "*.hpp"
          PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME} 
        EXPORT export_coe_core
        ARCHIVE DESTINATION ${CNR_INSTALL_LIB_DIR}
        LIBRARY DESTINATION ${CNR_INSTALL_LIB_DIR}
        RUNTIME DESTINATION ${CNR_INSTALL_BIN_DIR}
)

if(USE_ROS1)
  install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
endif()
###################
## END - Install ##
###################

##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################
if(NOT USE_ROS1) 
  set(export_targets ${export_targets};${PROJECT_NAME})
  export(EXPORT export_coe_core
    FILE "${PROJECT_BINARY_DIR}/export_coe_core.cmake")

  # Create the coe_coreConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(coe_coreConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/coe_coreConfig.cmake" @ONLY)

  # Create the coe_coreConfigVersion.cmake
  configure_file(coe_coreConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/coe_coreConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/coe_coreConfig.cmake"
    "${PROJECT_BINARY_DIR}/coe_coreConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_coe_core
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_coe_core.cmake
    COMPONENT dev
  )
endif()


##########################
## Testing              ##
##########################
if(ENABLE_TESTING)
  include(GoogleTest)
#   add_executable(coe_core_server_test 
#     src/coe_core_server/args_parser.cpp
#     src/coe_core_server/yaml_manager.cpp
#     test/test_server.cpp)
#   if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
#     target_link_libraries(coe_core_server_test 
#       ${PROJECT_NAME} Threads::Threads GTest::gtest_main)
#   else()
#     target_link_libraries(coe_core_server_test ${PROJECT_NAME} GTest::gtest_main)
#     if(THREADS_HAVE_PTHREAD_ARG)
#       target_compile_options(coe_core_server_test PUBLIC "-pthread")
#     endif()
#     if(CMAKE_THREAD_LIBS_INIT)
#       target_link_libraries(coe_core_server_test "${CMAKE_THREAD_LIBS_INIT}")
#     endif()
#   endif()
#   target_compile_definitions(coe_core_server_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")  
#   gtest_discover_tests(coe_core_server_test)
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
#       DEPENDENCIES coe_core_server_test
#     )
#   else()
#     ## NOTHING TO DO, SINCE GTEST ALREADY CREATES THE COVERAGE
#     ## IF THE COMPILING OPTIONS ARE SET
#   endif()
#endif()



