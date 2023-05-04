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
  find_package(catkin REQUIRED COMPONENTS roscpp coe_core)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES coe_soem_utilities
    CATKIN_DEPENDS roscpp coe_core
  ) 
endif()

find_package(soem REQUIRED)
find_package(coe_core REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)


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
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${coe_core_INCLUDE_DIRS})
endif()

list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS} include)

# ================================================
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/coe_soem_utilities.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

if(USE_ROS1)
  target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES}
    soem ${catkin_LIBRARIES}
  )
else()
  target_link_libraries(${PROJECT_NAME} 
  INTERFACE ${coe_core_LIBRARIES} 
  PUBLIC soem 
  INTERFACE Boost::system 
  )
endif()
#################
## END - Build ##
#################


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME} 
        EXPORT export_coe_soem_utilities
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
  export(EXPORT export_coe_soem_utilities
    FILE "${PROJECT_BINARY_DIR}/export_coe_soem_utilities.cmake")

  # Create the coe_soem_utilitiesConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(coe_soem_utilitiesConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/coe_soem_utilitiesConfig.cmake" @ONLY)

  # Create the coe_soem_utilitiesConfigVersion.cmake
  configure_file(coe_soem_utilitiesConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/coe_soem_utilitiesConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/coe_soem_utilitiesConfig.cmake"
    "${PROJECT_BINARY_DIR}/coe_soem_utilitiesConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_coe_soem_utilities
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_coe_soem_utilities.cmake
    COMPONENT dev
  )
endif()


##########################
## Testing              ##
##########################
if(ENABLE_TESTING)
  include(GoogleTest)
#   add_executable(coe_soem_utilities_server_test 
#     src/coe_soem_utilities_server/args_parser.cpp
#     src/coe_soem_utilities_server/yaml_manager.cpp
#     test/test_server.cpp)
#   if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
#     target_link_libraries(coe_soem_utilities_server_test 
#       ${PROJECT_NAME} Threads::Threads GTest::gtest_main)
#   else()
#     target_link_libraries(coe_soem_utilities_server_test ${PROJECT_NAME} GTest::gtest_main)
#     if(THREADS_HAVE_PTHREAD_ARG)
#       target_compile_options(coe_soem_utilities_server_test PUBLIC "-pthread")
#     endif()
#     if(CMAKE_THREAD_LIBS_INIT)
#       target_link_libraries(coe_soem_utilities_server_test "${CMAKE_THREAD_LIBS_INIT}")
#     endif()
#   endif()
#   target_compile_definitions(coe_soem_utilities_server_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")  
#   gtest_discover_tests(coe_soem_utilities_server_test)
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
#       DEPENDENCIES coe_soem_utilities_server_test
#     )
#   else()
#     ## NOTHING TO DO, SINCE GTEST ALREADY CREATES THE COVERAGE
#     ## IF THE COMPILING OPTIONS ARE SET
#   endif()
#endif()



