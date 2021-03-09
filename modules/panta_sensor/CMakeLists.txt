cmake_minimum_required(VERSION 3.5)

include(CheckSymbolExists)

#========================
# debug
#========================
add_definitions(-DDEBUG_LEVEL=0)
#========================
# Project details / setup
#========================
set(PROJECT_NAME panta_sensor)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# The version number.
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 2)
set (PROJECT_VERSION_PATCH 0)
set (PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

cmake_policy(SET CMP0048 NEW)
project(${PROJECT_NAME} VERSION ${PROJECT_VERSION})

set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/version.h.in"
  "${PROJECT_BINARY_DIR}/include/version.h"
)

#========================
# Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/local/panta_sdk)
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(INSTALL_PKGCONFIG_DIR ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)


#========================
# Dependencies Setup
#========================
set(EXTERNAL_LIBS "")

find_package(PCL REQUIRED common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#========================
# Ros part
#========================
find_package(roscpp 1.12 QUIET COMPONENTS
            geometry_msgs
            sensor_msgs
            pcl_conversions
            pcl_ros
            std_msgs
            tf
            )

if(roscpp_FOUND)
  message("-- ROS Found, Ros Support is turned On!")
  add_definitions(-DROS_FOUND)
  include_directories(${roscpp_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${roscpp_LIBRARIES})
else(roscpp_FOUND)
  message("-- ROS Not Found, Ros Support is turned Off!")
endif(roscpp_FOUND)
#========================
# Proto part
#========================
find_package(Protobuf QUIET)
if(Protobuf_FOUND )
  message("-- Protobuf Found, Protobuf Support is turned On!")
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${PROTOBUF_LIBRARY})
else(Protobuf_FOUND)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
endif(Protobuf_FOUND)

#========================
# Build Setup
#========================
# panta_sensor
add_library(rs_serial SHARED
           src/driver/serial/rs_serial.cpp
)
target_link_libraries(rs_serial rs_yaml)
target_include_directories(rs_serial PUBLIC ${PROJECT_SOURCE_DIR}/include)    
set_target_properties(rs_serial PROPERTIES 
            VERSION ${PROJECT_VERSION}
            SOVERSION ${PROJECT_VERSION_MAJOR}
)
#==============================
#imu
add_library(rs_imu SHARED
          src/driver/imu/imu_base.cpp
          src/driver/imu/imu_TL740D.cpp
          src/driver/imu/imu_HWT605.cpp
          src/driver/imu/imu_DY551.cpp
           )
add_dependencies(rs_imu rs_serial rs_yaml)
target_link_libraries(rs_imu 
                      rs_serial
                      rs_yaml
                      
                     )     
target_include_directories(rs_imu PUBLIC ${PROJECT_SOURCE_DIR}/include)    
set_target_properties(rs_imu PROPERTIES 
                     VERSION ${PROJECT_VERSION}
                     SOVERSION ${PROJECT_VERSION_MAJOR})
#==============================
#gnss
add_library(rs_gnss SHARED
            src/driver/gnss/gnss_base.cpp
            src/driver/gnss/gnss_BT708.cpp
            src/driver/gnss/gnss_POSLVX.cpp
          )

add_dependencies(rs_gnss rs_serial rs_yaml )
target_link_libraries(rs_gnss
                      rs_serial
                      pthread
                      rs_yaml
                       )    
target_include_directories(rs_gnss PUBLIC ${PROJECT_SOURCE_DIR}/include)    
set_target_properties(rs_gnss PROPERTIES 
                      VERSION ${PROJECT_VERSION}
                      SOVERSION ${PROJECT_VERSION_MAJOR})
#=============================
#ins
add_library(rs_ins SHARED
            src/driver/ins/ins_XWG13668.cpp
             )
add_dependencies(rs_ins rs_serial rs_gnss rs_yaml )
target_link_libraries(rs_ins
                      rs_gnss   
                      rs_serial
                      rs_yaml
                       )     
target_include_directories(rs_ins PUBLIC ${PROJECT_SOURCE_DIR}/include)
          
set_target_properties(rs_ins PROPERTIES 
                      VERSION ${PROJECT_VERSION}
                      SOVERSION ${PROJECT_VERSION_MAJOR})
#=============================
#odom
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("-- System processor is x86, will use canbus lib")
  add_definitions(-DX86_64)
  add_library(rs_odom SHARED
              src/driver/odom/odom_byd.cpp
              src/driver/odom/odom_bieke.cpp
              src/driver/odom/odom_base.cpp
             src/driver/odom/odom_jili.cpp
              src/driver/odom/can_bridge.cpp
              )
  add_dependencies(rs_odom rs_yaml)
  target_link_libraries(rs_odom
                        rs_yaml
                        pthread  
                        ${PROJECT_SOURCE_DIR}/lib/libcontrolcan_x86_64.so
                        )     
  target_include_directories(rs_odom PUBLIC ${PROJECT_SOURCE_DIR}/include)
  set_target_properties(rs_odom PROPERTIES 
                       VERSION ${PROJECT_VERSION}
                       SOVERSION ${PROJECT_VERSION_MAJOR})
else()
  message("-- System processor is not x86, will not use canbus lib!")
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#=============================
#sensor manager
add_library(panta_sensormanager SHARED
              src/sensor_manager/sensor_manager.cpp
              src/ros/lidar_points_ros_adapter.cpp
              src/ros/lidar_packtes_ros_adapter.cpp
              src/ros/imu_ros_adapter.cpp
              src/ros/gnss_ros_adapter.cpp
              src/ros/odom_ros_adapter.cpp
              src/proto/imu/imu_proto_adapter.cpp
              src/proto/gnss/gnss_proto_adapter.cpp
              src/proto/odom/odom_proto_adapter.cpp  
              src/proto/lidar/lidar_points_proto_adapter.cpp
              src/proto/lidar/lidar_packets_proto_adapter.cpp
)
add_dependencies(panta_sensormanager rs_ins rs_imu rs_gnss rs_yaml rs_lidar)
if(Protobuf_FOUND)
add_dependencies(panta_sensormanager proto_msg)
endif(Protobuf_FOUND)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
add_dependencies(panta_sensormanager rs_odom)
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
target_link_libraries(panta_sensormanager
                      rs_imu
                      rs_gnss  
                      rs_ins
                      rs_yaml
                      rs_lidar
                      ${roscpp_LIBRARIES}
                      ${PROTOBUF_LIBRARY}
                      pthread
                      )
if(Protobuf_FOUND)
target_link_libraries(panta_sensormanager proto_msg)
endif(Protobuf_FOUND)
target_include_directories(panta_sensormanager PUBLIC ${PROJECT_SOURCE_DIR}/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")   
target_link_libraries(panta_sensormanager rs_odom)   
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")  
set_target_properties(panta_sensormanager PROPERTIES 
VERSION ${PROJECT_VERSION}
SOVERSION ${PROJECT_VERSION_MAJOR}) 
                
                                         
#========================
# Installation
#========================    
# install(TARGETS rs_serial
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})    
# install(TARGETS rs_imu
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})
# install(TARGETS rs_gnss
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")  
# install(TARGETS rs_odom
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})
# endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")  
# install(TARGETS rs_ins
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})
# install(TARGETS panta_sensormanager
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})

# install(DIRECTORY include/
#        DESTINATION ${INSTALL_INCLUDE_DIR}
#        FILES_MATCHING PATTERN "*.h")

# install(DIRECTORY lib/
#        DESTINATION ${INSTALL_LIB_DIR}
#        FILES_MATCHING PATTERN "*.so")
       
# # Change the library location for an install location
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
# set(PACKAGE_LIBS ${INSTALL_LIB_DIR}/libpanta_sensormanager.so
#                  ${INSTALL_LIB_DIR}/librs_imu.so
#                  ${INSTALL_LIB_DIR}/librs_gnss.so
#                  ${INSTALL_LIB_DIR}/librs_ins.so
#                  ${INSTALL_LIB_DIR}/librs_serial.so
#                  ${INSTALL_LIB_DIR}/librs_odom.so)
# else()
# set(PACKAGE_LIBS ${INSTALL_LIB_DIR}/libpanta_sensormanager.so
#                  ${INSTALL_LIB_DIR}/librs_imu.so
#                  ${INSTALL_LIB_DIR}/librs_gnss.so
#                  ${INSTALL_LIB_DIR}/librs_ins.so
#                  ${INSTALL_LIB_DIR}/librs_serial.so)
# endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")  

# #========================
# # Allow find_package()
# #======================== 
# # 1- local build #

# # Register the local build in case one doesn't use "make install"
# export(PACKAGE ${PROJECT_NAME})

# # Create variable for the local build tree
# get_property(CONF_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# # Configure config file for local build tree
# # configure_file(
# #   "${PROJECT_NAME}Config.cmake.in"
# #   "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY
# # )

# # 2- installation build #

# # Change the include location for the case of an install location
# set(CONF_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${CONF_INCLUDE_DIRS} )

# #FIXME: this will only be applied to installed files. Confirm that we want that.
# # gather all the includes but remove ones in the source tree
# # because we added an include for the local yaml-cpp-pm we should also remove it
# list(REMOVE_ITEM CONF_INCLUDE_DIRS ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include)

# # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
# configure_file(
#   "${PROJECT_NAME}Config.cmake.in"
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
# )

# # The same versioning file can be used for both cases
# configure_file(
#   "${PROJECT_NAME}ConfigVersion.cmake.in"
#   "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY
# )


# # Install the libpointmatcherConfig.cmake and libpointmatcherConfigVersion.cmake
# install(
#   FILES
#     "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
#     "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION 
#     "${INSTALL_CMAKE_DIR}/${PROJECT_NAME}"
# )

# #Handle pkg-config file
# set(LIBRARY_CC_ARGS "")
# foreach(_LIB IN LISTS PACKAGE_LIBS EXTERNAL_LIBS) 
#   get_filename_component(_FILE_NAME ${_LIB} NAME)
#   if(${_FILE_NAME} STREQUAL ${_LIB}) # not an absolute path
#     set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} -l${_LIB}")
#   else()
#     set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} ${_LIB}")
#   endif()
# endforeach()
# unset(_LIB)
# unset(_FILE_NAME)

# configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
# install(FILES
#   ${CMAKE_BINARY_DIR}/modules/${PROJECT_NAME}/${PROJECT_NAME}.pc
#   DESTINATION ${INSTALL_PKGCONFIG_DIR}
# )
# unset(LIBRARY_CC_ARGS)

# #========================
# # Uninstall
# #======================== 
# # uninstall target
# if(NOT TARGET uninstall)
#    configure_file(
#   "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
#   "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" @ONLY
# )
#     add_custom_target(uninstall
#         COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
# endif()