cmake_minimum_required(VERSION 3.5)
include(CheckSymbolExists)
#========================
# Project details / setup~
#========================
set(PROJECT_NAME panta_common)
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
# Complie Proto
#========================
find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/include/panta_common/msg/proto_msg)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.Imu.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.Gnss.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.Odom.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.LidarPoints.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.LidarPacket.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.LidarScan.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.VehicleState.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.Percept.proto)
  execute_process(COMMAND protoc --proto_path=${PROTO_FILE_PATH} --cpp_out=${PROTO_FILE_PATH} ${PROTO_FILE_PATH}/Proto_msg.GridMap.proto)
else(Protobuf_FOUND)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
endif(Protobuf_FOUND)

#========================
# Dependencies Setup
#========================
set(EXTERNAL_LIBS "")
find_package(yaml-cpp REQUIRED)
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${YAML_CPP_LIBRARIES})

#========================
# Build Setup
#========================

add_library(encryptor SHARED IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set_target_properties(encryptor PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libencryptor_x86_64.so)
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set_target_properties(encryptor PROPERTIES
    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libencryptor_aarch64.so)
endif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")  
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} encryptor)

add_library(rs_yaml SHARED
            src/yaml/yaml_parser.cpp
            )
target_link_libraries(rs_yaml
                      encryptor
                      stdc++fs
                    ${YAML_CPP_LIBRARIES}
                      )  
target_include_directories(rs_yaml PUBLIC
                  ${PROJECT_SOURCE_DIR}/include
                      ${YAML_CPP_INCLUDE_DIR}
                          )
set_target_properties(rs_yaml PROPERTIES 
                 VERSION ${PROJECT_VERSION}
                 SOVERSION ${PROJECT_VERSION_MAJOR}
                      )
if(Protobuf_FOUND)
add_library(proto_msg SHARED
            ${PROTO_FILE_PATH}/Proto_msg.Imu.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.Gnss.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.Odom.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarPoints.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarScan.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarPacket.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.VehicleState.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.Percept.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.GridMap.pb.cc
            )
endif(Protobuf_FOUND)

#========================
# Installation
#========================        
install(TARGETS rs_yaml
ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
LIBRARY DESTINATION ${INSTALL_LIB_DIR}
RUNTIME DESTINATION ${INSTALL_BIN_DIR})
if(Protobuf_FOUND)
install(TARGETS proto_msg
ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
LIBRARY DESTINATION ${INSTALL_LIB_DIR}
RUNTIME DESTINATION ${INSTALL_BIN_DIR})
endif(Protobuf_FOUND)
install(DIRECTORY include/
       DESTINATION ${INSTALL_INCLUDE_DIR}
       FILES_MATCHING PATTERN "*.h")

install(DIRECTORY include/
        DESTINATION ${INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY lib/
       DESTINATION ${INSTALL_LIB_DIR}
       FILES_MATCHING PATTERN "*.so")

# Change the library location for an install location
set(PACKAGE_LIBS ${INSTALL_LIB_DIR}/librs_yaml.so)
list(APPEND PACKAGE_LIBS ${INSTALL_LIB_DIR}/libproto_msg.so)

#========================
# Allow find_package()
#======================== 
# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE ${PROJECT_NAME})

# Create variable for the local build tree
include_directories(${CMAKE_SOURCE_DIR}/include)
get_property(CONF_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Configure config file for local build tree
configure_file(${PROJECT_NAME}Config.cmake.in
"${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)

# 2- installation build #

# Change the include location for the case of an install location
set(CONF_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${CONF_INCLUDE_DIRS} )

#FIXME: this will only be applied to installed files. Confirm that we want that.
# gather all the includes but remove ones in the source tree
# because we added an include for the local yaml-cpp-pm we should also remove it
# list(REMOVE_ITEM CONF_INCLUDE_DIRS ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/include)

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(
  "${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
)

# The same versioning file can be used for both cases
configure_file(
  "${PROJECT_NAME}ConfigVersion.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY
)

# Install Config.cmake
install(
  FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION 
    "${INSTALL_CMAKE_DIR}/${PROJECT_NAME}"
)

#Handle pkg-config file
set(LIBRARY_CC_ARGS "")
foreach(_LIB IN LISTS PACKAGE_LIBS EXTERNAL_LIBS) 
  get_filename_component(_FILE_NAME ${_LIB} NAME)
  if(${_FILE_NAME} STREQUAL ${_LIB}) # not an absolute path
    set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} -l${_LIB}")
  else()
    set(LIBRARY_CC_ARGS "${LIBRARY_CC_ARGS} ${_LIB}")
  endif()
endforeach()
unset(_LIB)
unset(_FILE_NAME)

# configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
# install(FILES
#   ${CMAKE_BINARY_DIR}/modules/${PROJECT_NAME}/${PROJECT_NAME}.pc
#   DESTINATION ${INSTALL_PKGCONFIG_DIR}
# )
# unset(LIBRARY_CC_ARGS)

#========================
# Uninstall
#======================== 
# uninstall target
if(NOT TARGET uninstall)
   configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" @ONLY
)
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
