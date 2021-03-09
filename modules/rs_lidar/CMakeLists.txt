cmake_minimum_required(VERSION 3.5)
include(CheckSymbolExists)
#========================
# debug
#========================
add_definitions(-DDEBUG_LEVEL=0)
#========================
# Project details / setup
#========================

set(PROJECT_NAME rs_lidar)
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
SET(CMAKE_INSTALL_PREFIX /usr/local/robosense_sdk)
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
FIND_PACKAGE( OpenMP REQUIRED) 
if(OPENMP_FOUND) 
message("OPENMP FOUND") 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 
set(CMAKE_SHARE_LINKER_FLAGS "${CMAKE_SHARE_LINKER_FLAGS} ${OpenMP_SHARE_LINKER_FLAGS}") 
endif()
#========================
# Build Setup
#========================
# rs_lidar
# main

add_library(rs_lidar SHARED
          src/input.cpp
          src/lidar_base.cpp
          )
add_dependencies(rs_lidar rs_yaml)

target_link_libraries(rs_lidar
                      rs_yaml
                      ${PCL_LIBRARIES}
                      pcap
                     )

target_include_directories(rs_lidar PUBLIC 
                      ${PROJECT_SOURCE_DIR}/include
                            ${PCL_INCLUDE_DIRS})

set_target_properties(rs_lidar PROPERTIES 
                     VERSION ${PROJECT_VERSION}
                     SOVERSION ${PROJECT_VERSION_MAJOR})


# #========================
# # Installation
# #========================        

# install(TARGETS rs_lidar
# ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
# LIBRARY DESTINATION ${INSTALL_LIB_DIR}
# RUNTIME DESTINATION ${INSTALL_BIN_DIR})

# install(DIRECTORY include/
#        DESTINATION ${INSTALL_INCLUDE_DIR}
#        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# install(DIRECTORY lib/
#        DESTINATION ${INSTALL_LIB_DIR}
#        FILES_MATCHING PATTERN "*.so")

# # Change the library location for an install location
# set(PACKAGE_LIBS ${INSTALL_LIB_DIR}/libpanta_lidar.so)

# #========================
# # Allow find_package()
# #======================== 
# # 1- local build #

# # Register the local build in case one doesn't use "make install"
# export(PACKAGE ${PROJECT_NAME})

# # Create variable for the local build tree
# get_property(CONF_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# # Configure config file for local build tree
# configure_file(
#   "${PROJECT_NAME}Config.cmake.in"
#   "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY
# )

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

# # Install Config.cmake
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
