cmake_minimum_required(VERSION 3.5)
project(panta_sdk)

#========================
# Subdirectory
#========================
add_subdirectory(modules/panta_common)
add_subdirectory(modules/rs_lidar)
add_subdirectory(modules/panta_sensor)
# add_subdirectory(modules/panta_preprocessing)

include(CheckSymbolExists)
#========================
# Project details / setup
#========================
set(PROJECT_NAME panta_sdk)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
#========================
# debug
#========================
add_definitions(-DDEBUG_LEVEL=0)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
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
  "${PROJECT_SOURCE_DIR}/cmake/version.h.in"
  "${PROJECT_SOURCE_DIR}/include/version.h"
)

#========================
# Installation Setup
#========================
SET(CMAKE_INSTALL_PREFIX /usr/local/panta_sdk)
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(INSTALL_CONFIG_DIR ${CMAKE_INSTALL_PREFIX}/config)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(INSTALL_PKGCONFIG_DIR ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

#========================
# Dependencies Setup
#========================
set(EXTERNAL_LIBS "")
find_package(tf QUIET)
find_package(cv_bridge QUIET)
find_package(roscpp 1.12 QUIET COMPONENTS)
if(roscpp_FOUND AND tf_FOUND)
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS_FOUND)
  include_directories(${roscpp_INCLUDE_DIRS})
  include_directories(${tf_INCLUDE_DIRS})
  include_directories(${cv_bridge_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${cv_bridge_LIBRARIES})
else(roscpp_FOUND AND tf_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND AND tf_FOUND)

find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  message(=============================================================)
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${PROTOBUF_LIBRARY})
else(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(Protobuf_FOUND)

#========================
# Build Setup
#========================
add_executable(panta_sdk_demo
              src/panta_sdk_demo_node.cpp
            src/panta_sdk_demo.cpp
            )
# add_dependencies(panta_sdk_demo panta_sensormanager panta_preprocessing)
add_dependencies(panta_sdk_demo panta_sensormanager)

target_link_libraries(panta_sdk_demo
                    panta_sensormanager
                    # panta_preprocessing       
                    ${roscpp_LIBRARIES}
                    ${PROTOBUF_LIBRARY}
                    ${tf_LIBRARIES}   
                    ${cv_bridge_LIBRARIES}        
                       )

target_include_directories(panta_sdk_demo PUBLIC ${PROJECT_SOURCE_DIR}/include)    
set_target_properties(panta_sdk_demo PROPERTIES 
            VERSION ${PROJECT_VERSION}
            SOVERSION ${PROJECT_VERSION_MAJOR}
                      )

#========================
# Installation
#========================        

# install(TARGETS panta_sdk_demo
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
# set(PACKAGE_LIBS "")

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
#   "cmake/${PROJECT_NAME}Config.cmake.in"
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
# )

# # The same versioning file can be used for both cases
# configure_file(
#   "cmake/${PROJECT_NAME}ConfigVersion.cmake.in"
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

# configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
# install(FILES
#   ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc
#   DESTINATION ${INSTALL_PKGCONFIG_DIR}
# )
# unset(LIBRARY_CC_ARGS)


