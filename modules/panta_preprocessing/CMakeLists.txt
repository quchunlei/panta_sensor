cmake_minimum_required(VERSION 3.5)
add_compile_options(-std=c++11 -fopenmp)
add_compile_options(-W)
set(CMAKE_BUILD_TYPE Release)
set(PROJECT_NAME panta_preprocessing)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

project(panta_preprocessing)
set(panta_preprocessing_VERSION_MAJOR 1)
set(panta_preprocessing_VERSION_MINOR 3)
set(panta_preprocessing_VERSION_PATCH 0)
set(panta_preprocessing_VERSION ${panta_preprocessing_VERSION_MAJOR}.${panta_preprocessing_VERSION_MINOR}.${panta_preprocessing_VERSION_PATCH})

configure_file("${PROJECT_SOURCE_DIR}/include/version.h.in"
               "${PROJECT_BINARY_DIR}/version.h")
include_directories("${PROJECT_BINARY_DIR}" include ${CMAKE_CURRENT_SOURCE_DIR} )

execute_process(COMMAND lsb_release -rs
                OUTPUT_VARIABLE RELEASE_VERSIONNAME
                OUTPUT_STRIP_TRAILING_WHITESPACE)
message("THE RELEASE_VERSIONNAME  " ${RELEASE_VERSIONNAME})


if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 
add_library(panta_preprocessing SHARED IMPORTED GLOBAL) 
set_target_properties(panta_preprocessing PROPERTIES 
                    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/x86_64/librs_preprocessing.so) 
set_target_properties(panta_preprocessing PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include) 
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64") 
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
add_library(panta_preprocessing SHARED IMPORTED GLOBAL) 
set_target_properties(panta_preprocessing PROPERTIES 
                    IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/aarch64/librs_preprocessing.so) 
set_target_properties(panta_preprocessing PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include) 
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")


