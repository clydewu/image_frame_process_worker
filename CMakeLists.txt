cmake_minimum_required(VERSION 2.8)

project(HOAWorker)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")

include(DpkgBuildFlags)
get_dpkg_buildflags_c(DPKG_BUILDFLAGS_C)
get_dpkg_buildflags_cxx(DPKG_BUILDFLAGS_CXX)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${DPKG_BUILDFLAGS_C}   -std=c11   -Wall -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DPKG_BUILDFLAGS_CXX} -std=c++11 -Wall -pthread")
set(CMAKE_C_FLAGS_DEBUG   "-g -O0") # FIXME Ideal is '-Og' but a bug in GCC prevents this,
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0") # causing "may be used uninitialized" errors: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=58455
set(CMAKE_C_FLAGS_RELEASE   "-DNDEBUG -O2") # CMake adds '-O3' by default
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O2") # but currently only '-O2' is tested
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # Use "-fPIC" for all targets, including static libs


find_package(PkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv>=2.4.0)
pkg_check_modules(PROTOBUF REQUIRED protobuf)
pkg_check_modules(LOG4CXX REQUIRED liblog4cxx)
#pkg_check_modules(RABBITMQ REQUIRED rabbitmq)

ADD_SUBDIRECTORY (src)

get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
