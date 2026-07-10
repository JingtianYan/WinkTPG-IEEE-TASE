cmake_minimum_required (VERSION 3.7.1)
project(tpg)


set(CMAKE_CXX_STANDARD 20)  # only useful when CMake version >= 3.1
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -g -O3 -ffast-math")
set(CMAKE_LINKER  "/usr/bin/ld")
set(CMAKE_VERBOSE_MAKEFILE OFF)  # Show all commands and parameters
SET(CMAKE_BUILD_TYPE "DEBUG")

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()



include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(tpg ${SOURCES})

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

# Find Boost
find_package(Boost)
if(Boost_FOUND)
    set(Boost_USE_STATIC_LIBS OFF)
    set(Boost_USE_MULTITHREADED ON)
    set(Boost_USE_STATIC_RUNTIME OFF)
    find_package(Boost REQUIRED COMPONENTS system program_options filesystem)
elseif(NOT Boost_FOUND)
    message(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
endif()
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(tpg ${Boost_LIBRARIES})
