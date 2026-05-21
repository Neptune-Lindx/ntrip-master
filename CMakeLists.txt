cmake_minimum_required(VERSION 3.0)

project(ntrip)

# All executables (neptune_*, examples, etc.) -> <project>/bin/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(NTRIP_VERSION_MAJOR 1)
set(NTRIP_VERSION_MINOR 2)
set(NTRIP_VERSION_PATCH 0)

option(NTRIP_BUILD_CASTER "Build ntrip caster" OFF)
option(NTRIP_BUILD_CLIENT "Build ntrip client" ON)
option(NTRIP_BUILD_SERVER "Build ntrip server" ON)
option(NTRIP_BUILD_EXAMPLES "Build ntrip examples" OFF)
option(NTRIP_BUILD_NEPTUNE "Build neptune_base and neptune_rover" ON)

# neptune_base requires caster + server in the ntrip library.
if (NTRIP_BUILD_NEPTUNE)
  set(NTRIP_BUILD_CASTER ON CACHE BOOL "Build ntrip caster" FORCE)
  set(NTRIP_BUILD_SERVER ON CACHE BOOL "Build ntrip server" FORCE)
  set(NTRIP_BUILD_CLIENT ON CACHE BOOL "Build ntrip client" FORCE)
endif ()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message("-- Using default build type: Debug.")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors")
endif("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O2 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_INCLUDE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/include)

set(ntrip_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(SYSTEM ${ntrip_include_dirs})

message(STATUS "Source code version: ${NTRIP_VERSION_MAJOR}.${NTRIP_VERSION_MINOR}.${NTRIP_VERSION_PATCH}")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(git_utility)
set(GIT_BRANCH "")
get_git_branch(GIT_BRANCH)
message(STATUS "Git branch: ${GIT_BRANCH}")
set(GIT_HASH "")
get_git_hash(GIT_HASH)
message(STATUS "Git hash: ${GIT_HASH}")

configure_file(
  ${PROJECT_SOURCE_DIR}/src/cmake_definition.h.in
  ${PROJECT_SOURCE_DIR}/src/cmake_definition.h
)

aux_source_directory(src src_MAIN)

# Application entry points; built as separate executables below.
list(REMOVE_ITEM src_MAIN "src/neptune_base.cc")
list(REMOVE_ITEM src_MAIN "src/neptune_rover.cc")

if (NOT NTRIP_BUILD_CASTER)
  list(REMOVE_ITEM src_MAIN "src/ntrip_caster.cc")
  message("-- Not build ntrip caster")
endif ()

if (NOT NTRIP_BUILD_SERVER)
  list(REMOVE_ITEM src_MAIN "src/ntrip_server.cc")
  message("-- Not build ntrip server")
endif ()

if (NOT NTRIP_BUILD_CLIENT)
  list(REMOVE_ITEM src_MAIN "src/ntrip_client.cc")
  message("-- Not build ntrip client")
endif ()

add_library(${PROJECT_NAME}
  ${src_MAIN}
)

if (WIN32)
  target_link_libraries(${PROJECT_NAME} ws2_32)
endif (WIN32)

set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${ntrip_include_dirs})

if (NTRIP_BUILD_NEPTUNE)
  add_executable(neptune_base src/neptune_base.cc)
  add_dependencies(neptune_base ${PROJECT_NAME})
  target_link_libraries(neptune_base ${PROJECT_NAME})
  if (WIN32)
    target_link_libraries(neptune_base ws2_32)
  endif ()

  add_executable(neptune_rover src/neptune_rover.cc)
  add_dependencies(neptune_rover ${PROJECT_NAME})
  target_link_libraries(neptune_rover ${PROJECT_NAME})
  if (WIN32)
    target_link_libraries(neptune_rover ws2_32)
  endif ()

  message(STATUS "Build neptune_base and neptune_rover")
endif ()

if (NTRIP_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif (NTRIP_BUILD_EXAMPLES)
