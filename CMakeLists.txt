cmake_minimum_required(VERSION 3.12)
project(hsimall)

option(MACOS "build for macos" ON)

#enable_testing()
set(BUILD_TESTS_DEFAULT OFF) # Turn off htree tests


if(MACOS)
  # MacOS sandbox utilties, only need this for macos but eh
  # This is above c++ compile options since it is objective c
  add_subdirectory(extern/MacSandboxUtilities)
  include_directories($<TARGET_PROPERTY:MacSandboxUtilities,INTERFACE_INCLUDE_DIRECTORIES>)
endif()



# Start PhysX config
set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/PhysX/physx")
set(PX_OUTPUT_LIB_DIR "${PHYSX_ROOT_DIR}")
set(PX_OUTPUT_BIN_DIR "${PHYSX_ROOT_DIR}")
set(PXSHARED_PATH "${PHYSX_ROOT_DIR}/../pxshared")
set(ENV{PM_CMakeModules_PATH} "${PHYSX_ROOT_DIR}/../externals/cmakemodules")
set(CMAKE_PREFIX_PATH "${PHYSX_ROOT_DIR}/../externals/opengl-linux")

#PhysX
if(MACOS)
  set(TARGET_BUILD_PLATFORM mac)
  set(PX_OUTPUT_ARCH x86)
endif()

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

add_subdirectory(${PHYSX_ROOT_DIR}/source/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/physx_bin)

# Temporary for eigen
include_directories(SYSTEM /usr/local/include/eigen3)

# Start HTREE lib config
add_subdirectory(extern/htreecpp/htree)
include_directories($<TARGET_PROPERTY:htree,INTERFACE_INCLUDE_DIRECTORIES>)

add_subdirectory(hsim)
include_directories($<TARGET_PROPERTY:hsim,INTERFACE_INCLUDE_DIRECTORIES>)


add_subdirectory(macpoc)
