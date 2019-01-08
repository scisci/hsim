cmake_minimum_required(VERSION 3.12)
project(hsimall)

#enable_testing()
set(BUILD_TESTS_DEFAULT OFF) # Turn off htree tests

add_compile_options(-std=c++11)


# Start PhysX config
set(PHYSX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/PhysX/physx")
set(PX_OUTPUT_LIB_DIR "${PHYSX_ROOT_DIR}")
set(PX_OUTPUT_BIN_DIR "${PHYSX_ROOT_DIR}")
set(PXSHARED_PATH "${PHYSX_ROOT_DIR}/../pxshared")
set(ENV{PM_CMakeModules_PATH} "${PHYSX_ROOT_DIR}/../externals/cmakemodules")
set(CMAKE_PREFIX_PATH "${PHYSX_ROOT_DIR}/../externals/opengl-linux")


# Start HTREE lib config
add_subdirectory(extern/htreecpp/htree)
include_directories($<TARGET_PROPERTY:htree,INTERFACE_INCLUDE_DIRECTORIES>)

add_subdirectory(hsim)
include_directories($<TARGET_PROPERTY:hsim,INTERFACE_INCLUDE_DIRECTORIES>)

add_subdirectory(macpoc)
