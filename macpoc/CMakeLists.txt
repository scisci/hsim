cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0003 NEW) # something about paths

project(macpoc)

#PhysX
set(TARGET_BUILD_PLATFORM mac)
set(PX_OUTPUT_ARCH x86)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

add_subdirectory(${PHYSX_ROOT_DIR}/source/compiler/cmake ${CMAKE_CURRENT_BINARY_DIR}/physx_bin)

# PhysX Render Helpers
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)


file(GLOB srcs "src/*.cpp" "src/*.hpp")

link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} PhysX PhysXExtensions ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} hsim)
