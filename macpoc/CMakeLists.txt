cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0003 NEW) # something about paths

project(macpoc)

# PhysX Render Helpers
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)

file(GLOB srcs "src/*.cpp" "src/*.mm" "src/*.hpp")

link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} PhysX PhysXExtensions PhysXCharacterKinematic ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} hsim MacSandboxUtilities)
