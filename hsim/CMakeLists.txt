find_package(Boost 1.67.0 REQUIRED COMPONENTS graph)
include_directories(SYSTEM /usr/local/include/eigen3)

file(GLOB srcs "src/*.cpp")
add_library(hsim ${srcs})

target_include_directories(hsim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> 
)

target_link_libraries(hsim
    PUBLIC
        htree
    PRIVATE
        Boost::boost
)

#option(PACKAGE_TESTS "Build the tests" BUILD_TESTS_DEFAULT)
#if(PACKAGE_TESTS)
#    enable_testing()
#    add_subdirectory(tests)
#endif()

