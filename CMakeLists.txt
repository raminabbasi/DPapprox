cmake_minimum_required(VERSION 3.20)
project(DPapprox)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=native -DNDEBUG")
set(DPAPPROX_SOURCES
        src/DPapprox.cpp
        src/utils/io_utils.cpp
        src/utils/vector_ops.cpp
        src/utils/logger.cpp
)

add_library(DPapprox STATIC ${DPAPPROX_SOURCES})

target_include_directories(DPapprox PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> # For installation
)

if (NOT DEFINED LOG_LEVEL)
    set(LOG_LEVEL INFO)
endif()

add_compile_definitions(LOG_LEVEL=${LOG_LEVEL})

include(GNUInstallDirs)

install(TARGETS DPapprox EXPORT DPapproxConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT DPapproxConfig
        NAMESPACE DPapprox::
        DESTINATION ${CMAKE_INSTALL_DATADIR}/DPapprox/cmake
)

# GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(run_test
        test/test_trj.cpp
        test/test_simple.cpp
        test/test_dtd.cpp
)

target_link_libraries(run_test ${GTEST_LIBRARIES} GTest::gtest GTest::gtest_main pthread DPapprox)

