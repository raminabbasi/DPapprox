cmake_minimum_required(VERSION 3.20)
project(DPapprox)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set(DPAPPROX_SOURCES
        src/DPapprox.cpp
        src/utils/io_utils.cpp
        src/utils/vector_ops.cpp
)

add_library(DPapprox SHARED ${DPAPPROX_SOURCES})

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

option(WITH_EXAMPLES "Build the examples" OFF)
option(WITH_TESTS "Build tests in examples" OFF)

if (WITH_EXAMPLES)
    set(WITH_TESTS ${WITH_TESTS} CACHE BOOL "Build tests in examples" FORCE)
    add_subdirectory(examples)
endif()