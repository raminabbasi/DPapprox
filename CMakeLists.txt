cmake_minimum_required(VERSION 3.20)
project(DPapprox)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

# Create library
add_library(DPapprox STATIC src/DPapprox.cpp src/utils/io_utils.cpp src/utils/vector_ops.cpp)

# Public include directory
target_include_directories(DPapprox PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include> # For installation
)

# Installation rules
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
