cmake_minimum_required(VERSION 3.20)
project(DPapprox)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=native -DNDEBUG")

# Create a library for the solver
add_library(solver STATIC src/solver.cpp)

# Add source files
add_executable(DPapprox src/main.cpp)
target_link_libraries(DPapprox solver)
target_include_directories(DPapprox PRIVATE include)
