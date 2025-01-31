cmake_minimum_required(VERSION 3.30)
project(DPapprox)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add source files
add_executable(DPapprox main.cpp solver.cpp)
