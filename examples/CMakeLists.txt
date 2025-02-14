cmake_minimum_required(VERSION 3.20)

# Define the project name for examples
project(DPapproxExamples)

# Find the main DPapprox library
add_subdirectory(.. ${CMAKE_BINARY_DIR}/DPapprox)  # Link to the main library

# Define an example executable
add_executable(trj trj.cpp)
target_link_libraries(trj PRIVATE DPapprox)
