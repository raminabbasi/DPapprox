cmake_minimum_required(VERSION 3.20)
project(MyProject)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=native -DNDEBUG")

# Set the path to your local DPapprox library (modify this path)
set(DPAPPROX_DIR "..")

# Include DPapprox headers
include_directories(${DPAPPROX_DIR}/include)

# Link to DPapprox library without installation
link_directories(${DPAPPROX_DIR}/build)

function(add_problem_executable name)
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name} DPapprox) # Link with the local library
endfunction()

add_problem_executable(mwe)
add_problem_executable(trj)
add_problem_executable(simple)
add_problem_executable(dtd)
add_problem_executable(rck)
add_problem_executable(rck_robust)
add_problem_executable(run_all)
