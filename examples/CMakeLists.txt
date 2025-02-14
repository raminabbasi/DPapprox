cmake_minimum_required(VERSION 3.20)
project(MyProject)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -march=native -DNDEBUG")

find_package(DPapprox REQUIRED)

function(add_problem_executable name)
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name} DPapprox::DPapprox)
endfunction()

add_problem_executable(trj)
add_problem_executable(simple)
add_problem_executable(dtd)
add_problem_executable(rck)
add_problem_executable(rck_robust)
add_problem_executable(run_all)

