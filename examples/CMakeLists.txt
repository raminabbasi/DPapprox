set(DPAPPROX_DIR "..")

include_directories(${DPAPPROX_DIR}/include)
link_directories(${DPAPPROX_DIR}/build)

function(add_problem_executable name)
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name} DPapprox)
endfunction()

add_problem_executable(mwe)
add_problem_executable(matrix)
add_problem_executable(trj)
add_problem_executable(simple)
add_problem_executable(dtd)
add_problem_executable(rck)
add_problem_executable(rck_robust)
add_problem_executable(run_all)

option(WITH_TESTS "Build tests" ON)

if (WITH_TESTS)
    # GTest
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    add_executable(run_test
            test/test_trj.cpp
            test/test_simple.cpp
            test/test_dtd.cpp
            test/test_rck.cpp
            test/test_rck_robust.cpp
            test/test_matrix.cpp
            test/test_mwe.cpp
    )

    target_link_libraries(run_test ${GTEST_LIBRARIES} GTest::gtest GTest::gtest_main pthread DPapprox)
endif()