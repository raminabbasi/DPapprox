// Test simple example

#include <gtest/gtest.h>
#include "DPapprox.h"

TEST(example_results_test, mwe) {
    using namespace DPapprox;
// Define Problem
    ProblemConfig config;

    std::vector<std::vector<double>> v_rel{{0.2, 0.8, 0.4}};
    std::vector<std::vector<double>> v_sol{{0.0}, {1.0}, {0.0}};
    config.N = 3;
    config.v_feasible = {{{0}, {1}}, {{0}, {1}}, {{0}, {1}}};

    DPapprox::Solver solver(v_rel, config);
    solver.solve();

// Compare Results
    ProblemConfig::disc_vector v;
    ProblemConfig::disc_vector w;
    size_t len = solver.solution.optimum_path.size();
    for (size_t i = 0; i < len; ++i){
        v = solver.solution.optimum_path.at(i);
        w = v_sol.at(i);
        for (size_t j = 0; j < v.size(); ++j) {
            EXPECT_DOUBLE_EQ(v[j], w[j]);
        }
    }

}