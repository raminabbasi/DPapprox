// Test simple example

#include <gtest/gtest.h>
#include "DPapprox.h"

namespace RCK {
    using namespace DPapprox;

    std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi, const std::vector<double>& ri, int i, double dt){
        return {(vi - ri) * dt};
    }
}
TEST(example_results_test, rck) {

// Load data
    std::string filename = "../data/rck.csv";
    std::string solution = "../data/sol_rck.csv";
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    std::vector<ProblemConfig::disc_vector> v_sol = read_csv(solution);

// Define Problem
    ProblemConfig config;
    config.N = 1000;
    config.v_feasible.assign(config.N, {{0}, {1}});
    config.dt = 0.0005;
    config.stage_cost = RCK::stage_cost;
    config.objective = [](const std::vector<double>& x){return std::fabs(x.at(0));};
    double min_dwell_time = 0.01;
    config.dwell_time_cons = { {{1}, {min_dwell_time}},
                               {{0}, {min_dwell_time}}};
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