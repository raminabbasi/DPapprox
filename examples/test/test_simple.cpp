// Test simple example

#include <gtest/gtest.h>
#include "DPapprox.h"

namespace SIMPLE {
    using namespace DPapprox;

    double objective(const std::vector<double> &x) {
        if (x.empty()) return 0.0;

        double max_value = *std::max_element(x.begin(), x.end(), [](double a, double b) {
            return std::abs(a) < std::abs(b);
        });

        return std::abs(max_value);
    }

    std::vector<double>
    stage_cost(const ProblemConfig::disc_vector &vi, const std::vector<double> &ri, int i, double dt) {
        return {(vi - ri) * dt};
    }
}
TEST(example_results_test, simple) {
    using namespace DPapprox;
// Load data
    std::string filename = "../../examples/data/simple.csv";
    std::string solution = "../../examples/data/sol_simple.csv";
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    std::vector<ProblemConfig::disc_vector> v_sol = read_csv(solution);

// Define Problem
    ProblemConfig config;
    config.N = 500;
    config.v_feasible.assign(config.N, {{1, 0}, {0, 1}});
    config.dt = 0.02;
    config.stage_cost = SIMPLE::stage_cost;
    config.objective = SIMPLE::objective;
    double min_dwell_time = 0.2;
    config.dwell_time_cons = { {{0}, {min_dwell_time, min_dwell_time}},
                               {{1}, {0, 0}}};
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
