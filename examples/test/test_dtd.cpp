// Test simple example

#include <gtest/gtest.h>
#include "DPapprox.h"

namespace DTD {
    using namespace DPapprox;

    std::vector<double> q_low = {10.0, 0.0};
    std::vector<double> q_high = {10.0, 10.0};

    double dot_product(const ProblemConfig::disc_vector& a, const std::vector<double>& b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi, const std::vector<double>& ri, int i, double dt) {
        std::vector<double> p{0.0};

        double tolerance = 1e-3;

        if (!(dot_product(vi, q_low) - tolerance <= ri.at(0) && ri.at(0) <= dot_product(vi, q_high) + tolerance)) {
            return {1e9};
        }
        if (vi == ProblemConfig::disc_vector {0,0}) p = {0.0};
        if (vi == ProblemConfig::disc_vector {1,0}) p = {10.0};
        if (vi == ProblemConfig::disc_vector {0,1}) p = {1.1 * ri[0]};
        if (vi == ProblemConfig::disc_vector {1,1}) p = {10.0 + 1.1 * (ri[0] - 10.0)};

        return (p * dt);
    }
}
TEST(example_results_test, dtd) {
    using namespace DPapprox;
// Load data
    std::string filename = "../../examples/data/dtd.csv";
    std::string solution = "../../examples/data/sol_dtd.csv";
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    std::vector<ProblemConfig::disc_vector> v_sol = read_csv(solution);

// Define Problem
    ProblemConfig config;
    config.N = 500;
    config.v_feasible.assign(config.N, {{1, 1}, {0, 1}, {1, 0}, {0, 0}});
    config.dt = 0.02;
    config.stage_cost = DTD::stage_cost;
    double min_dwell_time = 0.2;
    config.dwell_time_cons = { {{0}, {min_dwell_time, min_dwell_time}},
                               {{1}, {min_dwell_time, min_dwell_time}}};
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
