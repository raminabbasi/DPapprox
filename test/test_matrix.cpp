// Test simple example

#include <gtest/gtest.h>
#include "../include/DPapprox.h"

namespace MATRIX {
    using namespace DPapprox;

    double objective(const std::vector<double>& x) {
        if (x.empty()) return 0.0;

        double max_value = *std::max_element(x.begin(), x.end(), [](double a, double b) {
            return std::abs(a) < std::abs(b);
        });

        return std::abs(max_value);
    }

    std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi, const std::vector<double>& ri, int i, double dt){
        return {(vi - ri) * dt};
    }

    std::vector<double> custom_cost(const ProblemConfig::disc_vector& vni, const std::vector<double>& cost_nxt, const ProblemConfig::disc_vector& vi,
                                    const ProblemConfig::CostMap& cost_to_go, const ProblemConfig::PathMap& path_to_go, int i, double dt){


        std::pair<ProblemConfig::disc_vector, int> v_now{vi, i};
        std::vector<double> val = cost_to_go.at(v_now);

        if (objective(cost_nxt + cost_to_go.at(v_now)) > objective(val)){
            val = cost_nxt + cost_to_go.at(v_now);
        }

        for (int j = i - 1; j >= 0; j--){
            v_now = {path_to_go.at(v_now), j};
            if (objective(cost_nxt + cost_to_go.at(v_now)) > objective(val)){
                val = cost_nxt + cost_to_go.at(v_now);
            }
        }

        return val;
    }
}
TEST(example_results_test, matrix) {

// Define Problem
    ProblemConfig config;
    std::vector<std::vector<double>> v_rel{{4.0/8.0, 0.0, 7.0/8.0, 7.0/8.0},
                                           {3.0/8.0, 3.0/8.0, 1.0/8.0, 1.0/8.0},
                                           {1.0/8.0, 5.0/8.0, 0.0, 0.0}};

    std::vector<std::vector<double>> v_sol{{0.0, 1.0, 0.0},
                                           {0.0, 0.0, 1.0},
                                           {1.0, 0.0, 0.0},
                                           {1.0, 0.0, 0.0}};

    config.N = 4;
    config.dt = 1.0;
    config.v_feasible.assign(config.N, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    config.stage_cost = MATRIX::stage_cost;
    config.objective = MATRIX::objective;
    config.customize = true;
    config.custom_cost = MATRIX::custom_cost;
    config.dwell_time_cons = {{{1}, {1.5, 0.5, 0.5}}};
    config.dwell_time_init = {{1.5, 0.5, 0.5}};
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