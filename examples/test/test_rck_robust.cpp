// Test simple example

#include <gtest/gtest.h>
#include "DPapprox.h"

namespace RCK_ROBUST {
    using namespace DPapprox;

    std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi,
                                   const std::vector<double>& ri,
                                   int /*i*/,
                                   double dt){
        return {(vi - ri) * dt};
    }

    float A = 310;
    float k = 500;
    float r0 = 1;
    float T_max = 3.5;
    float b = 7;
    float C = 0.6;

    ProblemConfig::traj_vector state_transition(const ProblemConfig::traj_vector& state,
                                                 const ProblemConfig::disc_vector& input,
                                                 int /*i*/,
                                                 double dt){
        auto f = [&](const ProblemConfig::traj_vector& s) {
            double r = s.at(0);
            double v = s.at(1);
            double m = s.at(2);
            double u = input.at(0);

            double rdot = v;
            double vdot = - 1 / (r * r) + 1 / m * (T_max * u - (A * v * v * exp(-k * (r - r0))));
            double mdot = -b * u;

            return ProblemConfig::traj_vector{rdot, vdot, mdot};
        };

        ProblemConfig::traj_vector k1 = f(state);
        ProblemConfig::traj_vector k2 = f(state + k1 * (dt / 2.0));
        ProblemConfig::traj_vector k3 = f(state + k2 * (dt / 2.0));
        ProblemConfig::traj_vector k4 = f(state + k3 * dt);

        return state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
    }


    std::vector<double> state_cost(const ProblemConfig::traj_vector& xi,
                                   const std::vector<double>& /*vi*/,
                                   int /*i*/,
                                   double /*dt*/){
        double r = xi.at(0);
        double v = xi.at(1);

        if ((A * v * v * exp(-k * (r - r0))) > C){
            return {1e20};
        }else{
            return {0};
        }
    }
}
TEST(example_results_test, rck_robust) {

// Load data
    std::string filename = "../data/rck_robust.csv";
    std::string solution = "../data/sol_rck_robust.csv";
    std::string solutionx = "../data/solx_rck_robust.csv";
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    std::vector<ProblemConfig::disc_vector> v_sol = read_csv(solution);
    std::vector<ProblemConfig::traj_vector> x_sol = read_csv(solutionx);

// Define Problem
    ProblemConfig config;
    config.N = 1000;
    config.v_feasible.assign(config.N, {{0}, {1}});
    config.dt = 0.0005;
    config.stage_cost = RCK_ROBUST::stage_cost;
    config.objective = [](const std::vector<double>& x){return std::fabs(x.at(0));};
    config.include_state = true;
    config.state_cost = RCK_ROBUST::state_cost;
    config.state_transition = RCK_ROBUST::state_transition;
    config.x0 = {1.0, 0.0, 1.0};
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

    ProblemConfig::traj_vector x;
    ProblemConfig::traj_vector y;
    size_t xlen = solver.solution.optimum_traj.size();
    for (size_t i = 0; i < xlen; ++i){
        x = solver.solution.optimum_traj.at(i);
        y = x_sol.at(i);
        for (size_t j = 0; j < x.size(); ++j) {
            EXPECT_NEAR(x[j], y[j], 1e-5);
        }
    }
}