// Test trj example

#include <gtest/gtest.h>
#include "../include/DPapprox.h"

using namespace DPapprox;

std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi, const std::vector<double>& ri, int i, double dt){
    return {(vi - ri) * dt};
}

TEST(DwellTimeTest, ExampleCase1) {

std::string filename = "../examples/data/trj.csv";
std::string solution = "../examples/data/sol_trj.csv";
std::vector<std::vector<double>> v_rel = read_csv(filename);
std::vector<ProblemConfig::disc_vector> v_sol = read_csv(solution);

ProblemConfig config;
config.N = 500;
config.v_feasible.assign(config.N, {{1}, {0}, {-1}});
config.dt = 0.02;
config.stage_cost = stage_cost;
config.objective = [](const std::vector<double>& x){return std::fabs(x.at(0));};
double min_dwell_time = 0.3;
config.dwell_time_cons = { {{1}, {min_dwell_time}},
                           {{0}, {min_dwell_time}},
                           {{-1}, {min_dwell_time}}};

DPapprox::Solver solver(v_rel, config);

solver.solve();

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