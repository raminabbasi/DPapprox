#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include "DPapprox.h"

using namespace DPapprox;

std::vector<double> running_cost(const ProblemConfig::vtype& vi,
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

ProblemConfig::xtype next_state_f(const ProblemConfig::xtype& state,
                                  const ProblemConfig::vtype& input,
                                  int /*i*/,
                                  double dt){
    auto f = [&](const ProblemConfig::xtype& s) {
        double r = s.at(0);
        double v = s.at(1);
        double m = s.at(2);
        double u = input.at(0);

        double rdot = v;
        double vdot = - 1 / (r * r) + 1 / m * (T_max * u - (A * v * v * exp(-k * (r - r0))));
        double mdot = -b * u;

        return ProblemConfig::xtype{rdot, vdot, mdot};
    };

    ProblemConfig::xtype k1 = f(state);
    ProblemConfig::xtype k2 = f(state + k1 * (dt / 2.0));
    ProblemConfig::xtype k3 = f(state + k2 * (dt / 2.0));
    ProblemConfig::xtype k4 = f(state + k3 * dt);

    return state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}


std::vector<double> dynamic_cost(const ProblemConfig::xtype& xi,
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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.csv>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string filename = argv[1];
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    ProblemConfig config;
    config.N = 1000;
    config.v_feasible.assign(config.N, {{0}, {1}});
    config.dt = 0.0005;
    config.running_cost = running_cost;
    config.sort_key = [](const std::vector<double>& x){return std::fabs(x.at(0));};
    config.include_state = true;
    config.dynamic_cost = dynamic_cost;
    config.next_state_f = next_state_f;
    config.x0 = {1.0, 0.0, 1.0};
    double min_dwell_time = 0.01;
    config.dwell_time_cons = { {{1}, {min_dwell_time}},
                               {{0}, {min_dwell_time}}};


    DPapprox::Solver solver(v_rel, config);

    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Optimal path: ";
    for (const ProblemConfig::vtype& v : solver.solution.optimum_path){
        std::cout << v[0] << " ";
    }
    std::cout << std::endl;
    if (argc > 2){
        DPapprox::Log(INFO) << "printing path into output";
        write_csv(argv[2], solver.solution.optimum_path);
    }
    if (argc > 3){
        DPapprox::Log(INFO) << "printing state into output";
        write_csv(argv[3], solver.solution.optimum_state);
    }
    std::cout << "\nFinal cost: " << solver.solution.f << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";

    return 0;
}