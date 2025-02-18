/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include "DPapprox.h"

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

ProblemConfig::state_vector state_transition(const ProblemConfig::state_vector& state,
                                  const ProblemConfig::disc_vector& input,
                                  int /*i*/,
                                  double dt){
    auto f = [&](const ProblemConfig::state_vector& s) {
        double r = s.at(0);
        double v = s.at(1);
        double m = s.at(2);
        double u = input.at(0);

        double rdot = v;
        double vdot = - 1 / (r * r) + 1 / m * (T_max * u - (A * v * v * exp(-k * (r - r0))));
        double mdot = -b * u;

        return ProblemConfig::state_vector{rdot, vdot, mdot};
    };

    ProblemConfig::state_vector k1 = f(state);
    ProblemConfig::state_vector k2 = f(state + k1 * (dt / 2.0));
    ProblemConfig::state_vector k3 = f(state + k2 * (dt / 2.0));
    ProblemConfig::state_vector k4 = f(state + k3 * dt);

    return state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}


std::vector<double> state_cost(const ProblemConfig::state_vector& xi,
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
    config.stage_cost = stage_cost;
    config.objective = [](const std::vector<double>& x){return std::fabs(x.at(0));};
    config.include_state = true;
    config.state_cost = state_cost;
    config.state_transition = state_transition;
    config.x0 = {1.0, 0.0, 1.0};
    double min_dwell_time = 0.01;
    config.dwell_time_cons = { {{1}, {min_dwell_time}},
                               {{0}, {min_dwell_time}}};


    DPapprox::Solver solver(v_rel, config);

    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Optimal path: ";
    for (const ProblemConfig::disc_vector& v : solver.solution.optimum_path){
        std::cout << v[0] << " ";
    }
    std::cout << std::endl;
    if (argc > 2){
        DPapprox::Log(INFO) << "printing path into output";
        write_csv(argv[2], solver.solution.optimum_path);
    }
    if (argc > 3){
        DPapprox::Log(INFO) << "printing state into output";
        write_csv(argv[3], solver.solution.state_to_go);
    }
    std::cout << "\nFinal cost: " << solver.solution.objective << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";

    return 0;
}
