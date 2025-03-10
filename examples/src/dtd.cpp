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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.csv>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string filename = argv[1];
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    ProblemConfig config;
    config.N = 500;
    config.v_feasible.assign(config.N, {{1, 1}, {0, 1}, {1, 0}, {0, 0}});
    config.dt = 0.02;
    config.stage_cost = stage_cost;
    double min_dwell_time = 0.2;
    config.dwell_time_cons = { {{0}, {min_dwell_time, min_dwell_time}},
                               {{1}, {min_dwell_time, min_dwell_time}}};

    Solver solver(v_rel, config);

    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Optimal path: ";
    for (const ProblemConfig::disc_vector& v : solver.solution.optimum_path){
        std::cout << v[0] << " ";
    }
    std::cout << std::endl;
    if (argc > 2){
        DPapprox::Log.log(INFO) << "printing solution into output";
        write_csv(argv[2], solver.solution.optimum_path);
    }
    std::cout << "\nFinal cost: " << solver.solution.objective << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";

    return 0;

}
