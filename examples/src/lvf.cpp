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

std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi, const std::vector<double>& ri, int i, double dt){
    return {(vi - ri) * dt};
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
    config.v_feasible.assign(config.N, {{0}, {1}});
    config.dt = 12.0/config.N;
    config.stage_cost = stage_cost;
    config.objective = [](const std::vector<double>& x){return std::fabs(x.at(0));};
    double min_dwell_time = 0.2;
    config.dwell_time_cons = { {{0}, {min_dwell_time}},
                               {{1}, {min_dwell_time}}};

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
        DPapprox::Log.log(INFO) << "printing solution into output";
        write_csv(argv[2], solver.solution.optimum_path);
    }
    std::cout << "\nFinal cost: " << solver.solution.objective << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";
    return 0;
}
