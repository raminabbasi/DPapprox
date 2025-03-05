/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#include "DPapprox.h"
#include <vector>
#include <chrono>

using namespace DPapprox;

double norm(const std::vector<double>& x) {
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
    std::vector<double> V = cost_to_go.at(v_now);

    double V_max = (i > 0) ? V.back() : norm(V);
    if (i == 0)
        V.push_back(V_max);

    std::vector<double> cost(V.begin(), V.end() - 1);
    std::vector<double> cost_max = cost + cost_nxt;
    double V_nxt = norm(cost_max);

    cost_max.push_back((V_nxt > V_max) ? V_nxt : V_max);

    return cost_max;
}

int main(){

    ProblemConfig config;
    std::vector<std::vector<double>> v_rel{{4.0/8.0, 0.0, 7.0/8.0, 7.0/8.0},
                                           {3.0/8.0, 3.0/8.0, 1.0/8.0, 1.0/8.0},
                                           {1.0/8.0, 5.0/8.0, 0.0, 0.0}};
    config.N = 4;
    config.dt = 1.0;
    config.v_feasible.assign(config.N, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    config.stage_cost = stage_cost;
    config.objective = [](const std::vector<double>& x){return x.back();};
    config.customize = true;
    config.custom_cost = custom_cost;
    config.dwell_time_cons = {{{1}, {1.5, 0.5, 0.5}}};
    config.dwell_time_init = {{1.5, 0.5, 0.5}};

    Solver solver(v_rel, config);
    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();

    for (std::size_t i = 0; i < v_rel.size(); i++){
        for (auto &v: solver.solution.optimum_path)
            std::cout << v.at(i) << " ";
        std::cout << std::endl;
    }

    std::cout << "\nFinal cost: " << solver.solution.objective << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";
    return 0;
}
