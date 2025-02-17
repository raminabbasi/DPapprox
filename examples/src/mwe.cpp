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

int main(){

    ProblemConfig config;
    std::vector<std::vector<double>> v_rel{{0.2, 0.8, 0.4}};

    config.N = 3;
    config.v_feasible = {{{0}, {1}}, {{0}, {1}}, {{0}, {1}}};

    DPapprox::Solver solver(v_rel, config);
    solver.solve();

    for (auto& v : solver.solution.optimum_path){
        std::cout << v[0] << " ";
    }
    std::cout << std::endl;
    return 0;
}
