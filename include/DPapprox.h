/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#ifndef DPAPPROX_H
#define DPAPPROX_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include "config.h"
#include "io_utils.h"
#include "vector_ops.h"
#include "./logger.h"

namespace  DPapprox {

constexpr double EPSILON = 1e-9;
const std::vector<double> INFTY {1e20};
constexpr double DWELL_FLAG = -2;

struct Solution {
    std::vector<ProblemConfig::disc_vector> optimum_path;
    std::unordered_map<std::pair<ProblemConfig::disc_vector, int>, std::vector<double> , ProblemConfig::pair_hash> optimum_cost;
    std::vector<ProblemConfig::state_vector> optimum_state;
    double f;
    bool success;
};

class Solver {

public:
    Solver(const std::vector<std::vector<double>> &v_rel, const ProblemConfig &config);
    void solve();
    Solution solution;

private:
    ProblemConfig dp;
    std::vector<std::vector<double>> v_rel;

    std::vector<ProblemConfig::CostMap> timers;
    ProblemConfig::CostMap cost_to_go;
    ProblemConfig::PathMap path_to_go;
    ProblemConfig::PathMap next_state;

    void set_timers();

    std::vector<double>
    dwell_time(const std::pair<std::vector<int>, std::vector<double>> &con,
               std::vector<double> yi,
               const ProblemConfig::disc_vector& vi,
               const ProblemConfig::disc_vector& vni, int i) const;
};

}
#endif 
