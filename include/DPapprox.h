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

/*
 * Global parameters for Solver
 * EPSILON  : epsilon is used to make sure dwell times always round up to the closest next time node.
 * INFTY    : defines the infinity penalty value used for violation of constraints.
 * DWELL_FLAG : is used for detecting a dwell time violation.
 */

constexpr double EPSILON = 1e-9;
const std::vector<double> INFTY {1e20};
constexpr double DWELL_FLAG = -2;

/*
 * Solution structure includes the following:
 * optimum_path : the optimum path to go from the final point to the beginning, i.e. the optimal discrete approximation.
 * optimum_traj : the map of state to go, which provides the next [traj_vector] to go at each pair of <disc_vector, i>.
 * cost         : the optimum approximation cost for the optimum path.
 * objective    : the optimum objective for the optimum path.
 * success      : a boolean that is true if objective is less than [INFTY].
 */

struct Solution {
    std::vector<ProblemConfig::disc_vector> optimum_path;
    std::vector<ProblemConfig::traj_vector> optimum_traj;
    std::vector<double> cost;
    double objective;
    bool success;
};

/*
 * Solver class that receives the relaxed solution [v_rel] and an approximation problem [ProblemConfig] and solves
 * the approximation problem using DP algorithm.
 *
 * Solver() : constructs the Solver, by receiving a [v_rel] and a [ProblemConfig].
 * solve()  : solves the discrete approximation problem.
 * solution : records the solution based on [Solution] structure.
 *
 * dp   : the problem to be solved using DP algorithm.
 * v_rel: the relaxed solution.
 * timers       : timers variables that are run along the optimum paths to detect dwell time constraint violation.
 * cost_to_go   : the discrete approximation cost to go. It provides the cost to go for each pair of <disc_vector, i>.
 * path_to_go   : the discrete approximation path to go. It provides the [disc_vector] to go for each pair of <disc_vector, i>.
 * next_state   : specifies the next [traj_vector] to go for each pain of <disc_vector, i>.
 *
 * set_timers   : initializes the timers for dwell time constraints.
 * dwell_time   : it checks the optimum paths for violating the dwell time constraints.
 */

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
