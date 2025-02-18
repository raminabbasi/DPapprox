/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <functional>
#include <array>
#include <cmath>

/* ProblemConfig defines the problem DPapprox solves:
 *
 * disc_vector  : the vector of discrete input.
 * traj_vector  : the vector of differential state of the system (used for state_cost, and state_transition).
 *
 * x0   : the initial state of the system.
 * dt   : time discretization step.
 * N    : number of time discretization nodes.
 *
 * v_feasible       : the set of feasible options of [disc_vector] for each time node in [N].
 * include_state    : a boolean to indicate whether system states should be propagated or not.
 * customize        : a boolean to indicate whether a [custom_cost] is defined.
 *
 * stage_cost       : the stage or running cost of discrete approximation.
 * objective        : the function that receives the approximation cost and returns the objective function to be optimized.
 * state_transition : a function that transitions the system state from previous state to the next.
 * state_cost       : the cost of system states. useful for enforcing state constraints.
 * custom_cost      : the function that takes all the information about the optimum path (cost_to_go) and (path_to_go)
 *                    to allow the user full control over the approximation cost.
 *
 * dwell_time_cons  : a vector that defines the minimum dwell time constraints.
 * dwell_time_init  : a vector that defines the initial value of timers for the minimum dwell time constraints.
 */

struct ProblemConfig {
    using disc_vector = std::vector<double>;
    using traj_vector = std::vector<double>;

    traj_vector x0;
    double dt{1.0};
    int N{1};

    std::vector<std::vector<disc_vector>> v_feasible;

    bool include_state{false};
    bool customize{false};

    std::function<std::vector<double>(const disc_vector&, const std::vector<double>&, int, double)> stage_cost{default_stage_cost};
    std::function<double(const std::vector<double>&)> objective{default_objective};
    std::function<traj_vector(const traj_vector&, const disc_vector&, int, double)> state_transition{default_state_transition};
    std::function<std::vector<double>(const traj_vector&, const std::vector<double>&, int, double)> state_cost{default_state_cost};

    std::vector<std::pair<std::vector<int>, std::vector<double>>>dwell_time_cons;
    std::vector<std::vector<double>> dwell_time_init{};

    static std::vector<double> default_stage_cost(const disc_vector& vi, const std::vector<double>& ri, int, double){
        return std::vector<double>{std::abs(vi[0] - ri[0])};
    };
    static double default_objective(const std::vector<double>& x){
        return x.at(0);
    };
    static std::vector<double> default_state_cost(const traj_vector& xi, const std::vector<double>&, int, double){
        return std::vector<double>{0};
    };
    static std::vector<double> default_state_transition(const traj_vector& xi, const disc_vector&, int, double){
        return xi;
    };

    struct pair_hash {
        template<class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return hash_vector(p.first) ^ std::hash<T2>()(p.second);
        }

    private:
        static std::size_t hash_vector(const std::vector<double>& v) {
            std::size_t seed = v.size();
            for (double num : v) {
                seed ^= std::hash<double>()(num) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    using KeyType = std::pair<disc_vector, int>;
    using CostMap = std::unordered_map<KeyType, std::vector<double> , pair_hash>;
    using PathMap = std::unordered_map<KeyType, disc_vector, pair_hash>;

    std::function<std::vector<double>(const disc_vector&, const disc_vector&, const std::vector<double>&, const PathMap&,
                                      const CostMap&, int, double)> custom_cost{default_custom_cost};

    static std::vector<double> default_custom_cost(const disc_vector& vni, const std::vector<double>& cost_nxt, const disc_vector& vi,
                                               const CostMap& cost_to_go, const PathMap& path_to_go, int i, double dt){
        return {0};
    };
};

#endif 
