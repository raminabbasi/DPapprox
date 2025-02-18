/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#include "DPapprox.h"
#include "logger.h"

namespace DPapprox {

Solver::Solver(const std::vector<std::vector<double>> &v_rel, const ProblemConfig &config)
    : v_rel(v_rel), dp(config)
    {
    Log(INFO) << "Initializing Solver.";

    if (dp.N != static_cast<int>(v_rel[0].size()))
        throw std::runtime_error("Error: N does not match v_rel[0].size().");
}

void Solver::solve() {
    Log(INFO) << "Solving...";
    set_timers();

    std::pair<ProblemConfig::disc_vector, int> v_ini, v_now, v_nxt;
    std::vector<double> opt, c{0}, v{0}, p{0}, d{0}, cost_nxt{0}, cost{0};
    ProblemConfig::traj_vector xni{dp.x0};
    std::vector<std::vector<double>> dwell;
    const int N = dp.N;

    for (auto const& v_0: dp.v_feasible[0]) {
        v_ini = {v_0, 0};
        cost_to_go[v_ini] = dp.stage_cost(v_0, get_column(v_rel, 0), 0, dp.dt);
        if (dp.include_state) next_state[v_ini] = dp.state_transition(dp.x0, v_0, 0, dp.dt);
    }

    for (int i = 0; i < N - 1; ++i) {
        for (auto const& vni: dp.v_feasible[i + 1]) {
            v_nxt = {vni, i + 1};

            opt = {std::numeric_limits<double>::infinity()};
            c = dp.stage_cost(vni, get_column(v_rel, i+1), i + 1, dp.dt);
            for (auto const& vi: dp.v_feasible[i]) {
                v_now = {vi, i};

                dwell.clear();
                for (size_t k = 0; k < dp.dwell_time_cons.size(); ++k) {
                    auto &con = dp.dwell_time_cons[k];
                    auto &timer = timers[k];
                    dwell.push_back(dwell_time(con, timer.at(v_now), vi, vni, i));
                }

                bool violate_dwell = std::any_of(dwell.begin(), dwell.end(), [](const std::vector<double> &row) {
                    return std::find(row.begin(), row.end(), DWELL_FLAG) != row.end();
                });

                d = {0};
                if (violate_dwell)
                    d = INFTY;


                if (dp.include_state) {
                    xni = next_state.at(v_now);
                    p = dp.state_cost(xni, get_column(v_rel, i), i, dp.dt);
                }

                v = cost_to_go.at(v_now);
                cost_nxt = (c + d + p);
                if (!dp.customize)
                    cost = v + cost_nxt;
                else
                    cost = dp.custom_cost(vni, cost_nxt, vi, cost_to_go, path_to_go, i, dp.dt);

                if (dp.objective(cost) < dp.objective(opt)) {

                    opt = cost;
                    cost_to_go[v_nxt] = opt;
                    path_to_go[v_nxt] = vi;

                    if (dp.include_state)
                        next_state[v_nxt] = dp.state_transition(xni, vni, i + 1, dp.dt);


                    for (size_t k = 0; k < dwell.size(); ++k) {
                        auto &timer = timers[k];
                        auto &dw = dwell[k];
                        timer[v_nxt] = dw;
                    }
                }
            }
        }
    }

    std::vector<std::pair<ProblemConfig::disc_vector, int>> keys;
    for (const ProblemConfig::disc_vector &val: dp.v_feasible[0])
        keys.emplace_back(val, N - 1);

    std::vector<double> cost_end{INFTY};
    ProblemConfig::disc_vector v_end{0};

    auto best = std::min_element(
            keys.begin(), keys.end(),
            [&](const auto& a, const auto& b) {
                return dp.objective(cost_to_go[a]) < dp.objective(cost_to_go[b]);
            }
    );

    if (best != keys.end()) {
        cost_end = cost_to_go[*best];
        v_end = best->first;
    }

    std::vector<ProblemConfig::disc_vector> optimum_path {v_end};
    std::vector<ProblemConfig::traj_vector> optimum_traj {};

    optimum_path.reserve(N + 1);
    optimum_traj.reserve(N + 1);

    for (auto i = N - 1; i > 0; --i) {
        optimum_path.emplace(optimum_path.begin(), path_to_go[{optimum_path[0], i}]);
        if (dp.include_state)
            optimum_traj.emplace(optimum_traj.begin(), next_state[{optimum_path[0], i}]);
    }

    solution.optimum_path = optimum_path;
    solution.objective = dp.objective(cost_end);
    solution.success = (solution.objective < INFTY.at(0));
    solution.cost = cost_end;
    if (solution.success)
        Log(INFO) << "Solved.";
    else
        Log(INFO) << "Something went wrong!";


    if (dp.include_state){
        v_ini = {optimum_path.at(0), 0};
        optimum_traj.emplace(optimum_traj.begin(), next_state[v_ini]);
        optimum_traj.emplace(optimum_traj.begin(), dp.x0);
        solution.optimum_traj = optimum_traj;
    }
}

void Solver::set_timers() {
    if (dp.dwell_time_init.empty()) {
        dp.dwell_time_init.resize(dp.dwell_time_cons.size(),
                                  std::vector<double>(dp.v_feasible[0][0].size(), 0.0));
    }

    std::unordered_map<std::pair<ProblemConfig::disc_vector, int>, std::vector<double>, ProblemConfig::pair_hash> timer;
    for (std::vector<double> &timer_0: dp.dwell_time_init) {
        for (const ProblemConfig::disc_vector &v_0: dp.v_feasible[0])
            timer[{v_0, 0}] = timer_0;
        timers.push_back(timer);
    }
}

std::vector<double> Solver::dwell_time(const std::pair<std::vector<int>, std::vector<double>> &con,
                                       std::vector<double> yi, const ProblemConfig::disc_vector& vi,
                                       const ProblemConfig::disc_vector& vni, int i) const {

    for (double &y: yi) {
        y -= dp.dt;
    }
    std::vector<double> yni = yi;
    for (size_t idx = 0; idx < yi.size(); ++idx) {
        if (yi[idx] <= 0)
            yni[idx] = 0;

        if (vni[idx] != vi[idx]) {
            if ((yi[idx] > 0) && (vi[idx] == con.first.back()))
                yni[idx] = DWELL_FLAG;
            else {
                auto it = std::find(con.first.begin(), con.first.end(), vni[idx]);
                if (it != con.first.end()) {
                    size_t vni_idx = std::distance(con.first.begin(), it);
                    if (vni_idx == 0)
                        yni[idx] = con.second[idx] + EPSILON;
                    else if (con.first[vni_idx - 1] != vi[idx])
                        yni[idx] = 0;

                }
            }
        }
    }
    return yni;
}

}
