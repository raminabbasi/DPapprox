#include "solver.h"
#include <stdexcept>
#include <iostream>

Solver::Solver(const std::vector<std::vector<double>>& v_rel, const std::vector<std::vector<int>>& v_feasible, double dt)
        : v_rel(v_rel), v_feasible(v_feasible), dt(dt) {
    running_cost = simple_rounding;
}

void Solver::solve() {

    for (int v_0 : v_feasible[0]) {
        std::pair<int, int> v_ini = {v_0, 0};
        cost_to_go[v_ini] = running_cost(v_0, v_rel[0], 0);
    }

    auto num = v_rel.size();
    double cost;
    double v;
    int v_end;
    double cost_end;

    for (int i = 0; i < num - 1; ++i) {
        for (int vni : v_feasible[i + 1]) {
            std::pair<int, int> v_nxt = {vni, i + 1};

            double opt = std::numeric_limits<double>::infinity();
            double c = running_cost(vni, v_rel[i + 1], i + 1);

            for (int vi : v_feasible[i]) {
                std::pair<int, int> v_now = {vi, i};

                v = cost_to_go[v_now];
                cost = c + v;

                if (cost < opt) {
                    opt = cost;
                    cost_to_go[v_nxt] = opt;
                    path_to_go[v_nxt] = vi;
                }
            }
        }
    }

    std::vector<std::pair<int, int>> keys;
    for (int val : v_feasible[0]) {
        keys.emplace_back(val, num - 1);
    }

    cost_end = std::numeric_limits<double>::infinity();

    for (const auto& key : keys) {
        auto it = cost_to_go.find(key);
        if (it != cost_to_go.end()) {
            std::cout << "Found: (" << key.first << ", " << key.second << ") -> " << it->second << "\n";
            if (it->second < cost_end) {
                cost_end = it->second;
                v_end = key.first;
            }
        }
    }

    std::vector<int> optimum_path = {v_end};

    for (auto i = num - 1; i > 0; --i) {
        int next_vi = path_to_go[{optimum_path[0], i}];
        optimum_path.insert(optimum_path.begin(), next_vi);
    }
    solution.first = optimum_path;
    solution.second = cost_end;

}

double zero_running(int vi, const std::vector<double>& ri, int i) {
    return 0.0;
}

double simple_rounding(int vi, const std::vector<double>& ri, int i) {
    double sum = 0.0;
    for (double r : ri) {
        sum += std::pow(vi - r, 2);
    }
    return std::sqrt(sum);
}

double Solver::sumup_rounding(int vi, const std::vector<double>& ri, int i) const {
    return (vi - ri[0]) * dt;
}

