#include "solver.h"
#include <stdexcept>

Solver::Solver(const std::vector<std::vector<double>>& v_rel, const std::vector<std::vector<int>>& v_feasible, double dt)
        : v_rel(v_rel), v_feasible(v_feasible), dt(dt) {
    running_cost = simple_rounding;
    sort_key = [](double x){return x;};
}

void Solver::solve() {

    for (int v_0 : v_feasible[0]) {
        std::pair<int, int> v_ini = {v_0, 0};
        cost_to_go[v_ini] = running_cost(v_0, v_rel[0][0], 0, dt);
    }

    auto num = v_rel.size();
    double cost;
    double v;
    int v_end;
    double cost_end;

    for (int i = 0; i < num - 1; ++i) {
        for (int vni : v_feasible[i + 1]) {
            std::pair<int, int> v_nxt = {vni, i + 1};

            double opt = INFTY;
            double c = running_cost(vni, v_rel[i + 1][0], i + 1, dt);
            for (int vi : v_feasible[i]) {
                std::pair<int, int> v_now = {vi, i};

                v = cost_to_go[v_now];
                cost = c + v;

                if (sort_key(cost) < sort_key(opt)) {
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

    cost_end = INFTY;

    for (const auto& key : keys) {
        auto it = cost_to_go.find(key);
        if (it != cost_to_go.end()) {
            if (sort_key(it->second) < sort_key(cost_end)) {
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

double Solver::simple_rounding(int vi, double ri, int i, double dt) {
    double sum;
    sum = std::pow(vi - ri, 2);
    return std::sqrt(sum);
}

