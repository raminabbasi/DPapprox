#include "solver.h"
#include <stdexcept>
#include <iostream>

Solver::Solver(const std::vector<std::vector<double>>& v_rel, const ProblemConfig& config)
        : v_rel(v_rel),
        v_feasible(config.v_feasible),
        dt(config.dt),
        dwell_time_init{},
        dwell_time_cons{},
        running_cost(simple_rounding),
        sort_key([](double x){return x;}){
    if (config.running_cost){
        running_cost = config.running_cost;
    }
    if (config.sort_key){
        sort_key = config.sort_key;
    }
    if (!config.dwell_time_cons.empty()){
        dwell_time_cons = config.dwell_time_cons;
    }
}

void Solver::solve() {
    set_timers();

    for (int v_0 : v_feasible[0]) {
        std::pair<int, int> v_ini = {v_0, 0};
        cost_to_go[v_ini] = running_cost(v_0, v_rel[0][0], 0, dt);
    }


    double cost(0);
    double v(0);
    int v_end(0);
    double cost_end(0);

    for (int i = 0; i < N - 1; ++i) {
        for (int vni : v_feasible[i + 1]) {
            std::pair<int, int> v_nxt = {vni, i + 1};

            double opt = std::numeric_limits<double>::infinity();
            double c = running_cost(vni, v_rel[i + 1][0], i + 1, dt);
            for (int vi : v_feasible[i]) {
                std::pair<int, int> v_now = {vi, i};

                std::vector<std::vector<double>> dwell;
                for (size_t k = 0; k < dwell_time_cons.size(); ++k) {
                    auto& con = dwell_time_cons[k];  // Equivalent to "con" in Python
                    auto& timer = timers[k];        // Equivalent to "timer" in Python
                    dwell.push_back(dwell_time(con, timer[v_now], vi, vni, i));
                }

                bool violate_dwell = std::any_of(dwell.begin(), dwell.end(), [](const std::vector<double>& row) {
                    return std::find(row.begin(), row.end(), DWELL_FLAG) != row.end();
                });

                double d = 0;
                if (violate_dwell){
                    d = INFTY;
                }
                v = cost_to_go[v_now];
                cost = (c + v + d);

                if (sort_key(cost) < sort_key(opt)) {



                    opt = cost;
                    cost_to_go[v_nxt] = opt;
                    path_to_go[v_nxt] = vi;

                    for (size_t k = 0; k < dwell.size(); ++k){
                        auto& timer = timers[k];
                        auto& dw = dwell[k];
                        timer[v_nxt] = dw;
                    }
                }
            }
        }
    }

    std::vector<std::pair<int, int>> keys;
    for (int val : v_feasible[0]) {
        keys.emplace_back(val, N - 1);
    }

    cost_end = INFTY;

    for (const auto& key : keys) {
        auto it = cost_to_go.find(key);
        if (it != cost_to_go.end()) {
            if (sort_key(it->second) < sort_key(cost_end)) {
                cost_end = sort_key(it->second);
                v_end = key.first;
            }
        }
    }

    std::vector<int> optimum_path = {v_end};

    for (auto i = N - 1; i > 0; --i) {
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

void Solver::set_timers(){
    if (dwell_time_init.empty()){
        std::size_t num_cons = dwell_time_cons.size();
        dwell_time_init.assign(num_cons, std::vector<double>(num_input, 0.0));
    }
    for (std::vector<double>& timer_0 : dwell_time_init){
        std::unordered_map<std::pair<int, int>, std::vector<double>, pair_hash> timer;
        for (int v_0 : v_feasible[0]){
            timer[{v_0, 0}] = timer_0;
        }
        timers.push_back(timer);
    }
}
std::vector<double> Solver::dwell_time(std::pair<std::vector<int>, std::vector<double>>& con,
                                       std::vector<double> yi, int vi, int vni, int i) const{
    // Subtract dt from each element in yi

    for (double &y : yi) {
        y -= dt;
    }

    std::vector<double> yni = yi;
    for (size_t idx = 0; idx < yi.size(); ++idx) {
        if (yi[idx] <= 0) {
            yni[idx] = 0;
        }
        if (vni != vi) {
            if ((yi[idx] > 0) && (vi == con.first.back())) {
                yni[idx] = DWELL_FLAG;
            } else {
                auto it = std::find(con.first.begin(), con.first.end(), vni);
                if (it != con.first.end()) {
                    size_t vni_idx = std::distance(con.first.begin(), it);
                    if (vni_idx == 0) {
                        yni[idx] = con.second[idx] + ZERO;
                    } else if (con.first[vni_idx - 1] != vi) {
                        yni[idx] = 0;
                    }
                }
            }
        }
    }
    return yni;
}
