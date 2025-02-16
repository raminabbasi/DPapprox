#include "DPapprox.h"
#include <vector>
#include <chrono>

using namespace DPapprox;

double sort_key(const std::vector<double>& x) {
    if (x.empty()) return 0.0;

    double max_value = *std::max_element(x.begin(), x.end(), [](double a, double b) {
        return std::abs(a) < std::abs(b);
    });

    return std::abs(max_value);
}

std::vector<double> running_cost(const ProblemConfig::vtype& vi, const std::vector<double>& ri, int i, double dt){
    return {(vi - ri) * dt};
}

std::vector<double> custom_cost(const ProblemConfig::vtype& vni, const std::vector<double>& cost_nxt, const ProblemConfig::vtype& vi,
                                const ProblemConfig::CostMap& cost_to_go, const ProblemConfig::PathMap& path_to_go, int i, double dt){


    std::pair<ProblemConfig::vtype, int> v_now{vi, i};
    std::vector<double> val = cost_to_go.at(v_now);

    if (sort_key(cost_nxt + cost_to_go.at(v_now)) > sort_key(val)){
        val = cost_nxt + cost_to_go.at(v_now);
    }

    for (int j = i - 1; j >= 0; j--){
        v_now = {path_to_go.at(v_now), j};
        if (sort_key(cost_nxt + cost_to_go.at(v_now)) > sort_key(val)){
            val = cost_nxt + cost_to_go.at(v_now);
        }
    }

    return val;
}

int main(){

    ProblemConfig config;
    std::vector<std::vector<double>> v_rel{{4.0/8.0, 0.0, 7.0/8.0, 7.0/8.0},
                                           {3.0/8.0, 3.0/8.0, 1.0/8.0, 1.0/8.0},
                                           {1.0/8.0, 5.0/8.0, 0.0, 0.0}};
    config.N = 4;
    config.dt = 1.0;
    config.v_feasible.assign(config.N, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    config.running_cost = running_cost;
    config.sort_key = sort_key;
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

    std::cout << "\nFinal cost: " << solver.solution.f << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";
    return 0;
}