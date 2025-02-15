#include "DPapprox.h"
#include <vector>

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

int main(){

    ProblemConfig config;
    std::vector<std::vector<double>> v_rel{{4/8, 0.0, 7/8, 7/8},
                                           {3/8, 3/8, 1/8, 1/8},
                                           {1/8, 5/8, 0.0, 0.0}};
    config.N = 4;
    config.dt = 1.0;
    config.v_feasible.assign(config.N, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    config.running_cost = running_cost;
    config.sort_key = sort_key;
    config.dwell_time_cons = {{{1}, {1.5, 0.5, 0.5}}};
    config.dwell_time_init = {{1.5, 0.5, 0.5}};

    Solver solver(v_rel, config);
    solver.solve();

    for (std::size_t i = 0; i < v_rel.size(); i++){
        for (auto &v: solver.solution.optimum_path)
            std::cout << v.at(i) << " ";
        std::cout << std::endl;
    }


    std::cout << std::endl;
    return 0;
}