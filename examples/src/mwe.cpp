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