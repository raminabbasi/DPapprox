#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include "DPapprox.h"

using namespace DPapprox;

std::vector<double> stage_cost(const ProblemConfig::disc_vector& vi, const std::vector<double>& ri, int i, double dt){
    return {(vi - ri) * dt};
}


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.csv>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string filename = argv[1];
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    ProblemConfig config;
    config.N = 1000;
    config.v_feasible.assign(config.N, {{0}, {1}});
    config.dt = 0.0005;
    config.stage_cost = stage_cost;
    config.objective = [](const std::vector<double>& x){return std::fabs(x.at(0));};
    double min_dwell_time = 0.01;
    config.dwell_time_cons = { {{1}, {min_dwell_time}},
                               {{0}, {min_dwell_time}}};

    DPapprox::Solver solver(v_rel, config);

    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Optimal path: ";
    for (const ProblemConfig::disc_vector& v : solver.solution.optimum_path){
        std::cout << v[0] << " ";
    }
    std::cout << std::endl;
    if (argc > 2){
        Log(INFO) << "printing solution into output";
        write_csv(argv[2], solver.solution.optimum_path);
    }
    std::cout << "\nFinal cost: " << solver.solution.f << std::endl;
    std::cout << "Success: " << solver.solution.success << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";
    return 0;
}
