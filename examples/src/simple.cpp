#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include "DPapprox.h"

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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.csv>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string filename = argv[1];
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    ProblemConfig config;
    config.N = 500;
    config.v_feasible.assign(config.N, {{1, 0}, {0, 1}});
    config.dt = 0.02;
    config.running_cost = running_cost;
    config.sort_key = sort_key;
    double min_dwell_time = 0.2;
    config.dwell_time_cons = { {{0}, {min_dwell_time, min_dwell_time}},
                               {{1}, {0, 0}}};

    Solver solver(v_rel, config);

    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Optimal path: ";
    for (const ProblemConfig::vtype& v : solver.solution.optimum_path){
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