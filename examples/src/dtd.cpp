#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include "DPapprox.h"

using namespace DPapprox;

std::vector<double> q_low = {10.0, 0.0}; 
std::vector<double> q_high = {10.0, 10.0}; 

double dot_product(const ProblemConfig::vtype& a, const std::vector<double>& b) {
    return a[0] * b[0] + a[1] * b[1];
}

std::vector<double> running_cost(const ProblemConfig::vtype& vi, const std::vector<double>& ri, int i, double dt) {
    std::vector<double> p{0.0};

    double tolerance = 1e-3;

    if (!(dot_product(vi, q_low) - tolerance <= ri.at(0) && ri.at(0) <= dot_product(vi, q_high) + tolerance)) {
        return {1e9};
    }
    if (vi == ProblemConfig::vtype {0,0}) p = {0.0};
    if (vi == ProblemConfig::vtype {1,0}) p = {10.0};
    if (vi == ProblemConfig::vtype {0,1}) p = {1.1 * ri[0]};
    if (vi == ProblemConfig::vtype {1,1}) p = {10.0 + 1.1 * (ri[0] - 10.0)};

    return (p * dt);
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
    config.v_feasible.assign(config.N, {{1, 1}, {0, 1}, {1, 0}, {0, 0}});
    config.dt = 0.02;
    config.running_cost = running_cost;
    double min_dwell_time = 0.2;
    config.dwell_time_cons = { {{0}, {min_dwell_time, min_dwell_time}},
                               {{1}, {min_dwell_time, min_dwell_time}}};

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