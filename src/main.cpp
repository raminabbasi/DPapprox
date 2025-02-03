#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "solver.h"
#include "../include/config.h"
#include "./utils/io_utils.h"

double running_cost(int vi, double ri, int i, double dt){
    return (vi - ri) * dt;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <data.csv>" << std::endl;
        return EXIT_FAILURE;
    }
    std::string filename = argv[1];
    std::vector<std::vector<double>> v_rel = read_csv(filename);
//


    ProblemConfig config;
    config.N = 500;
    config.v_feasible.assign(config.N, {1, 0, -1});
    config.dt = 0.02;
    config.running_cost = running_cost;
    config.sort_key = [](double x){return std::fabs(x);};
    double min_dwell_time = 0.3;
    config.dwell_time_cons = { {{1}, {min_dwell_time}},
                               {{0}, {min_dwell_time}},
                               {{-1}, {min_dwell_time}}};

    Solver solver(v_rel, config);

    auto start = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Optimal path: ";
    for (int v : solver.solution.first){
        std::cout << v << " ";
    }
    write_csv("_trj.csv", solver.solution.first);
    std::cout << "\nFinal cost: " << solver.solution.second << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";
    return 0;
}
