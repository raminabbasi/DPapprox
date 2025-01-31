#include <iostream>
#include "solver.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

void write_csv(const std::string& filename, std::vector<int> data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << " for writing." << std::endl;
        return;
    }

    for (int row : data) {
        file << row << ",";
        file << "\n";
    }

    file.close();
}

std::vector<std::vector<double>> read_csv(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return data;
    }

    std::string line;
    if (std::getline(file, line)) {  // Read the single row
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            data.push_back({std::stod(value)});  // Store each value as its own row
        }
    }

    file.close();
    return data;
}

double running_cost(int vi, double ri, int i, double dt){
    return (vi - ri) * dt;
}

int main() {

    std::string filename = "trj.csv";  // Replace with your actual CSV file
    std::vector<std::vector<double>> v_rel = read_csv(filename);
    //std::vector<std::vector<double>> v_rel = {{0.3, 0.4, 0.5}};
    std::vector<std::vector<int>> v_feasible(v_rel.size(), {1, 0, -1});

    std::size_t N = v_rel.size();

    double tf = 10.0;
    double dt = tf/N;
    double min_dwell_time = 0.3;

    auto start = std::chrono::high_resolution_clock::now();
    Solver solver(v_rel, v_feasible, dt);

    solver.running_cost = running_cost;
    solver.sort_key = [](double x){return std::fabs(x); };
    solver.dwell_time_cons = { {{-1}, {min_dwell_time}}};

    solver.solve();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Optimal path: ";
    write_csv("sol_trj.csv", solver.solution.first);
    std::cout << "\nFinal cost: " << solver.solution.second << std::endl;
    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds > (end - start).count()<< " microseconds\n";
    return 0;
}
