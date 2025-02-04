#include "../../include/io_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>

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
