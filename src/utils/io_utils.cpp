#include "../../include/io_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>


void write_csv(const std::string& filename, const std::vector<ProblemConfig::vtype>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << " for writing." << std::endl;
        return;
    }

    for (const ProblemConfig::vtype& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];  // Write value

            if (i < row.size() - 1) {  // Add comma only between elements, not at the end
                file << ",";
            }
        }
        file << "\n";  // New line after each row
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
    while (std::getline(file, line)) {  // Read each line (row)
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {  // Read each column (value)
            row.push_back(std::stod(value));
        }

        data.push_back(row);  // Store the row in data
    }

    file.close();
    return data;
}