/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#include "../../include/io_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>


void DPapprox::write_csv(const std::string& filename, const std::vector<std::vector<double>>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << " for writing." << std::endl;
        return;
    }

    for (const DPapprox::ProblemConfig::disc_vector& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];  

            if (i < row.size() - 1) {  
                file << ",";
            }
        }
        file << "\n";  
    }

    file.close();
}



std::vector<std::vector<double>> DPapprox::read_csv(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return data;
    }

    std::string line;
    while (std::getline(file, line)) {  
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {  
            row.push_back(std::stod(value));
        }

        data.push_back(row);  
    }

    file.close();
    return data;
}

std::vector<double> DPapprox::get_column(const std::vector<std::vector<double>> &v, size_t col_index) {
    std::vector<double> column;
    for (const auto& row : v) {
        if (col_index < row.size()) {
            column.push_back({row[col_index]});
        }
    }
    return column;
}
