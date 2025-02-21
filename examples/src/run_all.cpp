/* This file is part of DPapprox
 * Copyright (C) 2025 Ramin Abbasi Esfeden.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This software is based on research described in:
 * "A Dynamic Programming-Inspired Approach for Mixed Integer Optimal Control Problems with Dwell Time Constraints,"
 * by Ramin Abbasi Esfeden, Christoph Plate, Sebastian Sager, Jan Swevers, 2025.
 */

#include <iostream>
#include <cstdlib>  

int main() {
    std::cout << "Running all problem executables...\n";

    std::string executables[] = {"trj", "simple", "dtd", "rck", "rck_robust", "mwe", "matrix"};
    int failed = 0;
    for (std::string& exe : executables) {

        std::string build = "./";
        std::string input = " ../../examples/data/" + std::string(exe) + ".csv";
        std::string sol_v = " ../../examples/data/v_" + std::string(exe) + ".csv";
        std::string sol_x = " ../../examples/data/x_" + std::string(exe) + ".csv";

        std::string path = build;
        path += exe;
        path += input;
        path += sol_v;
        std::cout << "-------" << "\nExecuting " << path << std::endl;
        if (exe == "rck_robust") path += sol_x;

        int result = std::system(path.c_str());

        if (result != 0) {
            std::cerr << "Error: " << exe << " failed with code " << result << "\n";
            failed += 1;
        }
    }

    std::cout << "All executables finished. Failed: " << failed << std::endl;
    return 0;
}
