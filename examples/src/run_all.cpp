#include <iostream>
#include <cstdlib>  // for std::system

int main() {
    std::cout << "Running all problem executables...\n";

    std::string executables[] = {"trj", "simple", "dtd", "rck", "rck_robust"};
    int failed = 0;
    for (std::string& exe : executables) {

        std::string build = "./build/";
        std::string input = " ./data/" + std::string(exe) + ".csv";
        std::string sol_v = " ./data/v_" + std::string(exe) + ".csv";
        std::string sol_x = " ./data/x_" + std::string(exe) + ".csv";

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
