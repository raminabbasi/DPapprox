#ifndef CONFIG_H
#define CONFIG_H

#include <functional>
#include <array>

struct ProblemConfig {
    using vtype = std::vector<int>;
    int N{};
    std::vector<std::vector<vtype>> v_feasible;
    double dt{};
    std::vector<double> (*running_cost)(const vtype& vi, std::vector<double> ri, int i, double dt){};
    double (*sort_key)(const std::vector<double>& x){};
    std::vector<std::pair<std::vector<int>, std::vector<double>>>dwell_time_cons;
};

#endif // CONFIG_H
