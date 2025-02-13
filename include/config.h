#ifndef CONFIG_H
#define CONFIG_H

#include <functional>
#include <array>

struct ProblemConfig {
    using vtype = std::vector<double>;
    using xtype = std::vector<double>;

    double dt{1.0};
    int N{1};
    int nv{1};

    std::vector<std::vector<vtype>> v_feasible;
    xtype x0{};

    std::vector<double> (*running_cost)(const vtype& vi, const std::vector<double>& ri, int i, double dt){};
    double (*sort_key)(const std::vector<double>& x){};

    bool is_dynamic_cost{false};
    std::vector<double> (*dynamic_cost)(const xtype& xi, const std::vector<double>& ri, int i, double dt){};
    std::vector<double> (*next_state_f)(const xtype& xi, const vtype& vi, int i, double dt){};

    std::vector<std::pair<std::vector<int>, std::vector<double>>>dwell_time_cons;
};

#endif // CONFIG_H
