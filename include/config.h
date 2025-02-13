#ifndef CONFIG_H
#define CONFIG_H

#include <functional>
#include <array>
#include <cmath>

struct ProblemConfig {
    using vtype = std::vector<double>;
    using xtype = std::vector<double>;

    double dt{1.0};
    int N{1};
    int nv{1};

    std::vector<std::vector<vtype>> v_feasible;
    xtype x0;
    bool is_dynamic_cost{false};

    std::vector<double> (*running_cost)(const vtype& vi, const std::vector<double>& ri, int i, double dt){running_cost_0};
    double (*sort_key)(const std::vector<double>& x){sort_key_0};
    std::vector<double> (*next_state_f)(const xtype& xi, const vtype& vi, int i, double dt){next_state_f_0};
    std::vector<double> (*dynamic_cost)(const xtype& xi, const std::vector<double>& ri, int i, double dt){dynamic_cost_0};

    std::vector<std::pair<std::vector<int>, std::vector<double>>>dwell_time_cons;

    static std::vector<double> running_cost_0(const vtype& vi, const std::vector<double>& ri, int, double){
        double sum;
        sum = std::pow(vi[0] - ri[0], 2);
        return std::vector<double>{std::sqrt(sum)};
    };
    static double sort_key_0(const std::vector<double>& x){
        return x.at(0);
    };
    static std::vector<double> dynamic_cost_0(const xtype& xi, const std::vector<double>&, int, double){
        return std::vector<double>{0};
    };
    static std::vector<double> next_state_f_0(const xtype& xi, const vtype&, int, double){
        return xi;
    };

};

#endif // CONFIG_H
