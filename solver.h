#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <algorithm>

constexpr double ZERO = 1e-9;
constexpr double INFTY = 1e20;
constexpr double DWELL_FLAG = -1;

class Solver {
public:
    Solver(const std::vector<std::vector<double>>& v_rel, const std::vector<std::vector<int>>& v_feasible, double dt);
    void solve();
    double (*running_cost)(int vi, double ri, int i, double dt);
    double (*sort_key)(double x);
    std::pair<std::vector<int>, double> solution;
    static double simple_rounding(int vi, double ri, int i, double dt);
    std::vector<std::vector<double>> dwell_time_init;
    std::vector<std::pair<std::vector<int>, std::vector<double>>> dwell_time_cons;
    template <typename T>
    static void print_vector(const std::vector<T>& vec);

private:
    std::vector<std::vector<double>> v_rel;
    std::vector<std::vector<int>> v_feasible;
    double dt;

    std::size_t N = v_rel.size();
    std::size_t num_input = v_rel[0].size();

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
        }
    };

    std::vector<std::unordered_map<std::pair<int, int>, std::vector<double>, pair_hash>> timers;
    std::unordered_map<std::pair<int, int>, double, pair_hash> cost_to_go;
    std::unordered_map<std::pair<int, int>, int, pair_hash> path_to_go;

    void set_timers();

    std::vector<double>
    dwell_time(std::pair<std::vector<int>, std::vector<double>> &con, std::vector<double> &yi, int vi, int vni, int i);

    static void print_vector();
};

#endif // SOLVER_H
