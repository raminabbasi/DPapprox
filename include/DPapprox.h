#ifndef DPAPPROX_H
#define DPAPPROX_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <algorithm>
#include "config.h"
#include "io_utils.h"

namespace  DPapprox {
    constexpr double ZERO = 1e-9;
    constexpr double INFTY = 1e20;
    constexpr double DWELL_FLAG = -2;

    class Solver {
    public:
        Solver(const std::vector<std::vector<double>> &v_rel, const ProblemConfig &config);

        void solve();

        double (*running_cost)(int vi, std::vector<std::vector<double>> ri, int i, double dt);

        double (*sort_key)(double x);

        std::pair<std::vector<int>, double> solution;

        static double simple_rounding(int vi, std::vector<std::vector<double>> ri, int i, double dt);

        std::vector<std::vector<double>> dwell_time_init;
        std::vector<std::pair<std::vector<int>, std::vector<double>>> dwell_time_cons;
        static std::vector<std::vector<double>> get_column(const std::vector<std::vector<double>>& v, size_t col_index);

    private:
        std::vector<std::vector<double>> v_rel;
        std::vector<std::vector<int>> v_feasible;
        double dt;

        std::size_t N = v_rel[0].size();
        std::size_t num_input = v_rel.size();

        struct pair_hash {
            template<class T1, class T2>
            std::size_t operator()(const std::pair<T1, T2> &p) const {
                return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
            }
        };

        std::vector<std::unordered_map<std::pair<int, int>, std::vector<double>, pair_hash>> timers;
        std::unordered_map<std::pair<int, int>, double, pair_hash> cost_to_go;
        std::unordered_map<std::pair<int, int>, int, pair_hash> path_to_go;

        void set_timers();

        std::vector<double>
        dwell_time(std::pair<std::vector<int>, std::vector<double>> &con, std::vector<double> yi, int vi, int vni,
                   int i) const;


    };
}
#endif // DPAPPROX_H
