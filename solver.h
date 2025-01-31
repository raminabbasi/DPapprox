#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <algorithm>

constexpr double ZERO = 1e-9;
constexpr double INFTY = 1e20;

class Solver {
public:
    Solver(const std::vector<std::vector<double>>& v_rel, const std::vector<std::vector<int>>& v_feasible, double dt);
    void solve();
    double (*running_cost)(int vi, double ri, int i, double dt);
    double (*sort_key)(double x);
    std::pair<std::vector<int>, double> solution;
    static double simple_rounding(int vi, double ri, int i, double dt);

private:
    std::vector<std::vector<double>> v_rel;
    std::vector<std::vector<int>> v_feasible;
    double dt;

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
        }
    };

    std::unordered_map<std::pair<int, int>, double, pair_hash> cost_to_go;
    std::unordered_map<std::pair<int, int>, int, pair_hash> path_to_go;


};

#endif // SOLVER_H
