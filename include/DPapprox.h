#ifndef DPAPPROX_H
#define DPAPPROX_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include "config.h"
#include "io_utils.h"
#include "vector_ops.h"
#include "./logger.h"

namespace  DPapprox {

constexpr double EPSILON = 1e-9;
const std::vector<double> INFTY {1e20};
constexpr double DWELL_FLAG = -2;

struct Solution {
    std::vector<ProblemConfig::vtype> optimum_path;
    std::unordered_map<std::pair<ProblemConfig::vtype, int>, std::vector<double> , ProblemConfig::pair_hash> optimum_cost;
    std::vector<ProblemConfig::xtype> optimum_state;
    double f;
    bool success;
};

class Solver {

public:
    Solver(const std::vector<std::vector<double>> &v_rel, const ProblemConfig &config);
    void solve();
    Solution solution;

private:
    ProblemConfig dp;
    std::vector<std::vector<double>> v_rel;

    // Define type alias for readability
    using KeyType = std::pair<ProblemConfig::vtype, int>;
    using CostMap = std::unordered_map<KeyType, std::vector<double> , ProblemConfig::pair_hash>;
    using PathMap = std::unordered_map<KeyType, ProblemConfig::vtype, ProblemConfig::pair_hash>;

    std::vector<CostMap> timers;
    CostMap cost_to_go;
    PathMap path_to_go;
    PathMap next_state;

    void set_timers();

    std::vector<double>
    dwell_time(const std::pair<std::vector<int>, std::vector<double>> &con,
               std::vector<double> yi,
               const ProblemConfig::vtype& vi,
               const ProblemConfig::vtype& vni, int i) const;
};

}
#endif // DPAPPROX_H
