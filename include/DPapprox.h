#ifndef DPAPPROX_H
#define DPAPPROX_H

#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include "config.h"
#include "io_utils.h"
#include "vector_ops.h"
#include "logger.h"

namespace  DPapprox {

    constexpr double ZERO = 1e-9;
    const std::vector<double> INFTY {1e20};
    constexpr double DWELL_FLAG = -2;

    class Solver {

    public:
        Solver(const std::vector<std::vector<double>> &v_rel, const ProblemConfig &config);

        void solve();
        std::pair<std::vector<ProblemConfig::vtype>, double> solution;

        static std::vector<double> get_column(const std::vector<std::vector<double>>& v, size_t col_index);

    private:
        ProblemConfig dp;
        std::vector<std::vector<double>> v_rel;

        struct pair_hash {
            template<class T1, class T2>
            std::size_t operator()(const std::pair<T1, T2>& p) const {
                return hash_vector(p.first) ^ std::hash<T2>()(p.second);
            }

        private:
            static std::size_t hash_vector(const std::vector<double>& v) {
                std::size_t seed = v.size();
                for (double num : v) {
                    seed ^= std::hash<double>()(num) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                }
                return seed;
            }
        };

        // Define type alias for readability
        using KeyType = std::pair<ProblemConfig::vtype, int>;
        using CostMap = std::unordered_map<KeyType, std::vector<double>, pair_hash>;
        using PathMap = std::unordered_map<KeyType, ProblemConfig::vtype, pair_hash>;

        std::vector<CostMap> timers;
        CostMap cost_to_go;
        PathMap path_to_go;
        PathMap next_state;

        void set_timers();

        std::vector<double>
        dwell_time(std::pair<std::vector<int>,
                   std::vector<double>> &con,
                   std::vector<double> yi,
                   const ProblemConfig::vtype& vi,
                   const ProblemConfig::vtype& vni, int i) const;
    };

}
#endif // DPAPPROX_H
