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

    std::vector<std::vector<vtype>> v_feasible;
    xtype x0;
    bool include_state{false};
    bool customize{false};

    std::function<std::vector<double>(const vtype&, const std::vector<double>&, int, double)> running_cost{running_cost_0};
    std::function<double(const std::vector<double>&)> sort_key{sort_key_0};
    std::function<xtype(const xtype&, const vtype&, int, double)> next_state_f{next_state_f_0};
    std::function<std::vector<double>(const xtype&, const std::vector<double>&, int, double)> dynamic_cost{dynamic_cost_0};

    std::vector<std::pair<std::vector<int>, std::vector<double>>>dwell_time_cons;
    std::vector<std::vector<double>> dwell_time_init{};

    static std::vector<double> running_cost_0(const vtype& vi, const std::vector<double>& ri, int, double){
        return std::vector<double>{std::abs(vi[0] - ri[0])};
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

    using KeyType = std::pair<vtype, int>;
    using CostMap = std::unordered_map<KeyType, std::vector<double> , pair_hash>;
    using PathMap = std::unordered_map<KeyType, vtype, pair_hash>;

    std::function<std::vector<double>(const vtype&, const vtype&, const std::vector<double>&, const PathMap&,
                                      const CostMap&, int, double)> custom_cost{custom_cost_0};

    static std::vector<double> custom_cost_0(const vtype& vni, const std::vector<double>& cost_nxt, const vtype& vi,
                                             const CostMap& cost_to_go, const PathMap& path_to_go, int i, double dt){
        return {0};
    };
};

#endif // CONFIG_H
