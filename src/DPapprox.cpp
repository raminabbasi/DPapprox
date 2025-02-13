#include "DPapprox.h"
#include "logger.h"

namespace DPapprox {

    Solver::Solver(const std::vector<std::vector<double>> &v_rel, const ProblemConfig &config):
        v_rel(v_rel),
        problem(config),
        next_state_f(next_state_0),
        running_cost(simple_rounding),
        sort_key([](const std::vector<double>& x) { return x.at(0); })
        {
        Log(INFO) << "Initializing Solver.";

        if (problem.N != static_cast<int>(v_rel[0].size())){
            throw std::runtime_error("Error: problem.N does not match v_rel[0].size().");
        }
        if (problem.nv != static_cast<int>(problem.v_feasible[0][0].size())){
            throw std::runtime_error("Error: problem.nv does not match problem.v_feasible[0][0].size().");
        }
        if (config.next_state_f) {
            next_state_f = config.next_state_f;
        }
        if (config.running_cost) {
            running_cost = config.running_cost;
        }
        if (config.sort_key) {
            sort_key = config.sort_key;
        }
        if (!config.dwell_time_cons.empty()) {
            dwell_time_cons = config.dwell_time_cons;
        }
    }

    void Solver::solve() {
        Log(INFO) << "Solving ...";
        set_timers();

        for (const ProblemConfig::vtype& v_0: problem.v_feasible[0]) {
            std::pair<ProblemConfig::vtype, int> v_ini = {v_0, 0};
            cost_to_go[v_ini] = running_cost(v_0, get_column(v_rel, 0), 0, problem.dt);
            if (problem.is_dynamic_cost) next_state[v_ini] = next_state_f(problem.x0, v_0, 0, problem.dt);

        }

        std::vector<double> cost{0};
        std::vector<double> v{0};
        std::vector<double> p{0};
        ProblemConfig::xtype xni{problem.x0};
        ProblemConfig::vtype v_end(0);
        std::vector<double> cost_end(0);
        Log(INFO) << "Starting the forward recursion";
        for (int i = 0; i < problem.N - 1; ++i) {
            for (const ProblemConfig::vtype& vni: problem.v_feasible[i + 1]) {
                std::pair<ProblemConfig::vtype, int> v_nxt = {vni, i + 1};

                std::vector<double> opt = {std::numeric_limits<double>::infinity()};
                std::vector<double> c = running_cost(vni, get_column(v_rel, i+1), i + 1, problem.dt);
                for (const ProblemConfig::vtype& vi: problem.v_feasible[i]) {
                    std::pair<ProblemConfig::vtype, int> v_now = {vi, i};


                    std::vector<std::vector<double>> dwell;
                    for (size_t k = 0; k < dwell_time_cons.size(); ++k) {
                        auto &con = dwell_time_cons[k];
                        auto &timer = timers[k];
                        dwell.push_back(dwell_time(con, timer[v_now], vi, vni, i));
                    }

                    bool violate_dwell = std::any_of(dwell.begin(), dwell.end(), [](const std::vector<double> &row) {
                        return std::find(row.begin(), row.end(), DWELL_FLAG) != row.end();
                    });

                    std::vector<double> d = {0};
                    if (violate_dwell) {
                        d = INFTY;
                    }

                    if (problem.is_dynamic_cost) {
                        ProblemConfig::xtype xni = next_state[v_now];
                        p = problem.dynamic_cost(xni, get_column(v_rel, i), i, problem.dt);
                    }

                    v = cost_to_go[v_now];

                    cost = (c + v + d + p);

                    if (sort_key(cost) < sort_key(opt)) {

                        opt = cost;
                        cost_to_go[v_nxt] = opt;
                        path_to_go[v_nxt] = vi;

                        if (problem.is_dynamic_cost) next_state[v_nxt] = next_state_f(xni, vni, i + 1, problem.dt);

                        for (size_t k = 0; k < dwell.size(); ++k) {
                            auto &timer = timers[k];
                            auto &dw = dwell[k];
                            timer[v_nxt] = dw;
                        }
                    }
                }
            }
        }

        Log(INFO) << "Finished the forward recursion";
        std::vector<std::pair<ProblemConfig::vtype, int>> keys;
        for (const ProblemConfig::vtype& val: problem.v_feasible[0]) {
            keys.emplace_back(val, problem.N - 1);
        }

        cost_end = INFTY;

        for (const auto &key: keys) {
            auto it = cost_to_go.find(key);
            if (it != cost_to_go.end()) {
                if (sort_key(it->second) < sort_key(cost_end)) {
                    cost_end = it->second;
                    v_end = key.first;
                }
            }
        }

        std::vector<ProblemConfig::vtype> optimum_path = {v_end};
        Log(INFO) << "Starting the backward recursion";
        for (auto i = problem.N - 1; i > 0; --i) {
            ProblemConfig::vtype next_vi = path_to_go[{optimum_path[0], i}];
            optimum_path.insert(optimum_path.begin(), next_vi);
        }
        Log(INFO) << "Finished the backward recursion";
        solution.first = optimum_path;
        solution.second = cost_end.at(0);

    }

    std::vector<double> Solver::simple_rounding(const ProblemConfig::vtype& vi,
                                                const std::vector<double>& ri,
                                                int /*i*/,
                                                double /*problem.dt*/) {
        double sum;
        sum = std::pow(vi[0] - ri[0], 2);
        return {std::sqrt(sum)};
    }

    std::vector<double> Solver::next_state_0(const ProblemConfig::xtype& xi,
                                     const ProblemConfig::vtype& /*vi*/,
                                     int /*i*/,
                                     double /*problem.dt*/){
        return xi;
    }

    void Solver::set_timers() {
        if (dwell_time_init.empty()) {
            std::size_t num_cons = dwell_time_cons.size();
            dwell_time_init.assign(num_cons, std::vector<double>(problem.nv, 0.0));
        }
        for (std::vector<double> &timer_0: dwell_time_init) {
            std::unordered_map<std::pair<ProblemConfig::vtype, int>, std::vector<double>, pair_hash> timer;
            for (const ProblemConfig::vtype& v_0: problem.v_feasible[0]) {
                timer[{v_0, 0}] = timer_0;
            }
            timers.push_back(timer);
        }
    }

    std::vector<double> Solver::dwell_time(std::pair<std::vector<int>, std::vector<double>> &con,
                                           std::vector<double> yi, const ProblemConfig::vtype& vi,
                                           const ProblemConfig::vtype& vni, int i) const {

        for (double &y: yi) {
            y -= problem.dt;
        }
        std::vector<double> yni = yi;
        for (size_t idx = 0; idx < yi.size(); ++idx) {
            if (yi[idx] <= 0) {
                yni[idx] = 0;
            }
            if (vni[idx] != vi[idx]) {
                if ((yi[idx] > 0) && (vi[idx] == con.first.back())) {
                    yni[idx] = DWELL_FLAG;
                } else {
                    auto it = std::find(con.first.begin(), con.first.end(), vni[idx]);
                    if (it != con.first.end()) {
                        size_t vni_idx = std::distance(con.first.begin(), it);
                        if (vni_idx == 0) {
                            yni[idx] = con.second[idx] + ZERO;
                        } else if (con.first[vni_idx - 1] != vi[idx]) {
                            yni[idx] = 0;
                        }
                    }
                }
            }
        }
        return yni;
    }

    std::vector<double> Solver::get_column(const std::vector<std::vector<double>> &v, size_t col_index) {
        std::vector<double> column;
        for (const auto& row : v) {
            if (col_index < row.size()) {
                column.push_back({row[col_index]});
            }
        }
        return column;
    }

}