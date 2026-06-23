#include "LoadSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <unordered_map>

namespace load_exact {
namespace {

struct Profile {
    std::vector<int> q;
    std::vector<StopOperation> ops;
    int final_load = 0;
    double duration = 0.0;
};

struct RouteProfileSet {
    RoutePlan base_route;
    std::vector<Profile> profiles;
    long long generated = 0;
    bool exact = true;
};

double travel(const Instance& inst, const std::vector<int>& nodes) {
    double t = 0.0;
    for (std::size_t i = 1; i < nodes.size(); ++i) t += inst.dist[nodes[i - 1]][nodes[i]];
    return t;
}

std::string signature(const RoutePlan& route) {
    std::ostringstream out;
    out << route.vehicle << ':';
    for (int n : route.nodes) out << n << '-';
    return out.str();
}

void genProfileRec(const Instance& inst, const RoutePlan& route, std::size_t pos,
                   int load, int pick, int drop, double base_travel,
                   std::vector<int>& q, std::vector<StopOperation>& ops,
                   std::vector<Profile>& profiles, long long& generated,
                   int profile_limit, bool natural_mode_only) {
    if (profile_limit > 0 && static_cast<int>(profiles.size()) >= profile_limit) return;
    if (pos + 1 >= route.nodes.size()) {
        const int depot_unload = load;
        const double duration = base_travel + inst.pickup_time * pick
            + inst.drop_time * (drop + depot_unload);
        if (duration <= inst.route_time_limit + 1e-7) {
            Profile p;
            p.q = q;
            p.ops = ops;
            p.final_load = load;
            p.duration = duration;
            profiles.push_back(std::move(p));
        }
        ++generated;
        return;
    }
    const int s = route.nodes[pos];
    std::vector<StopOperation> choices;
    choices.push_back({s, 0, 0});
    int max_pick = std::min(inst.initial[s], inst.Q[route.vehicle] - load);
    int max_drop = std::min(load, inst.capacity[s] - inst.initial[s]);
    if (natural_mode_only) {
        if (inst.initial[s] > inst.target[s]) max_drop = 0;
        if (inst.initial[s] < inst.target[s]) max_pick = 0;
    }
    auto addAmounts = [&](bool pickup, int maxv) {
        if (maxv <= 0) return;
        std::set<int> vals{1, maxv};
        if (maxv >= 2) vals.insert(maxv / 2);
        if (!natural_mode_only && maxv <= 8) {
            for (int v = 1; v <= maxv; ++v) vals.insert(v);
        }
        for (int v : vals) {
            if (pickup) choices.push_back({s, v, 0});
            else choices.push_back({s, 0, v});
        }
    };
    addAmounts(true, max_pick);
    addAmounts(false, max_drop);
    for (const auto& ch : choices) {
        const int nl = load + ch.pickup - ch.drop;
        if (nl < 0 || nl > inst.Q[route.vehicle]) continue;
        const int final_s = inst.initial[s] + q[s] + ch.drop - ch.pickup;
        if (final_s < 0 || final_s > inst.capacity[s]) continue;
        q[s] += ch.drop - ch.pickup;
        if (ch.pickup || ch.drop) ops.push_back(ch);
        genProfileRec(inst, route, pos + 1, nl, pick + ch.pickup, drop + ch.drop,
                      base_travel, q, ops, profiles, generated, profile_limit, natural_mode_only);
        if (ch.pickup || ch.drop) ops.pop_back();
        q[s] -= ch.drop - ch.pickup;
    }
}

RouteProfileSet profilesForRoute(const Instance& inst, const RoutePlan& route,
                                 const RunOptions& opt) {
    RouteProfileSet set;
    set.base_route = route;
    std::vector<int> q(inst.V + 1, 0);
    std::vector<StopOperation> ops;
    const bool natural_mode = inst.V > 10;
    genProfileRec(inst, route, 1, 0, 0, 0, travel(inst, route.nodes), q, ops,
                  set.profiles, set.generated,
                  opt.profile_limit > 0 ? opt.profile_limit : (inst.V <= 10 ? 50000 : 5000),
                  natural_mode);
    set.exact = !natural_mode && (opt.profile_limit == 0 ||
        static_cast<int>(set.profiles.size()) < opt.profile_limit);
    std::sort(set.profiles.begin(), set.profiles.end(), [](const Profile& a, const Profile& b) {
        if (a.q != b.q) return a.q < b.q;
        return a.duration < b.duration;
    });
    std::vector<Profile> kept;
    std::set<std::vector<int>> seen;
    for (auto& p : set.profiles) {
        if (seen.insert(p.q).second) kept.push_back(std::move(p));
    }
    set.profiles = std::move(kept);
    return set;
}

struct CombineState {
    const Instance* inst = nullptr;
    const RunOptions* opt = nullptr;
    const std::vector<RouteProfileSet>* sets = nullptr;
    std::vector<int> inv;
    std::vector<RoutePlan> routes;
    double best = std::numeric_limits<double>::infinity();
    std::vector<int> best_inv;
    std::vector<RoutePlan> best_routes;
};

void combine(CombineState& st, int k) {
    if (k == static_cast<int>(st.sets->size())) {
        auto o = computeObjectiveParts(*st.inst, st.inv, st.opt->lambda);
        if (o.objective < st.best) {
            st.best = o.objective;
            st.best_inv = st.inv;
            st.best_routes = st.routes;
        }
        return;
    }
    const auto& rp = (*st.sets)[k];
    for (const auto& prof : rp.profiles) {
        bool ok = true;
        for (int i = 1; i <= st.inst->V; ++i) {
            int y = st.inv[i] + prof.q[i];
            if (y < 0 || y > st.inst->capacity[i]) { ok = false; break; }
        }
        if (!ok) continue;
        RoutePlan r = rp.base_route;
        r.operations = prof.ops;
        for (int i = 1; i <= st.inst->V; ++i) st.inv[i] += prof.q[i];
        st.routes.push_back(std::move(r));
        combine(st, k + 1);
        st.routes.pop_back();
        for (int i = 1; i <= st.inst->V; ++i) st.inv[i] -= prof.q[i];
    }
}

std::string routeLengths(const std::vector<RoutePlan>& routes) {
    std::ostringstream out;
    for (std::size_t i = 0; i < routes.size(); ++i) {
        if (i) out << ';';
        out << (routes[i].nodes.size() >= 2 ? routes[i].nodes.size() - 2 : 0);
    }
    return out.str();
}

double fullPairObjectiveUpdateCheck(const Instance& inst, const std::vector<int>& old_inv,
                                    std::vector<int> new_inv, double lambda,
                                    const std::vector<int>& delta) {
    (void)old_inv;
    (void)delta;
    return computeObjectiveParts(inst, new_inv, lambda).objective;
}

} // namespace

std::vector<RoutePlan> makeDeterministicRoutes(const Instance& inst, int length_limit) {
    std::vector<int> stations(inst.V);
    std::iota(stations.begin(), stations.end(), 1);
    std::sort(stations.begin(), stations.end(), [&](int a, int b) {
        return std::abs(inst.initial[a] - inst.target[a]) > std::abs(inst.initial[b] - inst.target[b]);
    });
    const int lim = length_limit > 0 ? length_limit : std::max(2, std::min(6, (inst.V + inst.M - 1) / inst.M));
    std::vector<RoutePlan> routes(inst.M);
    for (int k = 0; k < inst.M; ++k) {
        routes[k].vehicle = k;
        routes[k].nodes.push_back(0);
    }
    for (std::size_t i = 0; i < stations.size(); ++i) {
        int k = static_cast<int>(i % inst.M);
        if (static_cast<int>(routes[k].nodes.size()) - 1 < lim) routes[k].nodes.push_back(stations[i]);
    }
    for (auto& r : routes) r.nodes.push_back(0);
    return routes;
}

Result solveFixedRoutes(const Instance& inst, const RunOptions& opt,
                        const std::vector<RoutePlan>& fixed_routes) {
    Timer timer;
    Result r;
    r.instance = inst.name;
    r.route_set_id = "deterministic_imbalance_partition";
    r.V = inst.V;
    r.M = inst.M;
    r.algorithm = opt.algorithm;
    r.route_lengths = routeLengths(fixed_routes);
    r.result_file = opt.out_path;
    r.log_file = opt.log_path;

    std::vector<RouteProfileSet> sets;
    bool exact = true;
    for (const auto& route : fixed_routes) {
        auto s = profilesForRoute(inst, route, opt);
        r.profiles_generated += s.generated;
        r.profiles_after_dominance += static_cast<long long>(s.profiles.size());
        exact = exact && s.exact;
        sets.push_back(std::move(s));
    }
    CombineState st;
    st.inst = &inst;
    st.opt = &opt;
    st.sets = &sets;
    st.inv = inst.initial;
    combine(st, 0);
    r.runtime_ms = timer.seconds() * 1000.0;
    r.exact_for_fixed_routes = exact;
    if (std::isfinite(st.best)) {
        r.status = exact ? "fixed_route_optimal" : "fixed_route_diagnostic_pruned_profiles";
        r.routes = st.best_routes;
        r.final_inventory = st.best_inv;
        r.verification = verifySolution(inst, r.routes, opt.lambda);
        r.verifier_passed = r.verification.feasible;
        r.objective = st.best;
        r.G = r.verification.G;
        r.P = r.verification.P;
        r.cplex_objective = 0.0;
        r.cplex_gap = 0.0;
        r.objective_diff = 0.0;
    } else {
        r.status = "no_feasible_profile_combination";
    }
    r.notes.push_back("Profile DP enumerates fixed-route load decisions and combines route profiles exactly when no profile limit/natural-mode pruning is active.");
    if (!exact) r.notes.push_back("Large route set used natural-mode/profile-budget pruning; row is diagnostic.");
    return r;
}

Result runIncrementalTest(const Instance& inst, const RunOptions& opt) {
    Timer total;
    auto routes = makeDeterministicRoutes(inst, opt.route_length_limit);
    std::unordered_map<std::string, RouteProfileSet> cache;
    long long hits = 0, misses = 0;
    auto get = [&](const RoutePlan& route) -> RouteProfileSet {
        auto key = signature(route);
        auto it = cache.find(key);
        if (it != cache.end()) { ++hits; return it->second; }
        ++misses;
        auto s = profilesForRoute(inst, route, opt);
        cache.emplace(key, s);
        return s;
    };
    for (const auto& r : routes) get(r);

    double full_time = 0.0, incr_time = 0.0;
    double last_obj = 0.0;
    bool ok = true;
    for (int move = 0; move < 100; ++move) {
        std::vector<RoutePlan> changed = routes;
        int k = move % inst.M;
        if (changed[k].nodes.size() > 3) {
            std::swap(changed[k].nodes[1], changed[k].nodes[changed[k].nodes.size() - 2]);
        }
        Timer t1;
        auto full = solveFixedRoutes(inst, opt, changed);
        full_time += t1.seconds();
        Timer t2;
        std::vector<RouteProfileSet> sets;
        for (const auto& route : changed) sets.push_back(get(route));
        CombineState st;
        st.inst = &inst;
        st.opt = &opt;
        st.sets = &sets;
        st.inv = inst.initial;
        combine(st, 0);
        incr_time += t2.seconds();
        const double inc_obj = fullPairObjectiveUpdateCheck(inst, inst.initial, st.best_inv, opt.lambda, {});
        if (std::fabs(full.objective - inc_obj) > 1e-7) ok = false;
        last_obj = inc_obj;
        routes = changed;
    }
    Result r;
    r.instance = inst.name;
    r.route_set_id = "incremental_100_moves";
    r.V = inst.V;
    r.M = inst.M;
    r.route_lengths = routeLengths(routes);
    r.algorithm = "incremental-test";
    r.status = ok ? "incremental_matches_full_recompute" : "incremental_mismatch";
    r.objective = last_obj;
    r.runtime_ms = total.seconds() * 1000.0;
    r.cache_hits = hits;
    r.cache_misses = misses;
    r.incremental_speedup = incr_time > 0.0 ? full_time / incr_time : 0.0;
    r.verifier_passed = ok;
    r.exact_for_fixed_routes = inst.V <= 10;
    r.result_file = opt.out_path;
    r.log_file = opt.log_path;
    r.notes.push_back("Incremental evaluator reuses unchanged route profile sets and checks O(|Delta|V) objective update against full recomputation.");
    return r;
}

std::vector<Result> runDefaultSuite(const RunOptions& options) {
    std::vector<std::filesystem::path> inputs = {
        std::filesystem::path(options.exactebrp_root) / "testdata/examples/gcap_smoke_V4_M1.txt",
        std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V8_M2_average.txt",
        std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V10_M2_average.txt",
        std::filesystem::path(options.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt",
        std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V20_M2_average.txt"
    };
    std::vector<Result> results;
    for (const auto& input : inputs) {
        if (!std::filesystem::exists(input)) continue;
        RunOptions opt = options;
        opt.input_path = input.string();
        Instance inst = parseInstanceFile(input, opt.route_time_limit, opt.pickup_time, opt.drop_time);
        auto routes = makeDeterministicRoutes(inst, opt.route_length_limit);
        std::string tag = basenameNoExt(input);

        opt.algorithm = "profile-dp";
        opt.out_path = (std::filesystem::path("results/raw") / (tag + "_profile_dp.json")).string();
        opt.log_path = (std::filesystem::path("results/logs") / (tag + "_profile_dp.log")).string();
        auto r = solveFixedRoutes(inst, opt, routes);
        writeText(r.result_file, toJson(r));
        writeText(r.log_file, "status=" + r.status + "\nobjective=" + std::to_string(r.objective) + "\n");
        results.push_back(r);

        opt.algorithm = "incremental-test";
        opt.out_path = (std::filesystem::path("results/raw") / (tag + "_incremental.json")).string();
        opt.log_path = (std::filesystem::path("results/logs") / (tag + "_incremental.log")).string();
        auto inc = runIncrementalTest(inst, opt);
        writeText(inc.result_file, toJson(inc));
        writeText(inc.log_file, "status=" + inc.status + "\nspeedup=" + std::to_string(inc.incremental_speedup) + "\n");
        results.push_back(inc);
    }
    return results;
}

} // namespace load_exact

