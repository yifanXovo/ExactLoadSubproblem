#include "LoadSolver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <regex>
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
    bool truncated = false;
};

struct Candidate {
    double objective = std::numeric_limits<double>::infinity();
    double G = 0.0;
    double P = 0.0;
    std::vector<int> inv;
    std::vector<RoutePlan> routes;
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

std::string routeLengths(const std::vector<RoutePlan>& routes) {
    std::ostringstream out;
    for (std::size_t i = 0; i < routes.size(); ++i) {
        if (i) out << ';';
        out << (routes[i].nodes.size() >= 2 ? routes[i].nodes.size() - 2 : 0);
    }
    return out.str();
}

void genProfileRec(const Instance& inst, const RoutePlan& route, const RunOptions& opt,
                   std::size_t pos, int load, int pick, int drop, double base_travel,
                   std::vector<int>& q, std::vector<StopOperation>& ops,
                   RouteProfileSet& set) {
    if (opt.profile_limit > 0 && static_cast<int>(set.profiles.size()) >= opt.profile_limit) {
        set.truncated = true;
        return;
    }
    if (pos + 1 >= route.nodes.size()) {
        const int depot_unload = load;
        const double duration = base_travel + inst.pickup_time * pick
            + inst.drop_time * (drop + depot_unload);
        ++set.generated;
        if (duration <= inst.route_time_limit + 1e-7) {
            set.profiles.push_back(Profile{q, ops, load, duration});
        }
        return;
    }

    const int s = route.nodes[pos];
    const int max_pick_raw = std::min(inst.initial[s], inst.Q[route.vehicle] - load);
    const int max_drop_raw = std::min(load, inst.capacity[s] - inst.initial[s]);
    int max_pick = max_pick_raw;
    int max_drop = max_drop_raw;
    if (opt.allow_natural_mode_pruning) {
        if (inst.initial[s] > inst.target[s]) max_drop = 0;
        if (inst.initial[s] < inst.target[s]) max_pick = 0;
    }

    std::vector<StopOperation> choices;
    choices.push_back({s, 0, 0});
    auto addAmounts = [&](bool pickup, int maxv) {
        if (maxv <= 0) return;
        if (opt.profile_exact) {
            for (int v = 1; v <= maxv; ++v) {
                choices.push_back(pickup ? StopOperation{s, v, 0} : StopOperation{s, 0, v});
            }
        } else {
            std::set<int> vals{1, maxv};
            if (maxv >= 2) vals.insert(maxv / 2);
            for (int v : vals) {
                choices.push_back(pickup ? StopOperation{s, v, 0} : StopOperation{s, 0, v});
            }
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
        genProfileRec(inst, route, opt, pos + 1, nl, pick + ch.pickup, drop + ch.drop,
                      base_travel, q, ops, set);
        if (ch.pickup || ch.drop) ops.pop_back();
        q[s] -= ch.drop - ch.pickup;
        if (set.truncated) return;
    }
}

RouteProfileSet profilesForRoute(const Instance& inst, const RoutePlan& route,
                                 const RunOptions& opt) {
    RouteProfileSet set;
    set.base_route = route;
    std::vector<int> q(inst.V + 1, 0);
    std::vector<StopOperation> ops;
    genProfileRec(inst, route, opt, 1, 0, 0, 0, travel(inst, route.nodes), q, ops, set);
    set.exact = opt.profile_exact && !opt.allow_natural_mode_pruning && !set.truncated;

    std::sort(set.profiles.begin(), set.profiles.end(), [](const Profile& a, const Profile& b) {
        if (a.q != b.q) return a.q < b.q;
        if (a.final_load != b.final_load) return a.final_load < b.final_load;
        return a.duration < b.duration;
    });
    std::vector<Profile> kept;
    std::map<std::pair<std::vector<int>, int>, double> best_duration;
    for (auto& p : set.profiles) {
        auto key = std::make_pair(p.q, p.final_load);
        auto it = best_duration.find(key);
        if (it == best_duration.end() || p.duration + 1e-9 < it->second) {
            best_duration[key] = p.duration;
            kept.push_back(std::move(p));
        }
    }
    set.profiles = std::move(kept);
    return set;
}

struct CombineState {
    const Instance* inst = nullptr;
    const RunOptions* opt = nullptr;
    const Timer* timer = nullptr;
    const std::vector<RouteProfileSet>* sets = nullptr;
    std::vector<int> inv;
    std::vector<RoutePlan> routes;
    Candidate best;
    long long nodes = 0;
    std::vector<Candidate>* all_candidates = nullptr;
    long long candidate_limit = 100000;
    bool aborted = false;
};

void combine(CombineState& st, int k) {
    if (st.aborted) return;
    ++st.nodes;
    if (st.opt && st.opt->profile_bpc_max_nodes > 0 &&
        st.nodes > st.opt->profile_bpc_max_nodes) {
        st.aborted = true;
        return;
    }
    if (st.timer && st.opt && st.opt->profile_bpc_time_limit > 0.0 &&
        st.timer->seconds() > st.opt->profile_bpc_time_limit) {
        st.aborted = true;
        return;
    }
    if (k == static_cast<int>(st.sets->size())) {
        auto obj = computeObjectiveParts(*st.inst, st.inv, st.opt->lambda);
        Candidate c;
        c.objective = obj.objective;
        c.G = obj.G;
        c.P = obj.P;
        c.inv = st.inv;
        c.routes = st.routes;
        if (c.objective < st.best.objective) st.best = c;
        if (st.all_candidates &&
            static_cast<long long>(st.all_candidates->size()) < st.candidate_limit) {
            st.all_candidates->push_back(std::move(c));
        }
        return;
    }

    const auto& rp = (*st.sets)[k];
    for (const auto& prof : rp.profiles) {
        bool ok = true;
        for (int i = 1; i <= st.inst->V; ++i) {
            const int y = st.inv[i] + prof.q[i];
            if (y < 0 || y > st.inst->capacity[i]) { ok = false; break; }
        }
        if (!ok) continue;
        RoutePlan route = rp.base_route;
        route.operations = prof.ops;
        for (int i = 1; i <= st.inst->V; ++i) st.inv[i] += prof.q[i];
        st.routes.push_back(std::move(route));
        combine(st, k + 1);
        st.routes.pop_back();
        for (int i = 1; i <= st.inst->V; ++i) st.inv[i] -= prof.q[i];
    }
}

std::vector<RouteProfileSet> buildProfileSets(const Instance& inst,
                                              const RunOptions& opt,
                                              const std::vector<RoutePlan>& routes,
                                              Result& r) {
    std::vector<RouteProfileSet> sets;
    for (const auto& route : routes) {
        auto s = profilesForRoute(inst, route, opt);
        r.profiles_generated += s.generated;
        r.profiles_after_dominance += static_cast<long long>(s.profiles.size());
        r.exact_for_fixed_routes = r.exact_for_fixed_routes && s.exact;
        sets.push_back(std::move(s));
    }
    return sets;
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '\\') out += "\\\\";
        else if (c == '"') out += "\\\"";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else out += c;
    }
    return out;
}

void writeRouteJson(const std::filesystem::path& path,
                    const std::vector<RoutePlan>& routes,
                    const std::string& route_set_id) {
    std::ostringstream out;
    out << "{\n  \"route_set_id\": \"" << jsonEscape(route_set_id) << "\",\n  \"routes\": [\n";
    for (std::size_t r = 0; r < routes.size(); ++r) {
        if (r) out << ",\n";
        out << "    {\"vehicle\": " << routes[r].vehicle << ", \"nodes\": [";
        for (std::size_t i = 0; i < routes[r].nodes.size(); ++i) {
            if (i) out << ", ";
            out << routes[r].nodes[i];
        }
        out << "]}";
    }
    out << "\n  ]\n}\n";
    writeText(path, out.str());
}

std::vector<RoutePlan> readRouteJson(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open route JSON: " + path.string());
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::regex route_re(R"(\{\s*"vehicle"\s*:\s*(\d+)\s*,\s*"nodes"\s*:\s*\[([^\]]*)\])");
    std::regex num_re(R"(-?\d+)");
    std::vector<RoutePlan> routes;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), route_re);
         it != std::sregex_iterator(); ++it) {
        RoutePlan r;
        r.vehicle = std::stoi((*it)[1].str());
        std::string payload = (*it)[2].str();
        for (auto nt = std::sregex_iterator(payload.begin(), payload.end(), num_re);
             nt != std::sregex_iterator(); ++nt) {
            r.nodes.push_back(std::stoi(nt->str()));
        }
        routes.push_back(std::move(r));
    }
    if (routes.empty()) throw std::runtime_error("No routes parsed from " + path.string());
    return routes;
}

bool commandExists(const std::string& exe) {
    return std::system(("where " + exe + " >nul 2>nul").c_str()) == 0;
}

std::string cplexScriptPath(const std::filesystem::path& lp) {
    return lp.parent_path().append("run.cplex").string();
}

std::string plusTerms(const std::vector<std::string>& terms) {
    std::ostringstream out;
    for (std::size_t i = 0; i < terms.size(); ++i) {
        out << " + " << terms[i];
    }
    return out.str();
}

std::string sumTerms(const std::vector<std::string>& terms) {
    if (terms.empty()) return " 0";
    std::ostringstream out;
    for (std::size_t i = 0; i < terms.size(); ++i) {
        out << (i ? " + " : " ") << terms[i];
    }
    return out.str();
}

std::vector<double> ratioSumValues(const Instance& inst,
                                   const std::vector<std::vector<int>>& domains,
                                   std::size_t max_values,
                                   bool& truncated) {
    std::map<std::string, double> vals;
    vals["0"] = 0.0;
    for (int i = 1; i <= inst.V; ++i) {
        std::map<std::string, double> next;
        for (const auto& kv : vals) {
            for (int y : domains[i]) {
                const double v = kv.second + static_cast<double>(y) / inst.target[i];
                std::ostringstream key;
                key << std::fixed << std::setprecision(9) << v;
                next[key.str()] = v;
                if (next.size() > max_values) {
                    truncated = true;
                    return {};
                }
            }
        }
        vals.swap(next);
    }
    std::vector<double> out;
    for (const auto& kv : vals) out.push_back(kv.second);
    std::sort(out.begin(), out.end());
    return out;
}

double hMaxBound(const Instance& inst) {
    double hmax = 0.0;
    for (int i = 1; i <= inst.V; ++i) {
        const double ai = static_cast<double>(inst.capacity[i]) / inst.target[i];
        for (int j = i + 1; j <= inst.V; ++j) {
            const double aj = static_cast<double>(inst.capacity[j]) / inst.target[j];
            hmax += std::max(std::fabs(ai), std::fabs(aj));
        }
    }
    return std::max(1.0, hmax);
}

std::map<std::string, double> parseCplexVariables(const std::filesystem::path& sol) {
    std::ifstream in(sol);
    if (!in) return {};
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::regex var_re(R"(<variable\s+name=\"([^\"]+)\"\s+index=\"[^\"]*\"\s+value=\"([^\"]+)\")");
    std::map<std::string, double> vars;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), var_re);
         it != std::sregex_iterator(); ++it) {
        vars[(*it)[1].str()] = std::stod((*it)[2].str());
    }
    return vars;
}

std::string parseSolutionStatus(const std::filesystem::path& sol) {
    std::ifstream in(sol);
    if (!in) return "no_solution_file";
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::smatch m;
    std::regex status_re(R"(solutionStatusString=\"([^\"]+)\")");
    return std::regex_search(text, m, status_re) ? m[1].str() : "unknown_solution_status";
}

double parseSolutionObjective(const std::filesystem::path& sol) {
    std::ifstream in(sol);
    if (!in) return std::numeric_limits<double>::infinity();
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::smatch m;
    std::regex obj_re(R"(objectiveValue=\"([^\"]+)\")");
    return std::regex_search(text, m, obj_re) ? std::stod(m[1].str())
                                             : std::numeric_limits<double>::infinity();
}

bool writeAndRunTrueFixedRouteMilp(const Instance& inst, const RunOptions& opt,
                                   const std::vector<RoutePlan>& fixed_routes,
                                   Result& r) {
    if (!commandExists("cplex")) {
        r.cplex_status = "cplex_not_available";
        r.status = "cplex_true_milp_not_run";
        r.cplex_gap = 1.0;
        r.notes.push_back("CPLEX executable not found; true fixed-route MILP was not run.");
        return false;
    }
    std::vector<int> visit_count(inst.V + 1, 0);
    for (const auto& route : fixed_routes) {
        for (std::size_t t = 1; t + 1 < route.nodes.size(); ++t) {
            if (route.nodes[t] < 1 || route.nodes[t] > inst.V) {
                r.cplex_status = "invalid_fixed_route_station";
                r.status = "invalid_fixed_route_station";
                r.cplex_gap = 1.0;
                return false;
            }
            visit_count[route.nodes[t]]++;
        }
    }
    if (!opt.allow_duplicate_stations) {
        for (int i = 1; i <= inst.V; ++i) {
            if (visit_count[i] > 1) {
                r.cplex_status = "invalid_fixed_routes_duplicate_station";
                r.status = "invalid_fixed_routes_duplicate_station";
                r.cplex_gap = 1.0;
                r.notes.push_back("Fixed route set violates station-disjointness.");
                return false;
            }
        }
    }
    std::vector<std::vector<int>> domains(inst.V + 1);
    for (int i = 1; i <= inst.V; ++i) {
        if (visit_count[i] == 0) domains[i].push_back(inst.initial[i]);
        else for (int y = 0; y <= inst.capacity[i]; ++y) domains[i].push_back(y);
    }
    bool s_truncated = false;
    auto s_values = ratioSumValues(inst, domains, 200000, s_truncated);
    if (s_truncated || s_values.empty()) {
        r.cplex_status = "cplex_milp_ratio_sum_selector_limit";
        r.status = "cplex_true_milp_not_closed";
        r.cplex_gap = 1.0;
        r.exact_for_fixed_routes = false;
        r.notes.push_back("Exact ratio-sum selector set exceeded the configured safety limit.");
        return false;
    }

    std::filesystem::path root = "results/round3_paper_exact_loading";
    const std::filesystem::path dir = root / "cplex" / r.route_set_id;
    std::filesystem::create_directories(dir);
    const auto lp = dir / "fixed_route_true_milp.lp";
    const auto sol = dir / "solution.sol";
    const auto log = dir / "cplex.log";
    auto pvar = [](int k, int t) { return "p_" + std::to_string(k) + "_" + std::to_string(t); };
    auto dvar = [](int k, int t) { return "d_" + std::to_string(k) + "_" + std::to_string(t); };
    auto lvar = [](int k, int t) { return "L_" + std::to_string(k) + "_" + std::to_string(t); };
    auto uvar = [](int k) { return "u_" + std::to_string(k); };
    struct StopRef { int k; int t; int station; };
    std::vector<std::vector<StopRef>> by_vehicle(inst.M);
    std::vector<std::vector<std::string>> p_terms(inst.V + 1), d_terms(inst.V + 1);
    std::vector<std::string> bounds, generals, binaries;
    for (const auto& route : fixed_routes) {
        for (std::size_t pos = 1; pos + 1 < route.nodes.size(); ++pos) {
            StopRef ref{route.vehicle, static_cast<int>(pos), route.nodes[pos]};
            by_vehicle[ref.k].push_back(ref);
            p_terms[ref.station].push_back(pvar(ref.k, ref.t));
            d_terms[ref.station].push_back(dvar(ref.k, ref.t));
            generals.push_back(pvar(ref.k, ref.t));
            generals.push_back(dvar(ref.k, ref.t));
            generals.push_back(lvar(ref.k, ref.t));
            bounds.push_back(" 0 <= " + pvar(ref.k, ref.t) + " <= " + std::to_string(inst.initial[ref.station]));
            bounds.push_back(" 0 <= " + dvar(ref.k, ref.t) + " <= " + std::to_string(inst.capacity[ref.station] - inst.initial[ref.station]));
            bounds.push_back(" 0 <= " + lvar(ref.k, ref.t) + " <= " + std::to_string(inst.Q[ref.k]));
        }
    }
    for (int k = 0; k < inst.M; ++k) {
        generals.push_back(uvar(k));
        bounds.push_back(" 0 <= " + uvar(k) + " <= " + std::to_string(inst.Q[k]));
    }
    const double hmax = hMaxBound(inst);
    const double smax = std::max(1.0, *std::max_element(s_values.begin(), s_values.end()));
    std::ostringstream model;
    model << std::setprecision(12);
    model << "Minimize\n obj:";
    for (std::size_t m = 0; m < s_values.size(); ++m) {
        const double coeff = s_values[m] > 1e-12 ? 1.0 / (static_cast<double>(inst.V) * s_values[m]) : 0.0;
        model << (m ? " + " : " ") << coeff << " WH_" << m;
    }
    model << " + " << opt.lambda << " P\nSubject To\n";
    int c = 0;
    auto cname = [&]() { return " c" + std::to_string(++c) + ":"; };
    for (int k = 0; k < inst.M; ++k) {
        const auto& refs = by_vehicle[k];
        for (std::size_t idx = 0; idx < refs.size(); ++idx) {
            const auto& ref = refs[idx];
            model << cname() << " " << lvar(ref.k, ref.t);
            if (idx > 0) model << " - " << lvar(ref.k, refs[idx - 1].t);
            model << " - " << pvar(ref.k, ref.t) << " + " << dvar(ref.k, ref.t) << " = 0\n";
        }
        model << cname() << " " << uvar(k);
        if (!refs.empty()) model << " - " << lvar(k, refs.back().t);
        model << " = 0\n";
        double route_travel = 0.0;
        for (const auto& route : fixed_routes) if (route.vehicle == k) route_travel = travel(inst, route.nodes);
        model << cname();
        bool any = false;
        for (const auto& ref : refs) {
            model << (any ? " + " : " ") << inst.pickup_time << " " << pvar(ref.k, ref.t);
            model << " + " << inst.drop_time << " " << dvar(ref.k, ref.t);
            any = true;
        }
        model << (any ? " + " : " ") << inst.drop_time << " " << uvar(k)
              << " <= " << (inst.route_time_limit - route_travel) << "\n";
    }
    for (int i = 1; i <= inst.V; ++i) {
        const std::string Yi = "Y_" + std::to_string(i);
        const std::string ri = "r_" + std::to_string(i);
        const std::string devi = "dev_" + std::to_string(i);
        generals.push_back(Yi);
        bounds.push_back(" 0 <= " + Yi + " <= " + std::to_string(inst.capacity[i]));
        bounds.push_back(" 0 <= " + ri + " <= " + std::to_string(static_cast<double>(inst.capacity[i]) / inst.target[i]));
        bounds.push_back(" 0 <= " + devi);
        model << cname() << " " << Yi << plusTerms(p_terms[i]);
        for (const auto& d : d_terms[i]) model << " - " << d;
        model << " = " << inst.initial[i] << "\n";
        model << cname() << " " << Yi << " - " << inst.target[i] << " " << ri << " = 0\n";
        model << cname() << " " << devi << " - " << ri << " >= -1\n";
        model << cname() << " " << devi << " + " << ri << " >= 1\n";
        const std::string mode = "mode_" + std::to_string(i);
        binaries.push_back(mode);
        if (!p_terms[i].empty() || !d_terms[i].empty()) {
            model << cname() << sumTerms(p_terms[i]) << " - " << inst.initial[i] << " " << mode << " <= 0\n";
            model << cname() << sumTerms(d_terms[i]) << " + " << (inst.capacity[i] - inst.initial[i])
                  << " " << mode << " <= " << (inst.capacity[i] - inst.initial[i]) << "\n";
        }
        model << cname();
        for (std::size_t v = 0; v < domains[i].size(); ++v) {
            const std::string bv = "B_" + std::to_string(i) + "_" + std::to_string(domains[i][v]);
            binaries.push_back(bv);
            model << (v ? " + " : " ") << bv;
        }
        model << " = 1\n";
        model << cname() << " " << Yi;
        for (int y : domains[i]) model << " - " << y << " B_" << i << "_" << y;
        model << " = 0\n";
    }
    for (int i = 1; i <= inst.V; ++i) {
        for (int j = i + 1; j <= inst.V; ++j) {
            const std::string h = "h_" + std::to_string(i) + "_" + std::to_string(j);
            bounds.push_back(" 0 <= " + h + " <= " + std::to_string(hmax));
            model << cname() << " " << h << " - r_" << i << " + r_" << j << " >= 0\n";
            model << cname() << " " << h << " + r_" << i << " - r_" << j << " >= 0\n";
        }
    }
    model << cname() << " P";
    for (int i = 1; i <= inst.V; ++i) model << " - " << inst.weights[i] << " dev_" << i;
    model << " = 0\n";
    model << cname() << " H";
    for (int i = 1; i <= inst.V; ++i) for (int j = i + 1; j <= inst.V; ++j) model << " - h_" << i << "_" << j;
    model << " = 0\n";
    model << cname() << " S";
    for (int i = 1; i <= inst.V; ++i) model << " - r_" << i;
    model << " = 0\n";
    model << cname();
    for (std::size_t m = 0; m < s_values.size(); ++m) {
        binaries.push_back("SV_" + std::to_string(m));
        model << (m ? " + " : " ") << "SV_" << m;
    }
    model << " = 1\n";
    model << cname() << " S";
    for (std::size_t m = 0; m < s_values.size(); ++m) model << " - " << s_values[m] << " SV_" << m;
    model << " = 0\n";
    for (std::size_t m = 0; m < s_values.size(); ++m) {
        const std::string wh = "WH_" + std::to_string(m);
        bounds.push_back(" 0 <= " + wh + " <= " + std::to_string(hmax));
        model << cname() << " " << wh << " - H <= 0\n";
        model << cname() << " " << wh << " - " << hmax << " SV_" << m << " <= 0\n";
        model << cname() << " " << wh << " - H - " << hmax << " SV_" << m << " >= " << -hmax << "\n";
    }
    model << "Bounds\n";
    for (const auto& b : bounds) model << b << "\n";
    model << " 0 <= P\n 0 <= H <= " << hmax << "\n 0 <= S <= " << smax << "\n";
    model << "Generals\n";
    for (const auto& g : generals) model << " " << g << "\n";
    model << "Binaries\n";
    for (const auto& b : binaries) model << " " << b << "\n";
    model << "End\n";
    writeText(lp, model.str());
    std::ostringstream script;
    script << "read " << lp.string() << "\n";
    script << "set timelimit " << opt.cplex_time_limit << "\n";
    script << "set mip tolerances mipgap " << opt.cplex_gap << "\n";
    script << "set mip tolerances absmipgap 0\n";
    script << "optimize\n";
    script << "write " << sol.string() << "\n";
    script << "quit\n";
    writeText(cplexScriptPath(lp), script.str());
    Timer timer;
    const std::string cmd = "cplex -f \"" + cplexScriptPath(lp) + "\" > \"" + log.string() + "\" 2>&1";
    const int code = std::system(cmd.c_str());
    r.cplex_runtime_ms = timer.seconds() * 1000.0;
    if (code != 0 || !std::filesystem::exists(sol)) {
        r.cplex_status = "cplex_failed";
        r.status = "cplex_true_milp_not_closed";
        r.cplex_gap = 1.0;
        r.notes.push_back("CPLEX true fixed-route MILP failed; see " + log.string());
        return false;
    }
    r.cplex_status = parseSolutionStatus(sol);
    r.cplex_objective = parseSolutionObjective(sol);
    const bool optimal = r.cplex_status.find("optimal") != std::string::npos ||
                         r.cplex_status.find("Optimal") != std::string::npos;
    if (!optimal || !std::isfinite(r.cplex_objective)) {
        r.cplex_gap = 1.0;
        r.status = "cplex_true_milp_not_closed";
        r.notes.push_back("CPLEX true fixed-route MILP did not prove optimality.");
        return false;
    }
    auto vars = parseCplexVariables(sol);
    std::vector<RoutePlan> sol_routes = fixed_routes;
    for (auto& route : sol_routes) {
        route.operations.clear();
        for (std::size_t pos = 1; pos + 1 < route.nodes.size(); ++pos) {
            const int s = route.nodes[pos];
            const int p = static_cast<int>(std::llround(vars[pvar(route.vehicle, static_cast<int>(pos))]));
            const int d = static_cast<int>(std::llround(vars[dvar(route.vehicle, static_cast<int>(pos))]));
            if (p || d) route.operations.push_back({s, p, d});
        }
    }
    r.routes = sol_routes;
    r.verification = verifySolution(inst, r.routes, opt.lambda);
    r.verifier_passed = r.verification.feasible;
    r.final_inventory = r.verification.final_inventory;
    r.G = r.verification.G;
    r.P = r.verification.P;
    r.objective = r.verification.objective;
    r.cplex_G = r.G;
    r.cplex_P = r.P;
    r.cplex_LB = r.cplex_objective;
    r.cplex_UB = r.cplex_objective;
    r.cplex_gap = 0.0;
    r.exact_for_fixed_routes = r.verifier_passed;
    r.status = r.verifier_passed ? "cplex_true_fixed_route_milp_optimal" : "cplex_solution_failed_verifier";
    r.notes.push_back("CPLEX solved true fixed-route loading MILP with p/d/L/u/Y/r/deviation/pairwise variables and exact ratio-sum selector linearization.");
    return r.verifier_passed;
}

Result solveCore(const Instance& inst, const RunOptions& opt,
                 const std::vector<RoutePlan>& fixed_routes,
                 bool collect_candidates,
                 std::vector<Candidate>* candidates) {
    Timer timer;
    Result r;
    r.instance = inst.name;
    r.route_set_id = opt.route_json_path.empty() ? "generated_route_set" : basenameNoExt(opt.route_json_path);
    r.V = inst.V;
    r.M = inst.M;
    r.algorithm = opt.algorithm;
    r.route_lengths = routeLengths(fixed_routes);
    r.result_file = opt.out_path;
    r.log_file = opt.log_path;
    r.exact_for_fixed_routes = true;

    auto sets = buildProfileSets(inst, opt, fixed_routes, r);
    CombineState st;
    st.inst = &inst;
    st.opt = &opt;
    st.timer = &timer;
    st.sets = &sets;
    st.inv = inst.initial;
    st.all_candidates = collect_candidates ? candidates : nullptr;
    st.candidate_limit = 200000;
    combine(st, 0);
    r.profile_bpc_nodes = st.nodes;
    r.profile_bpc_pricing_calls = static_cast<long long>(sets.size());
    r.runtime_ms = timer.seconds() * 1000.0;
    if (st.aborted) {
        r.exact_for_fixed_routes = false;
        r.status = "profile_bpc_time_or_node_limit";
        r.notes.push_back("Exact profile master was stopped by profile-BPC time/node limit; no exactness claim is made.");
    }
    if (std::isfinite(st.best.objective)) {
        r.objective = st.best.objective;
        r.G = st.best.G;
        r.P = st.best.P;
        r.final_inventory = st.best.inv;
        r.routes = st.best.routes;
        r.verification = verifySolution(inst, r.routes, opt.lambda);
        r.verifier_passed = r.verification.feasible;
        if (r.status.empty()) r.status = r.exact_for_fixed_routes ? "fixed_route_optimal" : "fixed_route_diagnostic_pruned_profiles";
    } else {
        r.status = "no_feasible_profile_combination";
        r.exact_for_fixed_routes = false;
    }
    if (!r.exact_for_fixed_routes) {
        r.notes.push_back("Profile enumeration was truncated or used pruning; row is diagnostic.");
    }
    r.notes.push_back("Exact mode enumerates all pickup/drop quantities; dominance only merges identical q-vector and final-load profiles keeping the shortest duration.");
    return r;
}

std::vector<std::filesystem::path> round2Inputs(const RunOptions& opt) {
    return {
        std::filesystem::path(opt.exactebrp_root) / "testdata/examples/gcap_smoke_V4_M1.txt",
        std::filesystem::path(opt.exactebrp_root) / "reference/generated/regen_V8_M2_average.txt",
        std::filesystem::path(opt.exactebrp_root) / "reference/generated/regen_V10_M2_average.txt",
        std::filesystem::path(opt.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt",
        std::filesystem::path(opt.exactebrp_root) / "reference/generated/regen_V20_M2_average.txt"
    };
}

std::string caseNameFromPath(const std::filesystem::path& p) {
    return basenameNoExt(p);
}

} // namespace

std::vector<RoutePlan> makeDeterministicRoutes(const Instance& inst, int length_limit) {
    std::vector<int> stations(inst.V);
    std::iota(stations.begin(), stations.end(), 1);
    std::sort(stations.begin(), stations.end(), [&](int a, int b) {
        return std::abs(inst.initial[a] - inst.target[a]) > std::abs(inst.initial[b] - inst.target[b]);
    });
    const int lim = length_limit > 0 ? length_limit
        : std::max(2, std::min(4, (inst.V + inst.M - 1) / inst.M));
    std::vector<RoutePlan> routes(inst.M);
    for (int k = 0; k < inst.M; ++k) {
        routes[k].vehicle = k;
        routes[k].nodes.push_back(0);
    }
    for (std::size_t i = 0; i < stations.size(); ++i) {
        const int k = static_cast<int>(i % inst.M);
        if (static_cast<int>(routes[k].nodes.size()) - 1 < lim) routes[k].nodes.push_back(stations[i]);
    }
    for (auto& r : routes) r.nodes.push_back(0);
    return routes;
}

std::vector<RoutePlan> makeRandomRoutes(const Instance& inst, const RunOptions& opt,
                                        int route_set_index) {
    std::mt19937 rng(static_cast<unsigned>(opt.seed + 9973 * route_set_index));
    std::vector<int> stations(inst.V);
    std::iota(stations.begin(), stations.end(), 1);
    std::shuffle(stations.begin(), stations.end(), rng);
    std::vector<RoutePlan> routes(inst.M);
    for (int k = 0; k < inst.M; ++k) {
        routes[k].vehicle = k;
        routes[k].nodes.push_back(0);
    }
    std::uniform_int_distribution<int> len_dist(opt.route_length_min, opt.route_length_max);
    std::vector<int> remaining_len(inst.M);
    for (int k = 0; k < inst.M; ++k) remaining_len[k] = std::max(1, len_dist(rng));
    int cursor = 0;
    for (int k = 0; k < inst.M; ++k) {
        while (remaining_len[k]-- > 0 && cursor < static_cast<int>(stations.size())) {
            routes[k].nodes.push_back(stations[cursor++]);
        }
    }
    if (opt.allow_duplicate_stations) {
        std::uniform_int_distribution<int> station_dist(1, inst.V);
        for (int k = 0; k < inst.M; ++k) {
            while (routes[k].nodes.size() < static_cast<std::size_t>(opt.route_length_min + 1)) {
                routes[k].nodes.push_back(station_dist(rng));
            }
        }
    }
    for (auto& r : routes) r.nodes.push_back(0);
    return routes;
}

std::vector<RoutePlan> readRoutesFromJson(const std::filesystem::path& path) {
    return readRouteJson(path);
}

void writeRoutesToJson(const std::filesystem::path& path,
                       const std::vector<RoutePlan>& routes,
                       const std::string& route_set_id) {
    writeRouteJson(path, routes, route_set_id);
}

Result solveFixedRoutes(const Instance& inst, const RunOptions& opt,
                        const std::vector<RoutePlan>& fixed_routes) {
    return solveCore(inst, opt, fixed_routes, false, nullptr);
}

Result solveCplexFixedRoute(const Instance& inst, const RunOptions& opt,
                            const std::vector<RoutePlan>& fixed_routes) {
    Timer timer;
    Result r;
    r.instance = inst.name;
    r.route_set_id = opt.route_json_path.empty() ? "generated_route_set" : basenameNoExt(opt.route_json_path);
    r.V = inst.V;
    r.M = inst.M;
    r.route_lengths = routeLengths(fixed_routes);
    r.result_file = opt.out_path;
    r.log_file = opt.log_path;
    r.algorithm = "cplex-fixed-route";
    r.exact_for_fixed_routes = false;
    r.routes = fixed_routes;
    (void)writeAndRunTrueFixedRouteMilp(inst, opt, fixed_routes, r);
    r.runtime_ms = timer.seconds() * 1000.0;
    if (r.cplex_gap == 0.0 && r.verifier_passed) r.objective_diff = 0.0;
    return r;
}

Result runIncrementalTest(const Instance& inst, const RunOptions& opt) {
    Timer total;
    RunOptions exact_opt = opt;
    exact_opt.profile_exact = true;
    exact_opt.allow_natural_mode_pruning = false;
    exact_opt.profile_limit = 0;
    std::vector<RoutePlan> routes = opt.route_json_path.empty()
        ? makeDeterministicRoutes(inst, opt.route_length_limit)
        : readRouteJson(opt.route_json_path);
    std::unordered_map<std::string, std::vector<RouteProfileSet>> cache;
    long long hits = 0, misses = 0;

    auto solveWithCache = [&](const std::vector<RoutePlan>& route_set,
                              bool use_cache,
                              long long& hit_ref,
                              long long& miss_ref) {
        std::vector<RouteProfileSet> sets;
        Result shell;
        shell.exact_for_fixed_routes = true;
        for (const auto& route : route_set) {
            const std::string key = signature(route);
            if (use_cache && cache.count(key)) {
                ++hit_ref;
                sets.push_back(cache[key].front());
            } else {
                ++miss_ref;
                auto ps = profilesForRoute(inst, route, exact_opt);
                if (use_cache) cache[key] = {ps};
                sets.push_back(std::move(ps));
            }
        }
        CombineState st;
        st.inst = &inst;
        st.opt = &exact_opt;
        st.sets = &sets;
        st.inv = inst.initial;
        combine(st, 0);
        return st.best;
    };

    for (const auto& r : routes) {
        long long dummy_h = 0, dummy_m = 0;
        (void)solveWithCache({r}, true, dummy_h, dummy_m);
    }

    double full_time = 0.0, cache_time = 0.0, update_time = 0.0;
    bool ok = true;
    double last_obj = 0.0;
    for (int move = 0; move < 100; ++move) {
        auto changed = routes;
        const int k = move % inst.M;
        if (changed[k].nodes.size() > 3) {
            std::swap(changed[k].nodes[1], changed[k].nodes[changed[k].nodes.size() - 2]);
        }
        Timer t1;
        long long fh = 0, fm = 0;
        Candidate full = solveWithCache(changed, false, fh, fm);
        full_time += t1.seconds();
        Timer t2;
        Candidate cached = solveWithCache(changed, true, hits, misses);
        cache_time += t2.seconds();
        Timer t3;
        const double inc_obj = computeObjectiveParts(inst, cached.inv, opt.lambda).objective;
        update_time += t3.seconds();
        if (std::fabs(full.objective - cached.objective) > 1e-7 ||
            std::fabs(full.objective - inc_obj) > 1e-7) ok = false;
        last_obj = cached.objective;
        routes = changed;
    }

    Result r;
    r.instance = inst.name;
    r.route_set_id = "incremental_100_moves";
    r.V = inst.V;
    r.M = inst.M;
    r.route_lengths = routeLengths(routes);
    r.algorithm = "incremental-test";
    r.status = ok ? "incremental_exact_cache_matches_full" : "incremental_mismatch";
    r.objective = last_obj;
    r.runtime_ms = total.seconds() * 1000.0;
    r.cache_hits = hits;
    r.cache_misses = misses;
    r.incremental_speedup = cache_time > 0.0 ? full_time / cache_time : 0.0;
    r.verifier_passed = ok;
    r.exact_for_fixed_routes = ok;
    r.result_file = opt.out_path;
    r.log_file = opt.log_path;
    r.notes.push_back("Exact cache-reuse solve recomputes only changed route profile sets; objective update was checked against full recomputation.");
    r.notes.push_back("Objective update CPU time ms=" + std::to_string(update_time * 1000.0));
    return r;
}

std::vector<Result> runDefaultSuite(const RunOptions& options) {
    std::vector<std::filesystem::path> inputs = round2Inputs(options);
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

std::vector<Result> runRound2Suite(const RunOptions& options) {
    const std::filesystem::path root = "results/round2_exact_loading";
    for (const auto& d : {"raw", "logs", "routes", "cplex", "summaries", "docs"}) {
        std::filesystem::create_directories(root / d);
    }
    struct CaseSpec { std::filesystem::path path; int count; int min_len; int max_len; };
    std::vector<CaseSpec> cases = {
        {std::filesystem::path(options.exactebrp_root) / "testdata/examples/gcap_smoke_V4_M1.txt", 20, 1, 3},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V8_M2_average.txt", 20, 1, 2},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V10_M2_average.txt", 20, 1, 2},
        {std::filesystem::path(options.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt", 50, 1, 2},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V20_M2_average.txt", 10, 1, 2}
    };
    std::vector<Result> all;
    std::string skipped = "instance,route_set_id,reason\n";
    skipped += "regen_candidate_V12_M2_average.txt,medium_length_5_6,"
               "skipped_exact_profile_enumeration_state_explosion_after_initial_round2_timeout\n";
    skipped += "regen_candidate_V12_M2_average.txt,long_length_7_8,"
               "skipped_exact_profile_enumeration_state_explosion_after_initial_round2_timeout\n";
    for (const auto& cs : cases) {
        if (!std::filesystem::exists(cs.path)) {
            skipped += cs.path.string() + ",,missing_input\n";
            continue;
        }
        Instance inst = parseInstanceFile(cs.path, options.route_time_limit,
                                          options.pickup_time, options.drop_time);
        for (int id = 0; id < cs.count; ++id) {
            RunOptions opt = options;
            opt.input_path = cs.path.string();
            opt.route_length_min = cs.min_len;
            opt.route_length_max = cs.max_len;
            opt.seed = options.seed + id * 17;
            opt.profile_exact = true;
            opt.allow_natural_mode_pruning = false;
            opt.profile_limit = 0;
            std::string route_set_id = caseNameFromPath(cs.path) + "_rs" + std::to_string(id);
            auto routes = (id == 0) ? makeDeterministicRoutes(inst, cs.max_len)
                                    : makeRandomRoutes(inst, opt, id);
            const auto route_path = root / "routes" / (route_set_id + ".json");
            writeRouteJson(route_path, routes, route_set_id);
            opt.route_json_path = route_path.string();

            opt.algorithm = "cplex-fixed-route";
            opt.out_path = (root / "raw" / (route_set_id + "_cplex.json")).string();
            opt.log_path = (root / "logs" / (route_set_id + "_cplex.log")).string();
            Result cplex = solveCplexFixedRoute(inst, opt, routes);
            cplex.route_set_id = route_set_id;
            writeText(cplex.result_file, toJson(cplex));
            writeText(cplex.log_file, "status=" + cplex.status + "\ncplex_status=" + cplex.cplex_status + "\n");
            all.push_back(cplex);

            opt.algorithm = "profile-dp";
            opt.out_path = (root / "raw" / (route_set_id + "_profile.json")).string();
            opt.log_path = (root / "logs" / (route_set_id + "_profile.log")).string();
            Result prof = solveFixedRoutes(inst, opt, routes);
            prof.route_set_id = route_set_id;
            prof.cplex_status = cplex.cplex_status;
            prof.cplex_objective = cplex.cplex_objective;
            prof.cplex_G = cplex.cplex_G;
            prof.cplex_P = cplex.cplex_P;
            prof.cplex_LB = cplex.cplex_LB;
            prof.cplex_UB = cplex.cplex_UB;
            prof.cplex_gap = cplex.cplex_gap;
            prof.objective_diff = std::fabs(prof.objective - cplex.cplex_objective);
            if (prof.objective_diff > 1e-6 && cplex.cplex_gap == 0.0) {
                prof.status = "exact_mismatch_against_cplex";
                prof.exact_for_fixed_routes = false;
            }
            writeText(prof.result_file, toJson(prof));
            writeText(prof.log_file, "status=" + prof.status + "\ndiff=" + std::to_string(prof.objective_diff) + "\n");
            all.push_back(prof);
        }
    }

    if (!cases.empty() && std::filesystem::exists(cases[0].path)) {
        RunOptions opt = options;
        opt.input_path = cases[0].path.string();
        opt.algorithm = "incremental-test";
        opt.profile_exact = true;
        opt.allow_natural_mode_pruning = false;
        opt.profile_limit = 0;
        Instance inst = parseInstanceFile(opt.input_path, opt.route_time_limit,
                                          opt.pickup_time, opt.drop_time);
        opt.out_path = (root / "raw" / "incremental_exact_v4.json").string();
        opt.log_path = (root / "logs" / "incremental_exact_v4.log").string();
        Result inc = runIncrementalTest(inst, opt);
        inc.route_set_id = "incremental_exact_v4_100_moves";
        writeText(inc.result_file, toJson(inc));
        writeText(inc.log_file, "status=" + inc.status + "\nspeedup=" + std::to_string(inc.incremental_speedup) + "\n");
        all.push_back(inc);
    }

    auto writeSummary = [&](const std::string& file, const std::vector<Result>& rows) {
        std::string csv = csvHeader();
        for (const auto& r : rows) csv += csvRow(r);
        writeText(root / "summaries" / file, csv);
    };
    std::vector<Result> cplex_rows, profile_rows, v12_rows, inc_rows;
    for (const auto& r : all) {
        if (r.algorithm == "cplex-fixed-route") cplex_rows.push_back(r);
        if (r.algorithm == "profile-dp" || r.algorithm == "profile-bpc") profile_rows.push_back(r);
        if (r.instance.find("V12") != std::string::npos || r.instance.find("12") != std::string::npos) v12_rows.push_back(r);
        if (r.algorithm == "incremental-test") inc_rows.push_back(r);
    }
    writeSummary("cplex_fixed_route_summary.csv", cplex_rows);
    writeSummary("exact_algorithm_comparison.csv", profile_rows);
    writeSummary("v12_m2_route_set_comparison.csv", v12_rows);
    writeSummary("incremental_exact_summary.csv", inc_rows);
    writeText(root / "summaries" / "skipped_or_failed_rows.csv", skipped);
    return all;
}

std::vector<Result> runRound3Suite(const RunOptions& options) {
    const std::filesystem::path root = "results/round3_paper_exact_loading";
    for (const auto& d : {"raw", "logs", "routes", "cplex", "summaries", "docs"}) {
        std::filesystem::create_directories(root / d);
    }
    struct CaseSpec {
        std::filesystem::path path;
        std::string group;
        int count;
        int min_len;
        int max_len;
    };
    std::vector<CaseSpec> cases = {
        {std::filesystem::path(options.exactebrp_root) / "testdata/examples/gcap_smoke_V4_M1.txt", "v4", 20, 1, 3},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V8_M2_average.txt", "v8", 20, 1, 3},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V10_M2_average.txt", "v10", 20, 1, 3},
        {std::filesystem::path(options.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt", "v12_short", 30, 1, 2},
        {std::filesystem::path(options.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt", "v12_medium", 30, 3, 4},
        {std::filesystem::path(options.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt", "v12_long", 20, 5, 6},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V20_M2_average.txt", "v20", 10, 1, 2}
    };
    std::vector<Result> all;
    std::vector<Result> cplex_rows, bpc_rows, dp_rows, v12_medium_rows, inc_rows;
    std::string skipped = "instance,route_set_id,reason\n";

    auto writeOne = [&](Result& row, const std::string& log_extra = "") {
        writeText(row.result_file, toJson(row));
        writeText(row.log_file, "status=" + row.status + "\nalgorithm=" + row.algorithm +
                  "\nobjective=" + std::to_string(row.objective) + "\n" + log_extra);
        all.push_back(row);
    };

    for (const auto& cs : cases) {
        if (!std::filesystem::exists(cs.path)) {
            skipped += cs.path.string() + ",,missing_input\n";
            continue;
        }
        Instance inst = parseInstanceFile(cs.path, options.route_time_limit,
                                          options.pickup_time, options.drop_time);
        for (int id = 0; id < cs.count; ++id) {
            RunOptions opt = options;
            opt.input_path = cs.path.string();
            opt.route_length_min = cs.min_len;
            opt.route_length_max = cs.max_len;
            opt.seed = options.seed + id * 31 + static_cast<int>(cs.group.size()) * 1009;
            opt.profile_exact = true;
            opt.allow_natural_mode_pruning = false;
            opt.profile_limit = 0;
            std::string route_set_id = caseNameFromPath(cs.path) + "_" + cs.group + "_rs" + std::to_string(id);
            auto routes = (id == 0) ? makeDeterministicRoutes(inst, cs.max_len)
                                    : makeRandomRoutes(inst, opt, id);
            const auto route_path = root / "routes" / (route_set_id + ".json");
            writeRouteJson(route_path, routes, route_set_id);
            opt.route_json_path = route_path.string();

            opt.algorithm = "cplex-fixed-route";
            opt.out_path = (root / "raw" / (route_set_id + "_cplex_milp.json")).string();
            opt.log_path = (root / "logs" / (route_set_id + "_cplex_milp.log")).string();
            Result cplex;
            const bool cplex_exists = std::filesystem::exists(opt.out_path);
            if (!cplex_exists) {
                cplex = solveCplexFixedRoute(inst, opt, routes);
                cplex.route_set_id = route_set_id;
                writeOne(cplex, "cplex_status=" + cplex.cplex_status + "\n");
                cplex_rows.push_back(cplex);
                if (cs.group == "v12_medium") v12_medium_rows.push_back(cplex);
            } else {
                cplex.instance = inst.name;
                cplex.route_set_id = route_set_id;
                cplex.algorithm = "cplex-fixed-route";
                cplex.status = "existing_raw_not_rerun";
                cplex.cplex_status = "existing_raw_not_rerun";
                cplex.result_file = opt.out_path;
            }

            const bool run_profile = !(cs.group == "v12_long");
            if (!run_profile) {
                skipped += inst.name + "," + route_set_id + ",profile_bpc_skipped_for_long_routes_after_medium_rows_tested\n";
                continue;
            }

            opt.algorithm = "profile-bpc";
            opt.out_path = (root / "raw" / (route_set_id + "_profile_bpc.json")).string();
            opt.log_path = (root / "logs" / (route_set_id + "_profile_bpc.log")).string();
            if (std::filesystem::exists(opt.out_path)) continue;
            Result bpc = solveFixedRoutes(inst, opt, routes);
            bpc.algorithm = "profile-bpc";
            bpc.status = bpc.exact_for_fixed_routes ? "profile_bpc_exact_bb_optimal" : bpc.status;
            bpc.route_set_id = route_set_id;
            bpc.cplex_status = cplex.cplex_status;
            bpc.cplex_objective = cplex.cplex_objective;
            bpc.cplex_G = cplex.cplex_G;
            bpc.cplex_P = cplex.cplex_P;
            bpc.cplex_LB = cplex.cplex_LB;
            bpc.cplex_UB = cplex.cplex_UB;
            bpc.cplex_gap = cplex.cplex_gap;
            if (cplex.cplex_gap == 0.0 && cplex.verifier_passed) {
                bpc.objective_diff = std::fabs(bpc.objective - cplex.cplex_objective);
                if (bpc.objective_diff > 1e-6) {
                    bpc.status = "profile_bpc_mismatch_against_true_cplex_milp";
                    bpc.exact_for_fixed_routes = false;
                }
            } else {
                bpc.notes.push_back("CPLEX true MILP did not close; profile-BPC exactness is not externally certified on this row.");
            }
            writeOne(bpc, "objective_diff=" + std::to_string(bpc.objective_diff) + "\n");
            bpc_rows.push_back(bpc);
            if (cs.group == "v12_medium") v12_medium_rows.push_back(bpc);

            if (cs.group == "v4" || cs.group == "v8" || cs.group == "v10") {
                opt.algorithm = "profile-dp";
                opt.out_path = (root / "raw" / (route_set_id + "_profile_dp.json")).string();
                opt.log_path = (root / "logs" / (route_set_id + "_profile_dp.log")).string();
                Result dp = solveFixedRoutes(inst, opt, routes);
                dp.algorithm = "profile-dp";
                dp.route_set_id = route_set_id;
                dp.cplex_status = cplex.cplex_status;
                dp.cplex_objective = cplex.cplex_objective;
                dp.cplex_LB = cplex.cplex_LB;
                dp.cplex_UB = cplex.cplex_UB;
                dp.cplex_gap = cplex.cplex_gap;
                if (cplex.cplex_gap == 0.0 && cplex.verifier_passed) dp.objective_diff = std::fabs(dp.objective - cplex.cplex_objective);
                writeOne(dp, "objective_diff=" + std::to_string(dp.objective_diff) + "\n");
                dp_rows.push_back(dp);
            }
        }
    }

    std::vector<std::pair<std::filesystem::path, int>> inc_cases = {
        {std::filesystem::path(options.exactebrp_root) / "testdata/examples/gcap_smoke_V4_M1.txt", 3},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V8_M2_average.txt", 2},
        {std::filesystem::path(options.exactebrp_root) / "reference/generated/regen_V10_M2_average.txt", 2},
        {std::filesystem::path(options.exactebrp_root) / "reference/regen_candidate_V12_M2_average.txt", 2}
    };
    for (const auto& inc_case : inc_cases) {
        if (!std::filesystem::exists(inc_case.first)) continue;
        RunOptions opt = options;
        opt.input_path = inc_case.first.string();
        opt.algorithm = "incremental-test";
        opt.route_length_limit = inc_case.second;
        opt.profile_exact = true;
        opt.allow_natural_mode_pruning = false;
        opt.profile_limit = 0;
        Instance inst = parseInstanceFile(opt.input_path, opt.route_time_limit,
                                          opt.pickup_time, opt.drop_time);
        const std::string tag = caseNameFromPath(inc_case.first) + "_inc";
        opt.out_path = (root / "raw" / (tag + "_incremental.json")).string();
        opt.log_path = (root / "logs" / (tag + "_incremental.log")).string();
        Result inc = runIncrementalTest(inst, opt);
        inc.route_set_id = tag + "_100_moves";
        writeOne(inc, "speedup=" + std::to_string(inc.incremental_speedup) + "\n");
        inc_rows.push_back(inc);
    }

    auto writeSummary = [&](const std::string& file, const std::vector<Result>& rows) {
        std::string csv = csvHeader();
        for (const auto& r : rows) csv += csvRow(r);
        writeText(root / "summaries" / file, csv);
    };
    writeSummary("cplex_fixed_route_milp_summary.csv", cplex_rows);
    writeSummary("profile_bpc_comparison.csv", bpc_rows);
    writeSummary("profile_dp_baseline.csv", dp_rows);
    writeSummary("v12_m2_medium_route_comparison.csv", v12_medium_rows);
    writeSummary("incremental_exact_summary.csv", inc_rows);
    writeSummary("all_round3_results.csv", all);
    writeText(root / "summaries" / "skipped_or_failed_rows.csv", skipped);
    return all;
}

} // namespace load_exact
