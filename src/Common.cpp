#include "Common.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace load_exact {
namespace {

constexpr double kReadDistanceSpeed = 1.5;

std::string readAll(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open instance: " + path.string());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::vector<double> nums(const std::string& text) {
    static const std::regex re(R"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)");
    std::vector<double> out;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), re);
         it != std::sregex_iterator(); ++it) out.push_back(std::stod(it->str()));
    return out;
}

std::string payload(const std::string& text, const std::string& name) {
    const std::regex re(name + R"(\s*=\s*\[([\s\S]*?)\])");
    std::smatch m;
    return std::regex_search(text, m, re) ? m[1].str() : std::string{};
}

std::vector<int> ints(const std::string& text, const std::string& name) {
    std::vector<int> out;
    for (double x : nums(payload(text, name))) out.push_back(static_cast<int>(std::llround(x)));
    return out;
}

std::vector<std::pair<double, double>> parsePoints(const std::string& text) {
    auto v = nums(payload(text, "points"));
    std::vector<std::pair<double, double>> p;
    for (std::size_t i = 0; i + 1 < v.size(); i += 2) p.push_back({v[i], v[i + 1]});
    return p;
}

std::vector<std::vector<double>> parseMatrix(const std::string& text) {
    std::vector<std::vector<double>> m;
    std::istringstream in(text);
    std::string line;
    bool active = false;
    while (std::getline(in, line)) {
        if (!active) {
            active = line.find("distances") != std::string::npos;
            continue;
        }
        if (line.find(']') != std::string::npos) break;
        auto row = nums(line);
        if (!row.empty()) m.push_back(row);
    }
    return m;
}

std::vector<std::vector<double>> distances(const std::vector<std::pair<double, double>>& p) {
    std::vector<std::vector<double>> d(p.size(), std::vector<double>(p.size(), 0.0));
    for (std::size_t i = 0; i < p.size(); ++i)
        for (std::size_t j = 0; j < p.size(); ++j)
            d[i][j] = i == j ? 0.0 : std::hypot(p[i].first - p[j].first, p[i].second - p[j].second) / kReadDistanceSpeed;
    return d;
}

std::string esc(const std::string& s) {
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

std::string csvEsc(const std::string& s) {
    if (s.find_first_of(",\"\n\r") == std::string::npos) return s;
    std::string out = "\"";
    for (char c : s) out += c == '"' ? "\"\"" : std::string(1, c);
    out += "\"";
    return out;
}

template <class T>
void vec(std::ostringstream& out, const std::vector<T>& v) {
    out << "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) out << ", ";
        out << v[i];
    }
    out << "]";
}

} // namespace

Timer::Timer() : start_(std::chrono::steady_clock::now()) {}
double Timer::seconds() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
}

Instance parseInstanceFile(const std::filesystem::path& path,
                           double route_time_limit,
                           double pickup_time,
                           double drop_time) {
    const std::string text = readAll(path);
    std::istringstream in(text);
    std::string first;
    std::getline(in, first);
    auto h = nums(first);
    if (h.size() < 2) throw std::runtime_error("Invalid instance first line");
    Instance inst;
    inst.path = std::filesystem::absolute(path).string();
    inst.name = path.filename().string();
    inst.V = static_cast<int>(std::llround(h[0]));
    inst.M = static_cast<int>(std::llround(h[1]));
    inst.route_time_limit = route_time_limit;
    inst.pickup_time = pickup_time;
    inst.drop_time = drop_time;
    auto l = first.find('['), r = first.find(']', l);
    for (double q : nums(first.substr(l, r - l + 1))) inst.Q.push_back(static_cast<int>(std::llround(q)));
    inst.capacity = ints(text, "capacities");
    inst.initial = ints(text, "initial");
    inst.target = ints(text, "target");
    inst.weights = nums(payload(text, "weights"));
    inst.min_ratio = nums(payload(text, "min_ratio"));
    const std::size_t n = inst.V + 1;
    if (inst.weights.empty()) { inst.weights.assign(n, 1.0); inst.weights[0] = 0.0; }
    if (inst.min_ratio.empty()) inst.min_ratio.assign(n, 0.0);
    inst.points = parsePoints(text);
    inst.dist = inst.points.size() == n ? distances(inst.points) : parseMatrix(text);
    if (inst.Q.size() != static_cast<std::size_t>(inst.M) || inst.capacity.size() != n ||
        inst.initial.size() != n || inst.target.size() != n || inst.dist.size() != n) {
        throw std::runtime_error("Parsed instance vector size mismatch");
    }
    return inst;
}

ObjectiveParts computeObjectiveParts(const Instance& inst,
                                     const std::vector<int>& final_inventory,
                                     double lambda) {
    ObjectiveParts o;
    std::vector<double> ratio(inst.V + 1, 0.0);
    for (int i = 1; i <= inst.V; ++i) {
        ratio[i] = static_cast<double>(final_inventory[i]) / inst.target[i];
        o.S += ratio[i];
        o.P += inst.weights[i] * std::fabs(ratio[i] - 1.0);
    }
    for (int i = 1; i <= inst.V; ++i)
        for (int j = i + 1; j <= inst.V; ++j) o.H += std::fabs(ratio[i] - ratio[j]);
    o.G = o.S > 0.0 ? o.H / (static_cast<double>(inst.V) * o.S) : 0.0;
    o.objective = o.G + lambda * o.P;
    return o;
}

Verification verifySolution(const Instance& inst,
                            const std::vector<RoutePlan>& routes,
                            double lambda) {
    Verification v;
    v.final_inventory = inst.initial;
    v.route_duration.assign(inst.M, 0.0);
    std::vector<int> seen(inst.V + 1, 0);
    for (const auto& r : routes) {
        if (r.vehicle < 0 || r.vehicle >= inst.M || r.nodes.size() < 2 ||
            r.nodes.front() != 0 || r.nodes.back() != 0) v.errors.push_back("invalid route");
        double travel = 0.0;
        std::unordered_set<int> local;
        for (std::size_t i = 1; i < r.nodes.size(); ++i) {
            travel += inst.dist[r.nodes[i - 1]][r.nodes[i]];
            if (r.nodes[i] != 0) { seen[r.nodes[i]]++; local.insert(r.nodes[i]); }
        }
        int load = 0, pick = 0, drop = 0;
        for (std::size_t i = 1; i + 1 < r.nodes.size(); ++i) {
            const int s = r.nodes[i];
            auto it = std::find_if(r.operations.begin(), r.operations.end(),
                                   [s](const StopOperation& op) { return op.station == s; });
            if (it == r.operations.end()) continue;
            load += it->pickup - it->drop;
            pick += it->pickup;
            drop += it->drop;
            v.final_inventory[s] += it->drop - it->pickup;
            if (load < 0 || load > inst.Q[r.vehicle]) v.errors.push_back("load infeasible");
        }
        const double duration = travel + inst.pickup_time * pick + inst.drop_time * (drop + load);
        v.route_duration[r.vehicle] = duration;
        if (duration > inst.route_time_limit + 1e-7) v.errors.push_back("duration exceeded");
    }
    for (int i = 1; i <= inst.V; ++i) {
        if (seen[i] > 1) v.errors.push_back("station served multiple times");
        if (v.final_inventory[i] < 0 || v.final_inventory[i] > inst.capacity[i]) v.errors.push_back("station capacity");
    }
    auto o = computeObjectiveParts(inst, v.final_inventory, lambda);
    v.objective = o.objective;
    v.G = o.G;
    v.P = o.P;
    v.feasible = v.errors.empty() && std::isfinite(v.objective);
    return v;
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot write " + path.string());
    out << text;
}

std::string toJson(const Result& r) {
    std::ostringstream out;
    out << std::setprecision(12);
    out << "{\n";
    out << "  \"instance\": \"" << esc(r.instance) << "\",\n";
    out << "  \"route_set_id\": \"" << esc(r.route_set_id) << "\",\n";
    out << "  \"V\": " << r.V << ",\n";
    out << "  \"M\": " << r.M << ",\n";
    out << "  \"route_lengths\": \"" << esc(r.route_lengths) << "\",\n";
    out << "  \"algorithm\": \"" << esc(r.algorithm) << "\",\n";
    out << "  \"status\": \"" << esc(r.status) << "\",\n";
    out << "  \"objective\": " << r.objective << ",\n";
    out << "  \"G\": " << r.G << ",\n";
    out << "  \"P\": " << r.P << ",\n";
    out << "  \"runtime_ms\": " << r.runtime_ms << ",\n";
    out << "  \"cplex_status\": \"" << esc(r.cplex_status) << "\",\n";
    out << "  \"cplex_objective\": " << r.cplex_objective << ",\n";
    out << "  \"cplex_G\": " << r.cplex_G << ",\n";
    out << "  \"cplex_P\": " << r.cplex_P << ",\n";
    out << "  \"cplex_LB\": " << r.cplex_LB << ",\n";
    out << "  \"cplex_UB\": " << r.cplex_UB << ",\n";
    out << "  \"cplex_gap\": " << r.cplex_gap << ",\n";
    out << "  \"cplex_runtime_ms\": " << r.cplex_runtime_ms << ",\n";
    out << "  \"cplex_nodes\": " << r.cplex_nodes << ",\n";
    out << "  \"profile_bpc_status\": \"" << esc(r.status) << "\",\n";
    out << "  \"profile_bpc_objective\": " << r.objective << ",\n";
    out << "  \"profile_bpc_G\": " << r.G << ",\n";
    out << "  \"profile_bpc_P\": " << r.P << ",\n";
    out << "  \"profile_bpc_LB\": " << r.objective << ",\n";
    out << "  \"profile_bpc_UB\": " << r.objective << ",\n";
    out << "  \"profile_bpc_gap\": " << (r.exact_for_fixed_routes ? 0.0 : 1.0) << ",\n";
    out << "  \"profile_bpc_runtime_ms\": " << r.runtime_ms << ",\n";
    out << "  \"objective_diff\": " << r.objective_diff << ",\n";
    out << "  \"profiles_generated\": " << r.profiles_generated << ",\n";
    out << "  \"profiles_after_dominance\": " << r.profiles_after_dominance << ",\n";
    out << "  \"profile_bpc_nodes\": " << r.profile_bpc_nodes << ",\n";
    out << "  \"profile_bpc_pricing_calls\": " << r.profile_bpc_pricing_calls << ",\n";
    out << "  \"cache_hits\": " << r.cache_hits << ",\n";
    out << "  \"cache_misses\": " << r.cache_misses << ",\n";
    out << "  \"incremental_speedup\": " << r.incremental_speedup << ",\n";
    out << "  \"verifier_passed\": " << (r.verifier_passed ? "true" : "false") << ",\n";
    out << "  \"exact_for_fixed_routes\": " << (r.exact_for_fixed_routes ? "true" : "false") << ",\n";
    out << "  \"result_file\": \"" << esc(r.result_file) << "\",\n";
    out << "  \"log_file\": \"" << esc(r.log_file) << "\",\n";
    out << "  \"final_inventory\": "; vec(out, r.final_inventory); out << ",\n";
    out << "  \"verification_errors\": [";
    for (std::size_t i = 0; i < r.verification.errors.size(); ++i) {
        if (i) out << ", ";
        out << "\"" << esc(r.verification.errors[i]) << "\"";
    }
    out << "],\n";
    out << "  \"notes\": [";
    for (std::size_t i = 0; i < r.notes.size(); ++i) {
        if (i) out << ", ";
        out << "\"" << esc(r.notes[i]) << "\"";
    }
    out << "]\n}\n";
    return out.str();
}

std::string csvHeader() {
    return "instance,route_set_id,V,M,route_lengths,algorithm,status,objective,G,P,runtime_ms,"
           "cplex_status,cplex_objective,cplex_G,cplex_P,cplex_LB,cplex_UB,cplex_gap,cplex_runtime_ms,cplex_nodes,"
           "objective_diff,profiles_generated,profiles_after_dominance,profile_bpc_nodes,profile_bpc_pricing_calls,"
           "cache_hits,cache_misses,incremental_speedup,verifier_passed,exact_for_fixed_routes,result_file,log_file,notes\n";
}

std::string csvRow(const Result& r) {
    std::string notes;
    for (const auto& n : r.notes) { if (!notes.empty()) notes += "; "; notes += n; }
    std::ostringstream out;
    out << std::setprecision(12)
        << csvEsc(r.instance) << ',' << csvEsc(r.route_set_id) << ',' << r.V << ',' << r.M << ','
        << csvEsc(r.route_lengths) << ',' << csvEsc(r.algorithm) << ',' << csvEsc(r.status) << ','
        << r.objective << ',' << r.G << ',' << r.P << ',' << r.runtime_ms << ','
        << csvEsc(r.cplex_status) << ',' << r.cplex_objective << ',' << r.cplex_G << ',' << r.cplex_P << ','
        << r.cplex_LB << ',' << r.cplex_UB << ',' << r.cplex_gap << ',' << r.cplex_runtime_ms << ','
        << r.cplex_nodes << ',' << r.objective_diff << ','
        << r.profiles_generated << ',' << r.profiles_after_dominance << ',' << r.profile_bpc_nodes << ','
        << r.profile_bpc_pricing_calls << ',' << r.cache_hits << ','
        << r.cache_misses << ',' << r.incremental_speedup << ',' << (r.verifier_passed ? "true" : "false")
        << ',' << (r.exact_for_fixed_routes ? "true" : "false") << ',' << csvEsc(r.result_file) << ','
        << csvEsc(r.log_file) << ',' << csvEsc(notes) << "\n";
    return out.str();
}

std::string basenameNoExt(const std::filesystem::path& path) {
    return path.stem().string();
}

} // namespace load_exact
