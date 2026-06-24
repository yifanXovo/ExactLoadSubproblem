#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace load_exact {

struct Instance {
    std::string path;
    std::string name;
    int V = 0;
    int M = 0;
    std::vector<int> Q;
    std::vector<int> capacity;
    std::vector<int> initial;
    std::vector<int> target;
    std::vector<double> weights;
    std::vector<double> min_ratio;
    std::vector<std::pair<double, double>> points;
    std::vector<std::vector<double>> dist;
    double route_time_limit = 3600.0;
    double pickup_time = 60.0;
    double drop_time = 60.0;
};

struct StopOperation {
    int station = 0;
    int pickup = 0;
    int drop = 0;
};

struct RoutePlan {
    int vehicle = 0;
    std::vector<int> nodes;
    std::vector<StopOperation> operations;
};

struct ObjectiveParts {
    double G = 0.0;
    double P = 0.0;
    double objective = 0.0;
    double S = 0.0;
    double H = 0.0;
};

struct Verification {
    bool feasible = false;
    double objective = 0.0;
    double G = 0.0;
    double P = 0.0;
    std::vector<int> final_inventory;
    std::vector<double> route_duration;
    std::vector<std::string> errors;
};

struct RunOptions {
    std::string input_path;
    std::string out_path;
    std::string log_path;
    std::string exactebrp_root = "../ExactEBRP";
    std::string algorithm = "profile-dp";
    double lambda = 0.15;
    double route_time_limit = 3600.0;
    double time_limit_seconds = 300.0;
    double pickup_time = 60.0;
    double drop_time = 60.0;
    int route_length_limit = 0;
    int profile_limit = 0;
    bool profile_exact = true;
    bool allow_natural_mode_pruning = false;
    double cplex_time_limit = 300.0;
    double cplex_gap = 0.0;
    std::string route_json_path;
    std::string route_generator = "deterministic";
    std::string profile_master = "bb";
    std::string profile_pricing = "exact-dp";
    int route_count = 1;
    int route_length_min = 3;
    int route_length_max = 4;
    int seed = 1;
    bool allow_duplicate_stations = false;
    double profile_bpc_time_limit = 300.0;
    double profile_bpc_gap = 0.0;
    int profile_bpc_max_nodes = 0;
    bool run_suite = false;
    bool round2_suite = false;
    bool round3_suite = false;
};

struct Result {
    std::string instance;
    std::string route_set_id;
    int V = 0;
    int M = 0;
    std::string route_lengths;
    std::string algorithm;
    std::string status;
    double objective = 0.0;
    double G = 0.0;
    double P = 0.0;
    double runtime_ms = 0.0;
    double cplex_objective = 0.0;
    double cplex_G = 0.0;
    double cplex_P = 0.0;
    double cplex_LB = 0.0;
    double cplex_UB = 0.0;
    double cplex_gap = 0.0;
    double cplex_runtime_ms = 0.0;
    long long cplex_nodes = 0;
    std::string cplex_status = "not_run";
    double objective_diff = 0.0;
    long long profiles_generated = 0;
    long long profiles_after_dominance = 0;
    long long profile_bpc_nodes = 0;
    long long profile_bpc_pricing_calls = 0;
    long long cache_hits = 0;
    long long cache_misses = 0;
    double incremental_speedup = 0.0;
    bool verifier_passed = false;
    bool exact_for_fixed_routes = false;
    std::string result_file;
    std::string log_file;
    std::vector<RoutePlan> routes;
    std::vector<int> final_inventory;
    Verification verification;
    std::vector<std::string> notes;
};

class Timer {
public:
    Timer();
    double seconds() const;
private:
    std::chrono::steady_clock::time_point start_;
};

Instance parseInstanceFile(const std::filesystem::path& path,
                           double route_time_limit,
                           double pickup_time,
                           double drop_time);
ObjectiveParts computeObjectiveParts(const Instance& instance,
                                     const std::vector<int>& final_inventory,
                                     double lambda);
Verification verifySolution(const Instance& instance,
                            const std::vector<RoutePlan>& routes,
                            double lambda);
void writeText(const std::filesystem::path& path, const std::string& text);
std::string toJson(const Result& result);
std::string csvHeader();
std::string csvRow(const Result& result);
std::string basenameNoExt(const std::filesystem::path& path);

} // namespace load_exact
