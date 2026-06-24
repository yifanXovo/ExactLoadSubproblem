#include "Common.hpp"
#include "LoadSolver.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace load_exact;

namespace {

std::string value(int& i, int argc, char** argv) {
    if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    return argv[++i];
}

RunOptions parse(int argc, char** argv) {
    RunOptions opt;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--input") opt.input_path = value(i, argc, argv);
        else if (a == "--out") opt.out_path = value(i, argc, argv);
        else if (a == "--log") opt.log_path = value(i, argc, argv);
        else if (a == "--algorithm") opt.algorithm = value(i, argc, argv);
        else if (a == "--lambda") opt.lambda = std::stod(value(i, argc, argv));
        else if (a == "--T") opt.route_time_limit = std::stod(value(i, argc, argv));
        else if (a == "--time-limit") opt.time_limit_seconds = std::stod(value(i, argc, argv));
        else if (a == "--pickup-time") opt.pickup_time = std::stod(value(i, argc, argv));
        else if (a == "--drop-time") opt.drop_time = std::stod(value(i, argc, argv));
        else if (a == "--route-length-limit") opt.route_length_limit = std::stoi(value(i, argc, argv));
        else if (a == "--profile-limit") opt.profile_limit = std::stoi(value(i, argc, argv));
        else if (a == "--profile-exact") opt.profile_exact = value(i, argc, argv) == "true";
        else if (a == "--allow-natural-mode-pruning") opt.allow_natural_mode_pruning = value(i, argc, argv) == "true";
        else if (a == "--cplex-time-limit") opt.cplex_time_limit = std::stod(value(i, argc, argv));
        else if (a == "--cplex-gap") opt.cplex_gap = std::stod(value(i, argc, argv));
        else if (a == "--route-json") opt.route_json_path = value(i, argc, argv);
        else if (a == "--route-generator") opt.route_generator = value(i, argc, argv);
        else if (a == "--profile-master") opt.profile_master = value(i, argc, argv);
        else if (a == "--profile-pricing") opt.profile_pricing = value(i, argc, argv);
        else if (a == "--route-count") opt.route_count = std::stoi(value(i, argc, argv));
        else if (a == "--route-length-min") opt.route_length_min = std::stoi(value(i, argc, argv));
        else if (a == "--route-length-max") opt.route_length_max = std::stoi(value(i, argc, argv));
        else if (a == "--seed") opt.seed = std::stoi(value(i, argc, argv));
        else if (a == "--allow-duplicate-stations") opt.allow_duplicate_stations = value(i, argc, argv) == "true";
        else if (a == "--profile-bpc-time-limit") opt.profile_bpc_time_limit = std::stod(value(i, argc, argv));
        else if (a == "--profile-bpc-gap") opt.profile_bpc_gap = std::stod(value(i, argc, argv));
        else if (a == "--profile-bpc-max-nodes") opt.profile_bpc_max_nodes = std::stoi(value(i, argc, argv));
        else if (a == "--exactebrp-root") opt.exactebrp_root = value(i, argc, argv);
        else if (a == "--run-suite") opt.run_suite = true;
        else if (a == "--round2-suite") opt.round2_suite = true;
        else if (a == "--round3-suite") opt.round3_suite = true;
        else if (a == "--help") {
            std::cout << "ExactLoadSubproblem --input file --algorithm profile-dp|profile-bpc|incremental-test|cplex-fixed-route --route-json path --out json --log log\n";
            std::exit(0);
        } else throw std::runtime_error("Unknown argument: " + a);
    }
    return opt;
}

} // namespace

int main(int argc, char** argv) {
    try {
        RunOptions opt = parse(argc, argv);
        if (opt.round2_suite) {
            auto results = runRound2Suite(opt);
            std::cout << "round2_results=" << results.size() << "\n";
            return 0;
        }
        if (opt.round3_suite) {
            auto results = runRound3Suite(opt);
            std::cout << "round3_results=" << results.size() << "\n";
            return 0;
        }
        if (opt.run_suite) {
            auto results = runDefaultSuite(opt);
            std::string csv = csvHeader();
            for (const auto& r : results) csv += csvRow(r);
            writeText("results/load_exact_summary.csv", csv);
            writeText("results/load_cplex_comparison.csv", csv);
            writeText("results/incremental_eval_summary.csv", csv);
            std::cout << "suite_results=" << results.size() << "\n";
            return 0;
        }
        if (opt.input_path.empty()) throw std::runtime_error("--input is required");
        if (opt.out_path.empty()) opt.out_path = "results/raw/result.json";
        if (opt.log_path.empty()) opt.log_path = "results/logs/result.log";
        Instance inst = parseInstanceFile(opt.input_path, opt.route_time_limit, opt.pickup_time, opt.drop_time);
        Result r;
        if (opt.algorithm == "incremental-test") {
            r = runIncrementalTest(inst, opt);
        } else if (opt.algorithm == "cplex-fixed-route") {
            auto routes = opt.route_json_path.empty()
                ? (opt.route_generator == "random" ? makeRandomRoutes(inst, opt, 0)
                                                    : makeDeterministicRoutes(inst, opt.route_length_limit))
                : readRoutesFromJson(opt.route_json_path);
            r = solveCplexFixedRoute(inst, opt, routes);
        } else if (opt.algorithm == "profile-bpc") {
            auto routes = opt.route_json_path.empty()
                ? (opt.route_generator == "random" ? makeRandomRoutes(inst, opt, 0)
                                                    : makeDeterministicRoutes(inst, opt.route_length_limit))
                : readRoutesFromJson(opt.route_json_path);
            r = solveFixedRoutes(inst, opt, routes);
            r.algorithm = "profile-bpc";
            r.status = r.exact_for_fixed_routes ? "profile_bpc_exact_bb_optimal" : r.status;
        } else {
            auto routes = opt.route_json_path.empty()
                ? (opt.route_generator == "random" ? makeRandomRoutes(inst, opt, 0)
                                                    : makeDeterministicRoutes(inst, opt.route_length_limit))
                : readRoutesFromJson(opt.route_json_path);
            r = solveFixedRoutes(inst, opt, routes);
        }
        writeText(opt.out_path, toJson(r));
        writeText("results/load_exact_summary.csv", csvHeader() + csvRow(r));
        writeText("results/load_cplex_comparison.csv", csvHeader() + csvRow(r));
        writeText("results/incremental_eval_summary.csv", csvHeader() + csvRow(r));
        writeText(opt.log_path, "status=" + r.status + "\nobjective=" + std::to_string(r.objective) + "\n");
        std::cout << r.status << " objective=" << r.objective << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
