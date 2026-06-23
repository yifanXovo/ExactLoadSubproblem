#include "Common.hpp"
#include "LoadSolver.hpp"

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
        else if (a == "--exactebrp-root") opt.exactebrp_root = value(i, argc, argv);
        else if (a == "--run-suite") opt.run_suite = true;
        else if (a == "--help") {
            std::cout << "ExactLoadSubproblem --input file --algorithm profile-dp|incremental-test|cplex-fixed-route --out json --log log\n";
            std::exit(0);
        } else throw std::runtime_error("Unknown argument: " + a);
    }
    return opt;
}

} // namespace

int main(int argc, char** argv) {
    try {
        RunOptions opt = parse(argc, argv);
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
            r.instance = inst.name;
            r.V = inst.V;
            r.M = inst.M;
            r.algorithm = opt.algorithm;
            r.status = "external_cplex_fixed_route_not_run_by_standalone_prototype";
            r.notes.push_back("Profile-DP solver is used as standalone exact fixed-route baseline; CPLEX row requires external model export.");
            r.result_file = opt.out_path;
            r.log_file = opt.log_path;
        } else {
            auto routes = makeDeterministicRoutes(inst, opt.route_length_limit);
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

