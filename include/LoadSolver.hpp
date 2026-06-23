#pragma once

#include "Common.hpp"

#include <filesystem>
#include <map>

namespace load_exact {

Result solveFixedRoutes(const Instance& instance, const RunOptions& options,
                        const std::vector<RoutePlan>& fixed_routes);
Result solveCplexFixedRoute(const Instance& instance, const RunOptions& options,
                            const std::vector<RoutePlan>& fixed_routes);
Result runIncrementalTest(const Instance& instance, const RunOptions& options);
std::vector<RoutePlan> makeDeterministicRoutes(const Instance& instance, int length_limit);
std::vector<RoutePlan> makeRandomRoutes(const Instance& instance, const RunOptions& options,
                                        int route_set_index);
std::vector<RoutePlan> readRoutesFromJson(const std::filesystem::path& path);
void writeRoutesToJson(const std::filesystem::path& path,
                       const std::vector<RoutePlan>& routes,
                       const std::string& route_set_id);
std::vector<Result> runDefaultSuite(const RunOptions& options);
std::vector<Result> runRound2Suite(const RunOptions& options);

} // namespace load_exact
