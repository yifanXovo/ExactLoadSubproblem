#pragma once

#include "Common.hpp"

#include <map>

namespace load_exact {

Result solveFixedRoutes(const Instance& instance, const RunOptions& options,
                        const std::vector<RoutePlan>& fixed_routes);
Result runIncrementalTest(const Instance& instance, const RunOptions& options);
std::vector<RoutePlan> makeDeterministicRoutes(const Instance& instance, int length_limit);
std::vector<Result> runDefaultSuite(const RunOptions& options);

} // namespace load_exact

