# ExactLoadSubproblem Algorithm Report

## 1. Problem Definition

This project fixes vehicle routes and solves the exact loading subproblem:
choose pickup/drop quantities along each fixed route to minimize
`G(Y)+lambda P(Y)` subject to station capacity, truck capacity, route duration,
station operation mode, depot unloading, and final inventory consistency.

## 2. Fixed-Route Formulation

For route position `t` on route `k`, variables are `p_kt`, `d_kt`, load
`L_kt`, depot unload `u_k`, and final station inventory `Y_i`.

Constraints:

- `L_kt = L_k,t-1 + p_kt - d_kt`
- `0 <= L_kt <= Q_k`
- `0 <= Y_i <= C_i`
- operation only at the station appearing at route position `t`
- route duration equals travel plus pickup handling plus station-drop and
  depot-unload handling
- `u_k = sum_t p_kt - sum_t d_kt`
- objective is exact `G(Y)+lambda P(Y)`.

## 3. Algorithms

### Profile DP

For each route, the solver enumerates feasible loading profiles:

- current route position;
- truck load;
- cumulative pickup/drop and route-duration budget;
- sparse net inventory vector `q`.

Dominance keeps one profile per net `q` vector with minimum duration. After
per-route profile generation, a cross-route combination step evaluates exact
final inventories and objective.

### Incremental Evaluator

The route profile set is cached by route signature. For a single-route change,
unchanged route profile sets are reused. Objective updates are checked against
full recomputation; pairwise Gini changes are handled by removing and re-adding
pair contributions for changed stations, giving `O(|Delta|V)` update work.

## 4. Validity Proof Sketches

The route-profile DP is exact for a fixed route when every feasible pickup/drop
quantity choice is enumerated and only load/duration/station-capacity feasible
states are retained. Combining one profile per route enumerates the Cartesian
product of all fixed-route loading decisions. The objective is computed from the
resulting final inventory vector, so the best profile combination is exactly the
fixed-route optimum. Dominance by identical `q` vector is valid because profiles
with identical inventory effect and no longer duration are interchangeable.

Incremental evaluation is valid because unchanged route profile sets have the
same feasible loading profile universe. For Gini, only pairs touching changed
stations change, so updating those pair contributions and leaving all other
pairs unchanged is algebraically equivalent to full recomputation.

## 5. Experimental Setup

The suite used V4 smoke, regenerated V8/V10, regenerated V12 M2 candidate, and
generated V20 M2 instances. For V4/V8/V10, profile enumeration is exact for the
fixed deterministic route set. V12/V20 use natural-mode/profile-budget pruning
and are labeled diagnostic.

## 6. Results

Key rows from `results/load_exact_summary.csv`:

| instance | algorithm | status | objective | runtime ms | profiles after dominance | verifier |
| --- | --- | --- | ---: | ---: | ---: | --- |
| V4 smoke | profile-dp | fixed_route_optimal | 0.222352941176 | 0.4762 | 375 | pass |
| V8 M2 average | profile-dp | fixed_route_optimal | 0.47263579148 | 188.4265 | 2546 | pass |
| V10 M2 average | profile-dp | fixed_route_optimal | 0.650898091836 | 218.8869 | 3304 | pass |
| V12 M2 average | profile-dp | fixed_route_diagnostic_pruned_profiles | 0.965363881082 | 0.3348 | 74 | pass |
| V20 M2 average | profile-dp | fixed_route_diagnostic_pruned_profiles | 1.04697180795 | 4.0432 | 263 | pass |

Incremental tests matched full recomputation on all rows. Speedup was strongest
on V4 (`3.13x`) and modest on V8/V10/V20 because the current deterministic
move sequence repeatedly changes short routes and the profile sets are already
small.

## 7. Interpretation

The fixed-route loading direction is promising as a component for HGA/TGBC,
ALNS, or local search: exactness is easy to preserve for small fixed route
sets, verifier behavior is clean, and cache reuse is simple. It is not a global
exact solver by itself because routes are fixed, but it gives a realistic path
to combine high-quality route search with exact loading evaluation.

