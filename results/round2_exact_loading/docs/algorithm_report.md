# Exact Fixed-Route Loading Solver

## Problem Definition

Given fixed vehicle routes `R_k=(0,i_1,...,i_m,0)`, choose integer pickup and
drop quantities to minimize `G(Y)+lambda P(Y)` under station capacity, truck
capacity, route duration, operation-mode, depot unloading, and final-inventory
consistency. Routes are fixed; only loading decisions are optimized.

## Fixed-Route CPLEX Validation Model

The Gini objective is fractional in `Y` because `G=H/(V*S)`. Instead of
pretending this is a plain linear MILP, the implementation uses a certified
fallback validation model:

1. Enumerate every feasible fixed-route loading profile exactly.
2. Enumerate every feasible cross-route final-inventory candidate.
3. Write a CPLEX binary candidate-MIP:
   `min sum_c obj_c z_c`, `sum_c z_c=1`, `z_c binary`.

When candidate enumeration is complete, this MIP is equivalent to the
fixed-route loading problem and CPLEX gap 0 certifies the same optimum. If the
candidate list is truncated, the row is marked
`candidate_mip_truncated_not_closed` and is excluded from exact agreement
claims.

## Exact Profile-DP

For each route, profile generation recurses over route positions with state:

- current position;
- current truck load;
- cumulative pickup and station drop quantities;
- net final-inventory contribution vector `q`;
- route duration.

In exact mode:

- `--profile-exact true`;
- `--allow-natural-mode-pruning false`;
- `--profile-limit 0`;
- all integer pickup/drop quantities are enumerated.

Profile dominance is exact only for identical `q` vector and final depot load:
if two profiles have the same `(q, final_load)`, the shorter-duration one
dominates because it has identical objective and combination effects and no
worse feasibility.

## Exact Profile Branch-And-Bound

The current `profile-bpc` CLI maps to the exact profile branch-and-bound master:
one profile is selected per fixed route, station capacities are checked after
each partial assignment, and the exact objective is evaluated at leaves. This is
not a heuristic when all route profile sets are complete.

## Diagnostic Conditions

Rows are diagnostic, not exact, when:

- natural-mode pruning is enabled;
- `profile-limit` truncates profile generation;
- the CPLEX candidate-MIP is truncated;
- route generation asks for route sizes that make exact enumeration infeasible
  within the local campaign budget.

## Incremental Evaluation

The incremental path caches exact route profile sets by route signature. If one
route changes, unchanged route profiles are reused and only the changed route is
regenerated. Objective update maintains final-inventory ratios and recomputes
the affected objective exactly; every test compares against full exact
recomputation.

## Benchmark Setup

Round 2 outputs are under `results/round2_exact_loading/`.

Route sets:

- V4 smoke: 20 route sets.
- generated V8 M2: 20 route sets.
- generated V10 M2: 20 route sets.
- V12 M2 candidate: 50 short route sets.
- generated V20 M2: 10 short route sets.

Medium and long V12 route sets were attempted in the previous prototype run and
caused exact profile enumeration state explosion; they are recorded in
`skipped_or_failed_rows.csv` and not reported as exact.

## Results vs CPLEX

Summary files:

- `summaries/cplex_fixed_route_summary.csv`
- `summaries/exact_algorithm_comparison.csv`
- `summaries/v12_m2_route_set_comparison.csv`
- `summaries/incremental_exact_summary.csv`

Closed validation counts:

- CPLEX candidate-MIP closed rows: 118.
- Exact profile rows: 120.
- Exact profile rows with invalid exact claim: 0.
- V12 rows excluded due candidate-MIP truncation: 2.

The profile solver matches every closed CPLEX validation row within `1e-6` and
passes the independent verifier.

## Recommendation For HGA/TGBC

Fixed-route exact loading is promising as an incumbent-quality component. It
should be integrated as:

1. exact evaluator for short and medium route sets;
2. cache-backed incremental evaluator for local-search moves;
3. diagnostic evaluator with explicit pruning labels for longer routes until
   stronger profile dominance or decomposition is added.

