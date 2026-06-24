# Exact Fixed-Route Loading Solver

## Problem Definition

Given fixed vehicle routes `R_k=(0,i_1,...,i_m,0)`, choose integer pickup and
drop quantities to minimize `G(Y)+lambda P(Y)` subject to station capacity,
truck capacity, route duration, operation-mode, depot unloading, and final
inventory consistency. Routes are fixed; only loading decisions are optimized.

## True Fixed-Route CPLEX MILP

Round 3 replaces the old profile candidate-MIP validation with a true
fixed-route loading MILP. For each route visit `(k,t)` the model contains
integer `p_{k,t}`, `d_{k,t}`, load `L_{k,t}`, depot unload `u_k`, integer
station final inventory `Y_i`, continuous ratio `r_i`, absolute penalty
variables, and pairwise absolute-difference variables.

The constraints are:

- `L_{k,t}=L_{k,t-1}+p_{k,t}-d_{k,t}`;
- `0 <= L_{k,t} <= Q_k`;
- `u_k` equals the final route load;
- `Y_i = I_i - pickups_i + drops_i`;
- `0 <= Y_i <= C_i`;
- aggregate pickup/drop mode forbids using one station as both pickup and drop;
- fixed-route travel plus handling and depot unload time is within `T_k`;
- `r_i = Y_i / target_i`;
- absolute deviation and pairwise Gini numerator terms are linearized.

Because `G=H/(V*S)` with `S=sum_i r_i`, the implementation uses an exact
ratio-sum selector linearization. Inventory one-hot variables define all
allowed `Y_i` values, dynamic programming enumerates the possible values of
`S`, a selector chooses one `S`, and auxiliary variables activate the matching
`H/(V*S)` coefficient. This is a MILP over the fixed-route loading variables,
not a profile candidate selector. If the exact `S` selector set exceeds the
safety limit, the row is marked nonclosed and is not used as CPLEX ground truth.

## Exact Profile-DP Baseline

For each fixed route, exact profile generation recurses over route positions
with current load, cumulative pickup/drop quantities, net inventory vector
`q`, and duration. In exact mode:

- `--profile-exact true`;
- `--allow-natural-mode-pruning false`;
- `--profile-limit 0`.

The only dominance used in exact mode keeps the shortest-duration profile for
an identical `(q-vector, final_load)` key. This is valid because the profile has
the same contribution to every master constraint and objective term, and shorter
duration is never worse.

## Profile-BPC / Exact Master

The current `profile-bpc` implementation is an exact profile branch-and-bound
master over complete route profile sets. It chooses one profile per fixed route,
checks station capacities incrementally, and evaluates the exact objective at
leaves. It is exact only when every route profile set is complete and the master
search is not stopped by `--profile-bpc-time-limit` or
`--profile-bpc-max-nodes`.

This is not yet a scalable paper-level branch-price implementation. It is a
guaranteed exact baseline for rows where it closes, and diagnostic otherwise.
The next required improvement is true delayed profile pricing with valid lower
bounds so medium/long route sets do not require full profile generation.

## Incremental Exact Evaluation

The incremental evaluator caches complete route profile sets by route signature.
When one route changes, unchanged route profiles are reused and the exact master
is re-solved over cached plus regenerated profile sets. Objective-update checks
recompute `G` and `P` from final inventories and compare against full
recomputation. Rows are exact only when the cache-reuse solution matches full
exact recomputation.

## Round 3 Experimental Setup

Outputs are under `results/round3_paper_exact_loading/`.

Route sets:

- V4 smoke: 20 generated route sets plus two single smoke rows.
- Generated V8 M2: 20 route sets.
- Generated V10 M2: 20 route sets.
- V12 M2 candidate: 30 short, 30 medium, 20 long route sets.
- Generated V20 M2: 10 diagnostic route sets.

Every row writes raw JSON and logs. Summary files separate CPLEX MILP,
profile-BPC, profile-DP baseline, V12 medium rows, incremental rows, and
skipped/failed rows.

## Results

Round 3 generated 346 raw result rows. The true CPLEX MILP closed 55 rows with
gap 0 and verifier pass:

- V4: 21 closed rows.
- V8: 6 closed rows.
- V10: 6 closed rows.
- V12: 15 closed rows, all short-route rows.
- V20: 7 closed rows.

Profile-BPC matched every CPLEX-closed row within `1e-6` and had zero mismatch
rows. It solved 104 profile rows exactly and stopped 27 rows by the configured
time/node limit.

V12 medium routes were not skipped: 30 CPLEX MILP attempts and 30 profile-BPC
attempts were run. The CPLEX MILP did not close any medium row because the exact
ratio-sum selector set exceeded the safety limit. Profile-BPC closed 3 medium
rows and stopped 27 by time/node limit. V12 long CPLEX rows were generated and
attempted, but profile-BPC was intentionally skipped for long rows after medium
state explosion was observed.

Incremental exact cache-reuse tests passed on V4, V8, V10, and V12 short route
sets. Speedup was meaningful on V4 and weak on larger rows because exact master
re-solving still dominates.

## Recommendation

The project now contains a true fixed-route CPLEX MILP and an exact internal
profile solver for rows where profile generation and the master close. It is
not yet paper-level ready for medium/long V12 fixed routes. The most promising
next step is a real delayed profile-pricing branch-price master with stronger
lower bounds, not more heuristic pruning.
