# Round 2 Exact Loading Assessment

## Is Fixed-Route Exact Loading Promising?

Yes, for short fixed routes. Exact profile enumeration plus CPLEX candidate-MIP
validation matched every closed row within tolerance and gives a clean
integration path for HGA/TGBC or ALNS route search.

## Practical Route Sizes

The exact mode is practical for the tested V4/V8/V10 route sets with route
lengths 1-2 or small deterministic routes. V12 short route sets are mostly
practical: 48 of 50 profile rows had closed CPLEX validation. Medium/long V12
route sets caused state explosion in this local campaign and remain TODO.

## Does The Exact Algorithm Match CPLEX?

For rows where `cplex_status=candidate_mip_optimal_gap0`, yes. The audit found
zero rows where `exact_for_fixed_routes=true`, verifier passed, and objective
difference exceeded `1e-6`.

Two V12 rows hit the CPLEX candidate-MIP candidate collection cap and were
marked `candidate_mip_truncated_not_closed`; they are not used as exact
agreement claims.

## Is Profile-BPC Needed?

The current exact profile branch-and-bound is sufficient for short routes.
Profile-BPC/column generation becomes necessary for medium/long routes because
full profile enumeration can explode before CPLEX validation.

## Is Incremental Exact Evaluation Useful?

The V4 100-move incremental test matched full recomputation and showed about
2.24x cache-reuse speedup. Larger route sets need a bounded exact incremental
campaign; V10 100-move exact incremental was skipped after the first full
matrix run timed out.

## Next HGA/TGBC Integration

1. Export HGA/TGBC route sets into the project route JSON schema.
2. Use exact profile loading for route sets whose profile generation is closed.
3. Fall back to diagnostic/pruned loading only with explicit labels.
4. Add stronger dominance and route-position lower bounds before attempting
   V12 medium/long exact route sets again.

