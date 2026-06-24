# Round 3 Paper Exact Loading Assessment

## Is This Paper-Level Ready?

No. Round 3 is a real exact-algorithm improvement because `cplex-fixed-route`
now builds a true fixed-route loading MILP, and `profile-bpc` no longer claims
exactness after time/node limits. However, the internal exact algorithm is still
not scalable enough for medium/long V12 fixed routes.

## Does It Match CPLEX On Closed Rows?

Yes. Across the round 3 campaign, CPLEX closed 55 rows with gap 0 and verifier
pass. `profile-bpc` matched every CPLEX-closed row within `1e-6`; there were no
objective mismatches among exact claims.

## What Route Lengths Are Solved Exactly?

Short routes are reliable. V4 closed fully; V8, V10, V12-short, and V20 have
closed subsets. Medium V12 routes were tested but are not broadly solved:
profile-BPC closed 3 medium rows and hit the configured time/node limit on 27.
Long V12 profile-BPC rows remain out of reach for the current implementation.

## Where Does State Explosion Remain?

Two places:

1. The true CPLEX MILP uses an exact ratio-sum selector for the fractional Gini
   denominator. For V12 medium/long rows, the selector set can exceed 200k
   values before CPLEX is called.
2. The internal profile-BPC still generates complete route profile sets before
   the exact master. Medium route profile sets can dominate runtime.

## Is BPC Necessary And Effective?

Yes, a real branch-price design is necessary. The current exact profile master
is a correctness baseline, not a scalable BPC. It proves that profile columns
are a sensible representation, but delayed pricing and stronger lower bounds
are needed before the method can be paper-level for V12 medium/long routes.

## Is Incremental Exact Evaluation Useful?

It is correct and useful as an integration primitive. Cache reuse matched full
exact recomputation on V4, V8, V10, and V12 short tests. The speedup is strong
only when route profile regeneration dominates; for larger rows, exact master
re-solving still dominates.

## What Remains Before Manuscript Integration?

1. Replace complete profile generation with delayed exact profile pricing.
2. Add valid lower bounds for the profile master.
3. Improve the true CPLEX MILP objective handling so medium V12 rows do not
   require enumerating huge ratio-sum selector sets.
4. Re-run medium and long V12 route sets with gap-0 CPLEX comparisons.
5. Integrate the exact cache evaluator into HGA/TGBC only for route sets whose
   profile solver closes.
