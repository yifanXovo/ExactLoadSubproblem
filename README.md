Experimental exact fixed-route loading project next to ExactEBRP.

This project isolates the loading subproblem: vehicle routes are fixed, and the
solver chooses pickup/drop quantities exactly for those routes. It reads the
ExactEBRP instance text format; the parser/objective conventions are adapted
from ExactEBRP for comparability.

Build:

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Fallback:

```powershell
D:\msys64\ucrt64\bin\g++.exe -std=c++17 -O2 -Wall -Wextra -Iinclude src/Common.cpp src/LoadSolver.cpp src/main.cpp -o build\ExactLoadSubproblem.exe
```

Example:

```powershell
build\ExactLoadSubproblem.exe --input ..\ExactEBRP\reference\generated\regen_V8_M2_average.txt --algorithm profile-dp --out results\raw\v8_profile_dp.json --log results\logs\v8_profile_dp.log
```

Round 2 exact validation campaign:

```powershell
build\ExactLoadSubproblem.exe --round2-suite --exactebrp-root ..\ExactEBRP --cplex-time-limit 300 --cplex-gap 0 --seed 42
```

Single fixed-route run:

```powershell
build\ExactLoadSubproblem.exe --input ..\ExactEBRP\testdata\examples\gcap_smoke_V4_M1.txt --algorithm cplex-fixed-route --profile-exact true --allow-natural-mode-pruning false --route-json results\round2_exact_loading\routes\gcap_smoke_V4_M1_rs0.json --out results\round2_exact_loading\raw\single_cplex.json --log results\round2_exact_loading\logs\single_cplex.log
```

Implemented approaches:

- `profile-dp`: exact route-profile enumeration for the fixed-route set when
  `--profile-exact true`, `--allow-natural-mode-pruning false`, and no profile
  limit truncates generation.
- `cplex-fixed-route`: CPLEX candidate-MIP validation over the completely
  enumerated fixed-route loading candidates. If the candidate list is truncated,
  the row is not treated as certified.
- `incremental-test`: recomputes only changed route profiles and updates Gini
  pair terms in O(|Delta| V), then checks against full recomputation.
- `cplex-fixed-route`: placeholder row unless an external CPLEX fixed-route
  MILP result is supplied. The C++ profile solver is the correctness baseline
  for this standalone prototype.

For V4/V8/V10 generated route sets, profile enumeration is exact for the fixed
routes. For larger route sets, the program records whether natural-mode pruning
was used and labels rows diagnostic if full profile coverage was not affordable.
