$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force -Path build,results/raw,results/logs | Out-Null
D:\msys64\ucrt64\bin\g++.exe -std=c++17 -O2 -Wall -Wextra -Iinclude src\Common.cpp src\LoadSolver.cpp src\main.cpp -o build\ExactLoadSubproblem.exe
build\ExactLoadSubproblem.exe --input ..\ExactEBRP\testdata\examples\gcap_smoke_V4_M1.txt --algorithm profile-dp --out results\raw\v4_profile_dp.json --log results\logs\v4_profile_dp.log
build\ExactLoadSubproblem.exe --run-suite --exactebrp-root ..\ExactEBRP --profile-limit 12000

