cmake_minimum_required(VERSION 3.16)
project(ExactLoadSubproblem LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ExactLoadSubproblem
    src/Common.cpp
    src/LoadSolver.cpp
    src/main.cpp
)

target_include_directories(ExactLoadSubproblem PRIVATE include)

