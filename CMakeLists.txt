cmake_minimum_required(VERSION 3.18)

set(CMAKE_CUDA_COMPILER "/usr/local/cuda-13.2/bin/nvcc")

set(CMAKE_CUDA_ARCHITECTURES 75)

project(F1HybridSolver LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(f1_solver 
    src/main.cpp 
    src/physics.cu
)