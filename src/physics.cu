#include "physics.cuh"
#include <cuda_runtime.h>
#include <iostream>


// Cuda Kernel: Executed by thousands of threads
__global__ void simulate_acceleration(const CarSetup* setups, SimResult* results, int num_setups) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < num_setups) {
        CarSetup setup = setups[idx];


        // Kinetic Energy = 0.5 * mass * vel²
        // 100km/h = 27.77m/s
        float target_vel = 27.77f;
        float kinetic_energy = 0.5f * setup.mass_kg * (target_vel * target_vel);

        // power = work / time -> time = work / power
        // convert kW to W
        float power_w = setup.engine_power_kw * 1000.f;

        results[idx].setup_id = setup.id;
        results[idx].time_to_100_s = kinetic_energy / power_w; // ideal time
    }
}

// Host functions
void run_simulation_batch(const CarSetup* setups, SimResult* results, int num_setups) {
    CarSetup* d_setups;
    SimResult* d_results;

    size_t setups_size = num_setups * sizeof(CarSetup);
    size_t results_size = num_setups * sizeof(SimResult);

    cudaMalloc(&d_setups, setups_size);
    cudaMalloc(&d_results, results_size);

    cudaMemcpy(d_setups, setups, setups_size, cudaMemcpyHostToDevice);

    int threads_per_block = 256;
    int blocks_per_grid = (num_setups + threads_per_block - 1) / threads_per_block;

    simulate_acceleration<<<blocks_per_grid, threads_per_block>>>(d_setups, d_results, num_setups);

    cudaDeviceSynchronize();

    cudaMemcpy(results, d_results, results_size, cudaMemcpyDeviceToHost);

    cudaFree(d_setups);
    cudaFree(d_results);
}