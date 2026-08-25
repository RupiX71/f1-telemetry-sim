#include "physics.cuh"
#include <cuda_runtime.h>
#include <iostream>


// Cuda Kernel: Executed by thousands of threads
__global__ void simulate_straight(const CarSetup* setups, SimResult* results, int num_setups) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < num_setups) {
        CarSetup setup = setups[idx];
        
        float v = 0.f;                          // vel
        float x = 0.f;                          // pos
        float t = 0.f;                          // t
        float dt = 0.01f;                       // delta (10ms)
        float battery_mj = 4.0f;                // initial battery 4MJ

        float air_density = 1.225f;
        float frontal_area = 1.5f;

        while(x < 1000.0f) {
            float drag_force = 0.5f * air_density * (v * v) * setup.drag_coef * frontal_area;
            
            float current_power_kw = setup.ice_power_kw;
            
            if (battery_mj > 0.0f) {
                current_power_kw += setup.mguk_power_kw;
                
                battery_mj -= (setup.mguk_power_kw * dt) / 1000.0f; 
            }
            
            float engine_force = 0.0f;
            if (v > 1.0f) {
                engine_force = (current_power_kw * 1000.0f) / v; 
            } else {
                engine_force = 15000.0f; // Limite de tração na 1ª mudança (grip dos pneus)
            }
            
            float net_force = engine_force - drag_force;
            float a = net_force / setup.mass_kg;
            
            v += a * dt;
            x += v * dt;
            t += dt;
        }
        
        // Guardar os resultados
        results[idx].setup_id = setup.id;
        results[idx].time_to_1000m = t;
        results[idx].top_speed_kmh = v * 3.6f;
        results[idx].battery_left_mj = battery_mj;
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

    simulate_straight<<<blocks_per_grid, threads_per_block>>>(d_setups, d_results, num_setups);

    cudaDeviceSynchronize();

    cudaMemcpy(results, d_results, results_size, cudaMemcpyDeviceToHost);

    cudaFree(d_setups);
    cudaFree(d_results);
}