#include "physics.cuh"
#include <cuda_runtime.h>
#include <math.h>
#include <iostream>


// Cuda Kernel
__global__ void simulate_lap(const CarSetup* setups, SimResult* results, const TrackSegment* track, int num_segments, int num_setups) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < num_setups) {
        CarSetup setup = setups[idx];
        
        float v = 80.f;                          // vel
        float t = 0.0f;                          // t
        float dt = 0.01f;                       // delta (10ms)
        float battery_mj = 4.0f;                // initial battery 4MJ
        float max_speed_reached = 0.0f;

        float air_density = 1.225f;
        float frontal_area = 1.5f;
        float mu = 1.6f;

        float downforce_coef = setup.drag_coef * 3.0f;

        for ( int i = 0 ; i < num_segments ; ++i) {
            TrackSegment seg = track[i];
            float distance_in_segment = 0.0f;

            while (distance_in_segment < seg.length_m) {
                float drag_force = 0.5f * air_density * (v * v) * setup.drag_coef * frontal_area;
                float downforce = 0.5f * air_density * (v * v) * downforce_coef * frontal_area;

                float max_grip_force = (setup.mass_kg * 9.81f + downforce) * mu;
                
                float max_v = sqrtf((max_grip_force * seg.radius_m) / setup.mass_kg);

                int lookahead_idx = (i + 15 < num_segments) ? i + 15 : num_segments -1;
                float next_radius = track[lookahead_idx].radius_m;
                float next_max_v = sqrtf((max_grip_force * next_radius) / setup.mass_kg);

                float net_force = 0.0f;

                if (v >= max_v || v >= next_max_v) {
                    net_force = -(setup.mass_kg * 9.81 * 5.0f) - drag_force;
                    
                    // This should regenerate during braking
                    if (battery_mj < 4.0f) {
                        battery_mj += (350.0f * dt) / 1000.f;
                    }
                } else {
                    // accelerate
                    float current_power_kw = setup.ice_power_kw;

                    if (battery_mj > 0.0f && v > 30.f) {
                        current_power_kw += setup.mguk_power_kw;
                        battery_mj -= (setup.mguk_power_kw * dt) / 1000.f;
                    }

                    // Engine Force (F = P / v)
                    float engine_force = (v > 1.0f) ? (current_power_kw * 1000.f) / v : 15000.0f;

                    if (engine_force > max_grip_force) {
                        engine_force = max_grip_force;
                    }

                    net_force = engine_force - drag_force;
                }

                // newton's 2nd law
                float a = net_force / setup.mass_kg;
                v += a * dt;

                if (v < 5.0f) v = 5.0f;

                if (v > max_speed_reached) max_speed_reached = v;

                distance_in_segment += v * dt;
                t += dt;
            }
        }
        // Guardar os resultados
        results[idx].setup_id = setup.id;
        results[idx].lap_time = t;
        results[idx].top_speed_kmh = max_speed_reached * 3.6f;
        results[idx].battery_left_mj = battery_mj;
    }
}

// Host functions
void run_simulation_batch(const CarSetup* setups, SimResult* results, const TrackSegment* track, int num_segments, int num_setups) {
    CarSetup* d_setups;
    TrackSegment* d_track;
    SimResult* d_results;
    
    size_t setups_size = num_setups * sizeof(CarSetup);
    size_t track_size = num_segments * sizeof(TrackSegment);
    size_t results_size = num_setups * sizeof(SimResult);

    cudaMalloc(&d_setups, setups_size);
    cudaMalloc(&d_track, track_size);
    cudaMalloc(&d_results, results_size);

    cudaMemcpy(d_setups, setups, setups_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_track, track, track_size, cudaMemcpyHostToDevice);

    int threads_per_block = 256;
    int blocks_per_grid = (num_setups + threads_per_block - 1) / threads_per_block;

    simulate_lap<<<blocks_per_grid, threads_per_block>>>(d_setups, d_results, d_track, num_segments, num_setups);

    cudaDeviceSynchronize();

    cudaMemcpy(results, d_results, results_size, cudaMemcpyDeviceToHost);

    cudaFree(d_setups);
    cudaFree(d_track);
    cudaFree(d_results);
}