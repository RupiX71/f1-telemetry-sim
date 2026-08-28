#include "physics.cuh"
#include <cuda_runtime.h>
#include <math.h>
#include <iostream>

__host__ __device__ void step_physics(CarState* state, const CarSetup* setup, 
                                      const TrackSegment* track, int num_segments, 
                                      float dt) {
    float air_density = 1.225f;
    float frontal_area = 1.5f;
    float mu = 1.6f;
    float downforce_coef = setup->drag_coef * 3.0f;

    float drag_force = 0.5f * air_density * (state->v * state->v) * setup->drag_coef * frontal_area;
    float downforce = 0.5f * air_density * (state->v * state->v) * downforce_coef * frontal_area;
    float max_grip = (setup->mass_kg * 9.81f + downforce) * mu;

    int cur = state->current_idx;
    float max_v = sqrtf((max_grip * track[cur].radius_m) / setup->mass_kg);
    
    int lookahead = (cur + 15) % num_segments;
    float next_max_v = sqrtf((max_grip * track[lookahead].radius_m) / setup->mass_kg);

    float net_force = 0.0f;
    state->is_braking = false;

    if (state->v >= max_v || state->v >= next_max_v) {
        net_force = -(setup->mass_kg * 9.81f * 5.0f) - drag_force;
        state->is_braking = true;
        if (state->battery_mj < 4.0f) state->battery_mj += (350.0f * dt) / 1000.0f; 
    } else {
        float current_power_kw = setup->ice_power_kw;
        if (state->battery_mj > 0.0f && state->v > 30.0f) {
            current_power_kw += setup->mguk_power_kw;
            state->battery_mj -= (setup->mguk_power_kw * dt) / 1000.0f;
        }
        float engine_force = (state->v > 1.0f) ? (current_power_kw * 1000.0f) / state->v : 15000.0f;
        if (engine_force > max_grip) engine_force = max_grip;
        net_force = engine_force - drag_force;
    }

    float a = net_force / setup->mass_kg;
    state->v += a * dt;
    if (state->v < 5.0f) state->v = 5.0f; 

    state->dist_in_seg += state->v * dt;
    while (state->dist_in_seg >= track[state->current_idx].length_m) {
        state->dist_in_seg -= track[state->current_idx].length_m;
        state->current_idx++;
        if (state->current_idx >= num_segments) return; 
    }
}

// Cuda Kernel
__global__ void simulate_lap(const CarSetup* setups, SimResult* results, int num_setups, const TrackSegment* track, int num_segments) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < num_setups) {
        CarSetup setup = setups[idx];
        
        CarState state;
        state.v = 10.0f;          
        state.battery_mj = 4.0f; 
        state.dist_in_seg = 0.0f;
        state.current_idx = 0;
        
        float t = 0.0f;
        float dt = 0.016f;
        float max_speed_reached = 0.0f;
        
        while (state.current_idx < num_segments - 1) {
            step_physics(&state, &setup, track, num_segments, dt);
            
            if (state.v > max_speed_reached) max_speed_reached = state.v;
            t += dt;
        }
        
        results[idx].setup_id = setup.id;
        results[idx].lap_time = t;
        results[idx].top_speed_kmh = max_speed_reached * 3.6f; 
        results[idx].battery_left_mj = state.battery_mj;
    }
}

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

    simulate_lap<<<blocks_per_grid, threads_per_block>>>(d_setups, d_results, num_setups, d_track, num_segments);

    cudaDeviceSynchronize();

    cudaMemcpy(results, d_results, results_size, cudaMemcpyDeviceToHost);

    cudaFree(d_setups);
    cudaFree(d_track);
    cudaFree(d_results);
}