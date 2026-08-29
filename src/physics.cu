#include "physics.cuh"
#include "config.cuh"
#include <cuda_runtime.h>
#include <math.h>
#include <iostream>

__host__ __device__ void step_physics(CarState* car, const CarSetup* setup, const TrackSegment* track, int num_segments, float dt) {

    float drag_force = 0.5f * Config::AIR_DENSITY * (car->v * car->v) * setup->drag_coef * Config::FRONTAL_AREA;
    float downforce = 0.5f * Config::AIR_DENSITY * (car->v * car->v) * (setup->drag_coef * 3.f) * Config::FRONTAL_AREA;

    float max_grip = (setup->mass_kg * Config::GRAVITY + downforce) * Config::BASE_MECH_GRIP;

    float speed = sqrtf((max_grip * track[car->current_seg].radius_m) / setup->mass_kg);

    float dist_to_curve = track[car->current_seg].length_m - car->current_m;

    for (int i = 0 ; i <= Config::LOOKAHEAD_METERS ; ++i) {
        int lookahead = (car->current_seg + i) % num_segments;

        if (track[lookahead].radius_m < 10000.0f) {
            float corner_grip = (setup->mass_kg * Config::GRAVITY) * Config::BASE_MECH_GRIP;
            float corner_v = sqrtf((corner_grip * track[lookahead].radius_m) / setup->mass_kg);
            float v_critical = sqrtf((corner_v * corner_v) + (2.0f * Config::DECEL_RATE * dist_to_curve));  // torricelli

            if (v_critical < speed) {
                speed = v_critical;
            }
        }
        dist_to_curve += track[lookahead].length_m;
    }

    car->is_braking = false;
    float net_force = 0.0f;
    if (car->v > speed) {
        car->is_braking = true;
        net_force = -(setup->mass_kg * Config::DECEL_RATE) - drag_force; 
        
        if (car->battery_mj < Config::MAX_BATTERY_MJ) {
            car->battery_mj += (Config::MGUK_REGEN_KW * dt) / 1000.0f;
        }
    } else {
        float current_power_kw = setup->ice_power_kw;
        if (car->battery_mj > 0.0f && car->v > 30.0f) {
            current_power_kw += setup->mguk_power_kw;
            car->battery_mj -= (setup->mguk_power_kw * dt) / 1000.0f;
        }
        
        float engine_force = (car->v > 1.0f) ? (current_power_kw * 1000.0f) / car->v : 15000.0f;
        
        if (engine_force > max_grip) engine_force = max_grip; 
        
        net_force = engine_force - drag_force; 
    }

    float a = net_force / setup->mass_kg;
    car->v += a * dt;
    car->current_m += car->v * dt;
    car->time_s += dt;

    if (car->current_m >= track[car->current_seg].length_m) {
        car->current_m -= track[car->current_seg].length_m;
        car->current_seg++;
    }
}

// Cuda Kernel
__global__ void simulate_lap(const CarSetup* setups, SimResult* results, int num_setups, const TrackSegment* track, int num_segments) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < num_setups) {
        CarSetup setup = setups[idx];
        CarState state;

        // Starter states for each setup
        state.v = 10.0f;
        state.battery_mj = 4.0f;
        state.current_seg = 0;
        state.time_s = 0.0f;
        
        float t = 0.0f;
        float dt = 0.002f;
        float max_speed = 0.0f;
        
        while (state.current_seg < num_segments - 1) {
            step_physics(&state, &setup, track, num_segments, dt);
            
            if (state.v > max_speed) max_speed = state.v;
            t += dt;
        }
        
        results[idx].setup_id = setup.id;
        results[idx].lap_time = t;
        results[idx].top_speed_kmh = max_speed * 3.6f; 
        results[idx].battery_used_mj = state.battery_mj;
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