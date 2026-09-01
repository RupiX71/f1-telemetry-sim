#include "physics.cuh"
#include "config.cuh"
#include <cuda_runtime.h>
#include <math.h>
#include <iostream>

__host__ __device__ void step_physics(F1Car* car, const CarSetup* setup, const TrackSegment* track, int num_segments, float dt) {

    float drag_force = 0.5f * Config::AIR_DENSITY * (car->v * car->v) * setup->drag_coef * Config::FRONTAL_AREA;
    float downforce = 0.5f * Config::AIR_DENSITY * (car->v * car->v) * (setup->drag_coef * 3.f) * Config::FRONTAL_AREA;

    float max_grip = (setup->mass_kg * Config::GRAVITY + downforce) * Config::BASE_MECH_GRIP;

    float speed = sqrtf((max_grip * track[car->current_seg].radius_m) / setup->mass_kg);

    float dist_to_curve = track[car->current_seg].length_m - car->current_m;

    float base_corner_accel = Config::GRAVITY * Config::BASE_MECH_GRIP;
    
    for (int i = 1; i <= Config::LOOKAHEAD_METERS; ++i) {
        int lookahead = (car->current_seg + i) % num_segments;
        
        if (track[lookahead].radius_m < 10000.0f) {
            
            float corner_v_sq = base_corner_accel * track[lookahead].radius_m;
            
            // v_critical should be less than what it is rn
            float v_critical = sqrtf(corner_v_sq + (2.0f * Config::DECEL_RATE * dist_to_curve));
            
            if (v_critical < speed) {
                speed = v_critical;
            }
        }
        dist_to_curve += track[lookahead].length_m; 
    }

    float lift_threshold_speed = speed - 10.0f;

    if (car->v > speed) {
        car->action = DriverAction::BRAKE;
    } else if (car->v > lift_threshold_speed && car->qualifying_mode == false) {
        car->action = DriverAction::COAST;
    } else if (car->v < speed) {
        car->action = DriverAction::ACCELERATE;
    }

    // Kamm Circle: Lateral Force: F * v² / R
    float lateral_force = (setup->mass_kg * car->v * car->v) / track[car->current_seg].radius_m;

    // pythagorean theorem: sqrt(F² + L²) = max_grip
    float long_grip = 0.0f;
    if (max_grip > lateral_force) {
        long_grip = sqrtf(max_grip * max_grip - lateral_force * lateral_force);
    }

    car->throttle_pedal = 0.0f;
    car->brake_pedal = 0.0f;

    float net_force = 0.0f;


    switch (car->action) {
        case DriverAction::BRAKE: {
            float desired_braking_force = setup->mass_kg * Config::DECEL_RATE;
            float actual_braking_force = (desired_braking_force > long_grip) ? long_grip : desired_braking_force;

            car->brake_pedal = actual_braking_force / desired_braking_force;

            net_force = -(setup->mass_kg * Config::DECEL_RATE) - drag_force; 
            if (car->battery_mj < Config::MAX_BATTERY_MJ) {
                car->battery_mj += (Config::MGUK_REGEN_KW * dt) / 1000.0f;
            }
            break;
        }
        case DriverAction::COAST: {
            net_force = -drag_force;
            if (car->battery_mj < Config::MAX_BATTERY_MJ) {
                car->battery_mj += (Config::MGUK_REGEN_KW * dt) / 1000.0f;
            }
            break;
        }
        case DriverAction::ACCELERATE: {
            float current_power_kw = setup->ice_power_kw;
            if (car->battery_mj > 0.0f && car->v > 60.0f && !(track[car->current_seg].radius_m < 10000.0f)) {
                current_power_kw += setup->mguk_power_kw;
                car->battery_mj -= (setup->mguk_power_kw * dt) / 1000.0f;
            }
            float desired_engine_force = (car->v > 1.0f) ? (current_power_kw * 1000.0f) / car->v : 15000.0f;
            
            float actual_engine_force = (desired_engine_force > long_grip) ? long_grip : desired_engine_force;
            car->throttle_pedal = actual_engine_force / desired_engine_force;
        
            net_force = actual_engine_force - drag_force;
            break;
        }
    }

    float a = net_force / setup->mass_kg;
    car->v += a * dt;
    car->current_m += car->v * dt;
    car->time_s += dt;

    if (car->current_m >= track[car->current_seg].length_m) {
        car->current_m -= track[car->current_seg].length_m;
        car->current_seg++;
    }
    while (car->current_m >= track[car->current_seg].length_m) {
        car->current_m -= track[car->current_seg].length_m;
        car->current_seg++;
    }
}

// Cuda Kernel
__global__ void simulate_lap(const CarSetup* setups, SimResult* results, int num_setups, const TrackSegment* track, int num_segments) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < num_setups) {
        CarSetup setup = setups[idx];
        F1Car car;

        // Starter states for each setup
        car.v = track[0].real_speed_kmh / 3.6f;
        car.battery_mj = 4.0f;
        car.current_seg = 0;
        car.time_s = 0.0f;
        car.qualifying_mode = false; // Assuming qualifying mode is true for all setups initially
        
        float t = 0.0f;
        float dt = 0.002f;
        float max_speed = 0.0f;
        
        while (car.current_seg < num_segments - 1) {
            step_physics(&car, &setup, track, num_segments, dt);
            
            if (car.v > max_speed) max_speed = car.v;
            t += dt;
        }
        
        results[idx].setup_id = setup.id;
        results[idx].lap_time = t;
        results[idx].top_speed_kmh = max_speed * 3.6f; 
        results[idx].battery_used_mj = car.battery_mj;
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