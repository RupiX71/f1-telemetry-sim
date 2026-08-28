#ifndef PHYSICS_CUH
#define PHYSICS_CUH

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

// the data that goes into the gpu
struct CarSetup {
    int id;
    float mass_kg;
    float ice_power_kw;                 // Internal Combustion Engine Power
    float mguk_power_kw;                // Eletric Engine Power
    float drag_coef;                    // Aerodynamic Coefficient
};

// Keeps the state of the car every dt
struct CarState {
    float v;                            // 
    float battery_mj;                   //
    float dist_in_seg;                  //
    int current_idx;                    //
    bool is_braking;                    //
    float time_s;                       //
};

// track segment
struct TrackSegment {
    float length_m;                     // 
    float radius_m;                     //
    float x;                            //
    float y;                            // 
};

// Results
struct SimResult {
    int setup_id;
    float lap_time;                     // Time it takes for 1000m straight line
    float top_speed_kmh;                // speed at 1000m
    float battery_left_mj;              // battery left after 1000m 
};

CUDA_CALLABLE void step_physics(CarState* state, const CarSetup* setup, const TrackSegment* track, int num_segments, float dt);

// This will start the kernel
void run_simulation_batch(const CarSetup* setups, SimResult* results, const TrackSegment* track, int num_segments, int numSetups);

#endif