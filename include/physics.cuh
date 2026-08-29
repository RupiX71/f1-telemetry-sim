#ifndef PHYSICS_CUH
#define PHYSICS_CUH

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

// the data that goes into the gpu
struct CarSetup {
    int id;                             // setup id
    float mass_kg;                      // Mass of the car
    float ice_power_kw;                 // Internal Combustion Engine Power
    float mguk_power_kw;                // Eletric Engine Power
    float drag_coef;                    // Aerodynamic Coefficient
};

// Keeps the state of the car every dt
struct CarState {
    float v;                            // Velocity of the car
    int current_seg;                    // Position of the car (segment of the circuit)
    float current_m;                    // Current meter of the circuit
    float battery_mj;                   // Ammount of battery
    bool is_braking;                    // braking
    float time_s;                       // Time on track (resets after going through finish line)
};

// track segment
struct TrackSegment {
    float length_m;                     // Length of the segment
    float radius_m;                     // Radius of the segment (< 10000 means curvature)
    float x;                            // x of segment
    float y;                            // y of segment
};

// Results
struct SimResult {
    int setup_id;                       // Setup Id
    float lap_time;                     // Lap Time
    float top_speed_kmh;                // Top Speed
    float battery_used_mj;              // Ammount of Battery Used in MJ (starting + what is regenerated)
};

CUDA_CALLABLE void step_physics(CarState* state, const CarSetup* setup, const TrackSegment* track, int num_segments, float dt);

// This will start the kernel
void run_simulation_batch(const CarSetup* setups, SimResult* results, const TrackSegment* track, int num_segments, int numSetups);

#endif