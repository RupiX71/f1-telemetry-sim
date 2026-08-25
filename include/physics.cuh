#ifndef PHYSICS_CUH
#define PHYSICS_CUH

// the data that goes into the gpu
struct CarSetup {
    int id;
    float mass_kg;
    float ice_power_kw;                 // Internal Combustion Engine Power
    float mguk_power_kw;                // Eletric Engine Power
    float drag_coef;                    // Aerodynamic Coefficient
};

// Results
struct SimResult {
    int setup_id;
    float time_to_1000m;                 // Time it takes for 1000m straight line
    float top_speed_kmh;                // speed at 1000m
    float battery_left_mj;              // battery left after 1000m 
};

// This will start the kernel
void run_simulation_batch(const CarSetup* setups, SimResult* results, int numSetups);

#endif