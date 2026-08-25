#ifndef PHYSICS_CUH
#define PHYSICS_CUH

// the data that goes into the gpu
struct CarSetup {
    int id;
    float mass_kg;
    float engine_power_kw;
};

// this will be the return value
struct SimResult {
    int setup_id;
    float time_to_100_s;
};

// This will start the kernel
void run_simulation_batch(const CarSetup* setups, SimResult* results, int numSetups);

#endif