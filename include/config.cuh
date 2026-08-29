#ifndef CONFIG_CUH
#define CONFIG_CUH

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

namespace Config {
    // Physics and ambient
    constexpr float AIR_DENSITY = 1.225f;
    constexpr float FRONTAL_AREA = 1.5f;
    constexpr float GRAVITY = 9.81f;

    // Car Limits
    constexpr float DECEL_RATE = 40.0f; // this one should be variable no?
    constexpr float BASE_MECH_GRIP = 1.6f;
    constexpr float ICE_MIN_FORCE = 15000.f;

    // ERS System (MGU-K)
    constexpr float MAX_BATTERY_MJ = 4.0f;
    constexpr float MGUK_REGEN_KW = 350.0f;

    // Simulation Parameters
    constexpr int LOOKAHEAD_METERS = 300;
    constexpr float PHYSICS_DT = 0.002f;
    constexpr int NUM_SETUPS = 5000;
}

#endif