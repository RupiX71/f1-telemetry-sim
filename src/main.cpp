#include <iostream>
#include <vector>
#include "physics.cuh"

int main() {
    int num_setups = 100000;
    
    std::cout << "[CPU] Generating " << num_setups << " different setups..." << std::endl;

    std::vector<CarSetup> setups(num_setups);
    std::vector<SimResult> results(num_setups);

    for(int i = 0 ; i < num_setups ; ++i) {
        setups[i].id = i;
        setups[i].mass_kg = 798.0f; // minimal mass in f1 rn
        setups[i].ice_power_kw = 200.f + (i * (200.0f / num_setups));
        setups[i].mguk_power_kw = 200.f + (i * (150.f / num_setups));
        setups[i].drag_coef = 0.3f + (i * (0.5f / num_setups));
    }

    std::cout << "[CUDA] Sending Data to GPU..." << std::endl;

    run_simulation_batch(setups.data(), results.data(), num_setups);

    std::cout << "[CPU] Success here are the results: \n\n";

    std::cout << "Small Power / Small Drag: "<< std::endl;
    std::cout << "Time to 1000m: " << results[0].time_to_1000m << "s\n";
    std::cout << "Top speed: " << results[0].top_speed_kmh << "km/h\n";

    std::cout << "Big Power / Big Drag: "<< std::endl;
    std::cout << "Time to 1000m: " << results[num_setups - 1].time_to_1000m << "s\n";
    std::cout << "Top speed: " << results[num_setups - 1].top_speed_kmh << "km/h\n";
}