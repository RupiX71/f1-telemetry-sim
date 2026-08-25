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
        setups[i].engine_power_kw = 700.f + (i * (50.0f / num_setups));

    }

    std::cout << "[CUDA] Sending Data to GPU..." << std::endl;

    run_simulation_batch(setups.data(), results.data(), num_setups);

    std::cout << "[CPU] Success here are the results: \n\n";

    // Imprimir o Setup 0, o do meio e o último
    std::cout << "Setup ID: " << results[0].setup_id 
              << " (Potencia Menor) -> 0-100 km/h: " << results[0].time_to_100_s << "s\n";
              
    std::cout << "Setup ID: " << results[num_setups/2].setup_id 
              << " (Potencia Media) -> 0-100 km/h: " << results[num_setups/2].time_to_100_s << "s\n";
              
    std::cout << "Setup ID: " << results[num_setups-1].setup_id 
              << " (Potencia Maior) -> 0-100 km/h: " << results[num_setups-1].time_to_100_s << "s\n";

    return 0;
}