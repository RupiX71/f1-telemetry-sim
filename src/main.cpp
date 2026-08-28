#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include "physics.cuh"
#include "circuit_loader.h"
#include "visualizer.h"

int main() {
    std::string track_file = "../data/monza_pole.csv";
    std::cout << "[CPU] Loading circuit from: " << track_file << std::endl;

    std::vector<TrackSegment> track = load_circuit_csv(track_file);

    if (track.empty()) return -1;

    std::cout << "[CPU] Track loaded! Total segments: " << track.size() << std::endl;

    int num_setups = 5000;
    
    std::cout << "[CPU] Generating " << num_setups << " different setups..." << std::endl;

    std::vector<CarSetup> setups(num_setups);
    std::vector<SimResult> results(num_setups);

    std::mt19937 gen(888); 
    std::uniform_real_distribution<float> ice_dist(380.0f, 420.0f);
    std::uniform_real_distribution<float> mguk_dist(300.0f, 350.0f);
    std::uniform_real_distribution<float> drag_dist(0.9f, 1.5f);

    for(int i = 0 ; i < num_setups ; ++i) {
        setups[i].id = i;
        setups[i].mass_kg = 805.0f; 
        setups[i].ice_power_kw = ice_dist(gen);
        setups[i].mguk_power_kw = mguk_dist(gen);
        setups[i].drag_coef = drag_dist(gen);
    }

    std::cout << "[CUDA] Sending Data to GPU..." << std::endl;

    run_simulation_batch(setups.data(), results.data(), track.data(), track.size(), num_setups);

    std::cout << "[CPU] Success here are the results: \n\n";

    std::sort(results.begin(), results.end(), [](const SimResult& a, const SimResult& b) {
        return a.lap_time < b.lap_time;
    });

    CarSetup pole_position_setup = setups[results[0].setup_id];

    std::cout << " ----- Pole Position -----" << std::endl;
    std::cout << " ----- Setup Id: " << results[0].setup_id << std::endl;
    std::cout << pole_position_setup.ice_power_kw << "kw(ICE)|" << pole_position_setup.mguk_power_kw << "kw(MGU-K)|" << 
    pole_position_setup.drag_coef << "(DRAG)" << std::endl;
    std::cout << " ----- Lap Time: " << results[0].lap_time << std::endl;
    std::cout << " ----- Max Velocity: " << results[0].top_speed_kmh << std::endl;

    run_sfml_visualizer(track, pole_position_setup);
}