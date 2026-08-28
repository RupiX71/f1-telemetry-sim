#include "circuit_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<TrackSegment> load_circuit_csv(const std::string& filename) {
    std::vector<TrackSegment> track;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "[ERROR] Not possible to open circuit file" << filename << std::endl;
        return track;
    }

    // First line is the header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string length_str, radius_str, x_str, y_str;

        if (std::getline(ss, length_str, ',') && 
            std::getline(ss, radius_str, ',') &&
            std::getline(ss, x_str, ',') &&
            std::getline(ss, y_str, ',')) {
            
            TrackSegment seg;
            seg.length_m = std::stof(length_str);
            seg.radius_m = std::stof(radius_str);
            seg.x = std::stof(x_str);
            seg.y = std::stof(y_str);
            track.push_back(seg);
        }
    }

    return track;
}