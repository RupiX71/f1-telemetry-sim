#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <physics.cuh>

void run_sfml_visualizer(const std::vector<TrackSegment>& track, const CarSetup& setup);

#endif