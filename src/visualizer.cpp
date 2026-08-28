#include "visualizer.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <algorithm>
#include <math.h>

void run_sfml_visualizer(const std::vector<TrackSegment>& track, const CarSetup& setup) {
    std::cout << "[SFML] A processar limites e rotacao do circuito..." << std::endl;
    
    float angle_rad = 1.66f; 
    float cos_theta = cos(angle_rad);
    float sin_theta = sin(angle_rad);

    std::vector<sf::Vector2f> rotated_points(track.size());

    for (size_t i = 0; i < track.size(); ++i) {
        rotated_points[i].x = track[i].x * cos_theta - track[i].y * sin_theta;
        rotated_points[i].y = track[i].x * sin_theta + track[i].y * cos_theta;
    }

    float min_x = rotated_points[0].x, max_x = rotated_points[0].x;
    float min_y = rotated_points[0].y, max_y = rotated_points[0].y;

    for (const auto& pt : rotated_points) {
        if (pt.x < min_x) min_x = pt.x;
        if (pt.x > max_x) max_x = pt.x;
        if (pt.y < min_y) min_y = pt.y;
        if (pt.y > max_y) max_y = pt.y;
    }

    int window_width = 1280;
    int window_height = 720;
    sf::RenderWindow window(sf::VideoMode(window_width, window_height), "F1 Telemetry Visualizer");
    window.setFramerateLimit(60);

    float margin = 50.0f; 
    float scale_x = (window_width - 2.0f * margin) / (max_x - min_x);
    float scale_y = (window_height - 2.0f * margin) / (max_y - min_y);
    float scale = std::min(scale_x, scale_y);

    float offset_x = (window_width - ((max_x - min_x) * scale)) / 2.0f;
    float offset_y = (window_height - ((max_y - min_y) * scale)) / 2.0f;

    sf::VertexArray track_lines(sf::LineStrip, track.size());
    for (size_t i = 0; i < track.size(); ++i) {
        float screen_x = offset_x + (rotated_points[i].x - min_x) * scale;
        float screen_y = offset_y + (max_y - rotated_points[i].y) * scale;

        track_lines[i].position = sf::Vector2f(screen_x, screen_y);
        track_lines[i].color = sf::Color(150, 150, 150);
    }
    
    std::cout << "[SFML] Renderizacao pronta. A abrir janela..." << std::endl;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);
        
        window.draw(track_lines);
        
        window.display();
    }
}