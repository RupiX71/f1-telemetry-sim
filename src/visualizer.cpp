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
    
    sf::CircleShape car(6.0f);
    car.setFillColor(sf::Color::Red);
    car.setOrigin(6.0f, 6.0f);

    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        std::cerr << "[AVISO] Fonte não encontrada. O texto não vai aparecer!\n";
    }
    sf::Text ui_text("", font, 18);
    ui_text.setFillColor(sf::Color::White);
    ui_text.setPosition(20, 20);

    CarState state;
    state.v = 10.0f;
    state.battery_mj = 4.0f;
    state.dist_in_seg = 0.0f;
    state.current_idx = 0;
    
    float dt = 0.016f;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        if (state.current_idx >= track.size() - 1) {
            state.current_idx = 0;
            state.dist_in_seg = 0.0f;
            state.battery_mj = 4.0f;
        }
        
        step_physics(&state, &setup, track.data(), track.size(), dt);

        std::string action = state.is_braking ? "TRAVAR (Regen)" : "ACELERAR";

        int next_idx = state.current_idx + 1;
        float lerp_factor = state.dist_in_seg / track[state.current_idx].length_m;
        
        float real_x = track[state.current_idx].x + (track[next_idx].x - track[state.current_idx].x) * lerp_factor;
        float real_y = track[state.current_idx].y + (track[next_idx].y - track[state.current_idx].y) * lerp_factor;

        float rot_x = real_x * cos_theta - real_y * sin_theta;
        float rot_y = real_x * sin_theta + real_y * cos_theta;

        float screen_x = offset_x + (rot_x - min_x) * scale;
        float screen_y = offset_y + (max_y - rot_y) * scale;
        
        car.setPosition(screen_x, screen_y);

        char buffer[200];
        snprintf(buffer, sizeof(buffer), 
            "TELEMETRIA EM TEMPO REAL\n"
            "Velocidade: %.0f km/h\n"
            "Acao: %s\n"
            "Bateria MGU-K: %.2f MJ\n"
            "Raio da Curva: %.0f m", 
            state.v * 3.6f, action.c_str(), state.battery_mj, track[state.current_idx].radius_m);
        ui_text.setString(buffer);

        window.clear(sf::Color(20, 20, 20)); 
        window.draw(track_lines);            
        window.draw(car);                    
        window.draw(ui_text);                
        window.display();                    
    }
}