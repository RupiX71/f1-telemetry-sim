#include "visualizer.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <algorithm>
#include <math.h>

void run_sfml_visualizer(const std::vector<TrackSegment>& track, const CarSetup& setup) {
    std::cout << "[SFML] A processar limites e rotacao do circuito..." << std::endl;
    
    std::vector<sf::Vector2f> points(track.size());

    for (size_t i = 0; i < track.size(); ++i) {
        points[i].x = track[i].x;
        points[i].y = track[i].y;
    }

    float min_x = points[0].x, max_x = points[0].x;
    float min_y = points[0].y, max_y = points[0].y;

    for (const auto& pt : points) {
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
        float screen_x = offset_x + (points[i].x - min_x) * scale;
        float screen_y = offset_y + (max_y - points[i].y) * scale;

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
    state.current_seg = 0;
    state.time_s = 0.0f;
    
    // (2ms per step)
    float physics_dt = 0.002f; 
    int steps_per_frame = 8; // 8 * 2ms = 16ms

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        for (int i = 0; i < steps_per_frame; ++i) {
            if (state.current_seg >= track.size() - 1) {
                state.current_seg = 0;
                state.battery_mj = 4.0f; 
                state.time_s = 0.0f;
            }
            
            step_physics(&state, &setup, track.data(), track.size(), physics_dt);
        }

        std::string action = state.is_braking ? "TRAVAR (Regen)" : "ACELERAR";
        
        int next_idx = (state.current_seg + 1) % track.size();
        
        // this needs to be done to fix the car position lmao
        float ax = track[state.current_seg].x;
        float ay = track[state.current_seg].y;
        float bx = track[next_idx].x;
        float by = track[next_idx].y;

        float seg_length = track[state.current_seg].length_m;
        float dir_x = (bx - ax) / seg_length;
        float dir_y = (by - ay) / seg_length;

        float real_car_x = ax + (dir_x * state.current_m);
        float real_car_y = ay + (dir_y * state.current_m);

        float screen_car_x = offset_x + (real_car_x - min_x) * scale;
        float screen_car_y = offset_y + (max_y - real_car_y) * scale;
        
        car.setPosition(screen_car_x, screen_car_y);

        char buffer[256];
        snprintf(buffer, sizeof(buffer),
            "TELEMETRIA EM TEMPO REAL\n"
            "Tempo de Volta: %.3f s\n"
            "Velocidade: %.0f km/h\n"
            "Acao: %s\n"
            "Bateria MGU-K: %.2f MJ\n"
            "Raio da Curva: %.0f m",
            state.time_s, state.v * 3.6f, action.c_str(), state.battery_mj, track[state.current_seg].radius_m);
        ui_text.setString(buffer);

        window.clear(sf::Color(20, 20, 20));
        window.draw(track_lines);
        window.draw(car);
        window.draw(ui_text);
        window.display();
    }
}