#include "visualizer.h"
#include "config.cuh"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <algorithm>
#include <math.h>

// all of this code should get divided lmao this way to crazy
void run_sfml_visualizer(const std::vector<TrackSegment>& track, const CarSetup& setup) {

    std::cout << "[SFML] Getting track limits..." << std::endl;
    // Track getter
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

    // Windows Creation
    int window_width = 1280;
    int window_height = 720;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(window_width, window_height), "F1 Telemetry Visualizer");
    window.setFramerateLimit(60);

    // track visualization (way to hard)
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
    

    // the car shape
    sf::CircleShape car_shape(6.0f);
    car_shape.setFillColor(sf::Color::Red);
    car_shape.setOrigin(6.0f, 6.0f);

    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        std::cerr << "[ERROR] Font not found!\n";
    }
    sf::Text ui_text("", font, 14);
    ui_text.setFillColor(sf::Color::White);
    ui_text.setPosition(1000, 20);

    F1Car car;
    car.v = track[0].real_speed_kmh / 3.6f;
    car.battery_mj = 4.0f;
    car.current_seg = 0;
    car.time_s = 0.0f;
    
    // (2ms per step)
    float physics_dt = 0.002f; 
    int steps_per_frame = 8; // 8 * 2ms = 16ms

    int sector1_end = track.size() / 3;
    int sector2_end = (track.size() * 2) / 3;

    float time_s1 = 0.0f;
    float time_s2 = 0.0f;
    float time_s3 = 0.0f;
    float last_lap_time = 0.0f;
    int prev_seg = 0;

    std::vector<sf::CircleShape> telemetry_trail;
    int frame_counter = 0;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        for (int i = 0; i < steps_per_frame; ++i) {
            if (car.current_seg >= track.size() - 1) {
                car.current_seg = 0;
                car.battery_mj = 4.0f;
                car.time_s = 0.0f;
            }
            
            int old_seg = car.current_seg;

            step_physics(&car, &setup, track.data(), track.size(), physics_dt);

            if (old_seg < sector1_end && car.current_seg >= sector1_end) {
                time_s1 = car.time_s;
            } else if (old_seg < sector2_end && car.current_seg >= sector2_end) {
                time_s2 = car.time_s - time_s1;
            }

            if (car.current_seg >= track.size() - 1) {
                time_s3 = car.time_s - (time_s1 + time_s2);
                last_lap_time = car.time_s;

                car.current_seg = 0;
                car.current_m = 0;
                car.battery_mj = Config::MAX_BATTERY_MJ;
                car.time_s = 0.0f;

                telemetry_trail.clear();
            }
        }

        std::string action = (car.action == DriverAction::ACCELERATE) ? "Throttle" :
                            (car.action == DriverAction::BRAKE) ? "Brakes" : "Coast";
        
        if (car.action == DriverAction::ACCELERATE) {
            action += " (Throttle: " + std::to_string(static_cast<int>(car.throttle_pedal * 100)) + "%)";
        } else if (car.action == DriverAction::BRAKE) {
            action += " (Brakes: " + std::to_string(static_cast<int>(car.brake_pedal * 100)) + "%)";
        }
        int next_idx = (car.current_seg + 1) % track.size();
        
        // this needs to be done to fix the car position lmao
        float ax = track[car.current_seg].x;
        float ay = track[car.current_seg].y;
        float bx = track[next_idx].x;
        float by = track[next_idx].y;
        float seg_length = track[car.current_seg].length_m;
        float dir_x = (bx - ax) / seg_length;
        float dir_y = (by - ay) / seg_length;
        float real_car_x = ax + (dir_x * car.current_m);
        float real_car_y = ay + (dir_y * car.current_m);
        float screen_car_x = offset_x + (real_car_x - min_x) * scale;
        float screen_car_y = offset_y + (max_y - real_car_y) * scale;
        
        sf::Color current_color = (car.action == DriverAction::ACCELERATE) ? sf::Color::Green :
                                (car.action == DriverAction::BRAKE) ? sf::Color::Red : sf::Color::Yellow;
        car_shape.setFillColor(current_color);

        frame_counter++;
        if (frame_counter % 3 == 0) {
            sf::CircleShape dot(2.5f);
            dot.setFillColor(current_color);
            dot.setOrigin(2.5f, 2.5f);
            dot.setPosition(screen_car_x, screen_car_y);
            telemetry_trail.push_back(dot);
        }

        car_shape.setPosition(screen_car_x, screen_car_y);
        char buffer[512];

        float sim_speed_kmh = car.v * 3.6f;
        float real_speed_kmh = track[car.current_seg].real_speed_kmh;
        float delta_speed = sim_speed_kmh - real_speed_kmh;

        snprintf(buffer, sizeof(buffer),
            "TELEMETRIA EM TEMPO REAL\n"
            "Velocidade Simulada: %.2f km/h\n"
            "Velocidade Real: %.2f km/h\n"
            "Delta Velocidade: %.2f km/h\n"
            "Acao: %s\n" // how do i put the throttle and braking percentage here
            "Bateria MGU-K: %.2f MJ\n"
            "Raio da Curva: %.0f m\n"
            "--- CRONOMETRAGEM ---\n"
            "Setor 1: %.3f s\n"
            "Setor 2: %.3f s\n"
            "Setor 3: %.3f s\n"
            "Lap Time: %.3f s\n"
            "Last Lap Time: %.3f s\n",
            sim_speed_kmh, real_speed_kmh, delta_speed, action.c_str(), car.battery_mj, track[car.current_seg].radius_m,
            time_s1, time_s2, time_s3, car.time_s, last_lap_time);
        ui_text.setString(buffer);

        window.clear(sf::Color(20, 20, 20));
        window.draw(track_lines);

        for (const auto& dot : telemetry_trail) {
            window.draw(dot);
        }

        window.draw(car_shape);
        window.draw(ui_text);
        window.display();
    }
}