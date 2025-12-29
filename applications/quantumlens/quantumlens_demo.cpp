// quantumlens_demo.cpp - APPLICATION INTERACTIVE
#include "quantumlens_core.hpp"
#include "quantumlens_ascii.hpp"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace ods::quantumlens;

// Keyboard input (non-blocking)
class KeyboardInput {
public:
    KeyboardInput() {
        // Save old settings
        tcgetattr(STDIN_FILENO, &old_tio_);
        
        // Set non-blocking
        struct termios new_tio = old_tio_;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
        
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    ~KeyboardInput() {
        // Restore settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
        fcntl(STDIN_FILENO, F_SETFL, 0);
    }
    
    int get_key() {
        char buf = 0;
        if (read(STDIN_FILENO, &buf, 1) > 0) {
            return buf;
        }
        return 0;  // No key
    }
    
private:
    struct termios old_tio_;
};

// Generate test data
void generate_test_data(OctoSpace& space, int num_points = 1000) {
    std::srand(std::time(nullptr));
    
    for (int i = 0; i < num_points; ++i) {
        float x = (std::rand() % 2000 - 1000) / 100.0f;  // -10 → +10
        float y = (std::rand() % 2000 - 1000) / 100.0f;
        float z = (std::rand() % 2000 - 1000) / 100.0f;
        
        float value = std::rand() % 100 / 100.0f;
        uint8_t octant = std::rand() % 8;
        
        space.add_point(OctoDataPoint(Vector3(x, y, z), value, octant, i));
    }
}

// Generate structured data (octree levels)
void generate_structured_data(OctoSpace& space) {
    // Level 0 : Centre dense
    for (int i = 0; i < 50; ++i) {
        float x = (std::rand() % 200 - 100) / 100.0f;  // -1 → +1
        float y = (std::rand() % 200 - 100) / 100.0f;
        float z = (std::rand() % 200 - 100) / 100.0f;
        space.add_point(OctoDataPoint(Vector3(x, y, z), 1.0f, 0, i));
    }
    
    // Level 1 : Couronne moyenne
    for (int i = 0; i < 100; ++i) {
        float angle = i * 3.14159f * 2.0f / 100.0f;
        float radius = 3.0f + (std::rand() % 100) / 100.0f;
        float x = radius * std::cos(angle);
        float y = (std::rand() % 200 - 100) / 50.0f;
        float z = radius * std::sin(angle);
        space.add_point(OctoDataPoint(Vector3(x, y, z), 0.7f, 1, 50 + i));
    }
    
    // Level 2 : Couronne large
    for (int i = 0; i < 200; ++i) {
        float angle = i * 3.14159f * 2.0f / 200.0f;
        float radius = 6.0f + (std::rand() % 200) / 100.0f;
        float x = radius * std::cos(angle);
        float y = (std::rand() % 400 - 200) / 50.0f;
        float z = radius * std::sin(angle);
        space.add_point(OctoDataPoint(Vector3(x, y, z), 0.4f, 2, 150 + i));
    }
    
    // Level 3 : Extrémités
    for (int octant = 0; octant < 8; ++octant) {
        float x = (octant & 1) ? 9.0f : -9.0f;
        float y = (octant & 2) ? 9.0f : -9.0f;
        float z = (octant & 4) ? 9.0f : -9.0f;
        
        for (int i = 0; i < 10; ++i) {
            float dx = (std::rand() % 100 - 50) / 100.0f;
            float dy = (std::rand() % 100 - 50) / 100.0f;
            float dz = (std::rand() % 100 - 50) / 100.0f;
            space.add_point(OctoDataPoint(
                Vector3(x + dx, y + dy, z + dz), 0.2f, octant, 350 + octant * 10 + i
            ));
        }
    }
}

int main(int argc, char** argv) {
    // Configuration
    bool use_structured = (argc > 1 && std::string(argv[1]) == "--structured");
    
    std::cout << "╔═══════════════════════════════════════════════════════╗\n";
    std::cout << "║          QUANTUMLENS ASCII VIEWER - DEMO             ║\n";
    std::cout << "╠═══════════════════════════════════════════════════════╣\n";
    std::cout << "║  Initializing octree space...                        ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════╝\n";
    
    // Create space
    OctoSpace space(BBox3D(Vector3(-10, -10, -10), Vector3(10, 10, 10)));
    
    // Generate data
    if (use_structured) {
        std::cout << "Generating structured data (3 levels)...\n";
        generate_structured_data(space);
    } else {
        std::cout << "Generating random data (1000 points)...\n";
        generate_test_data(space, 1000);
    }
    
    std::cout << "Points inserted : " << space.num_points() << "\n";
    std::cout << "Octree nodes    : " << space.num_nodes() << "\n";
    std::cout << "Max depth       : " << static_cast<int>(space.max_depth()) << "\n\n";
    std::cout << "Press any key to start...\n";
    std::cin.get();
    
    // Setup camera
    Camera camera;
    camera.set_position(Vector3(0, 0, 15));
    camera.set_target(Vector3(0, 0, 0));
    camera.set_level(0);
    
    // Renderer
    AsciiRenderer renderer(80, 30);
    
    // Input
    KeyboardInput keyboard;
    
    // Main loop
    bool running = true;
    float move_speed = 0.5f;
    float rotate_speed = 5.0f;
    float zoom_factor = 1.1f;
    
    while (running) {
        // Render
        renderer.render(space, camera);
        renderer.display(camera, space);
        
        // Input
        int key = keyboard.get_key();
        
        switch (key) {
            // Movement
            case 'w': case 'W':
                camera.move_forward(move_speed);
                break;
            case 's': case 'S':
                camera.move_forward(-move_speed);
                break;
            case 'a': case 'A':
                camera.move_right(-move_speed);
                break;
            case 'd': case 'D':
                camera.move_right(move_speed);
                break;
            case 'q': case 'Q':
                camera.move_up(move_speed);
                break;
            case 'e': case 'E':
                camera.move_up(-move_speed);
                break;
            
            // Rotation (arrow keys - ANSI sequences)
            case 27:  // ESC or arrow key start
            {
                int key2 = keyboard.get_key();
                if (key2 == '[') {
                    int key3 = keyboard.get_key();
                    switch (key3) {
                        case 'A':  // Up arrow
                            camera.rotate_around_target(0, -rotate_speed);
                            break;
                        case 'B':  // Down arrow
                            camera.rotate_around_target(0, rotate_speed);
                            break;
                        case 'C':  // Right arrow
                            camera.rotate_around_target(rotate_speed, 0);
                            break;
                        case 'D':  // Left arrow
                            camera.rotate_around_target(-rotate_speed, 0);
                            break;
                    }
                } else if (key2 == 0) {
                    // Real ESC (no follow-up)
                    running = false;
                }
                break;
            }
            
            // Zoom
            case '+': case '=':
                camera.zoom(1.0f / zoom_factor);
                break;
            case '-': case '_':
                camera.zoom(zoom_factor);
                break;
            
            // Level
            case '[':
                camera.level_down();
                break;
            case ']':
                camera.level_up();
                break;
            
            // Reset
            case 'r': case 'R':
                camera.set_position(Vector3(0, 0, 15));
                camera.set_target(Vector3(0, 0, 0));
                camera.set_level(0);
                break;
            
            // Quit
            case 'x': case 'X':
                running = false;
                break;
        }
        
        // Frame delay
        usleep(50000);  // 50ms = 20 FPS
    }
    
    std::cout << "\n\n";
    std::cout << "╔═══════════════════════════════════════════════════════╗\n";
    std::cout << "║          QUANTUMLENS ASCII VIEWER - CLOSED            ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════╝\n";
    
    return 0;
}
