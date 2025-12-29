// quantumlens_ascii.hpp - ASCII RENDERER
#pragma once

#include "quantumlens_core.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace ods {
namespace quantumlens {

/**
 * @brief ASCII Frame buffer
 */
class AsciiFrameBuffer {
public:
    AsciiFrameBuffer(size_t width, size_t height)
        : width_(width), height_(height) {
        buffer_.resize(width_ * height_, ' ');
        depth_.resize(width_ * height_, 1e6f);  // Far plane
    }
    
    void clear(char fill = ' ') {
        std::fill(buffer_.begin(), buffer_.end(), fill);
        std::fill(depth_.begin(), depth_.end(), 1e6f);
    }
    
    void set_pixel(size_t x, size_t y, char c, float depth = 0) {
        if (x >= width_ || y >= height_) return;
        
        size_t idx = y * width_ + x;
        
        // Z-buffer test
        if (depth < depth_[idx]) {
            buffer_[idx] = c;
            depth_[idx] = depth;
        }
    }
    
    char get_pixel(size_t x, size_t y) const {
        if (x >= width_ || y >= height_) return ' ';
        return buffer_[y * width_ + x];
    }
    
    size_t width() const { return width_; }
    size_t height() const { return height_; }
    
    std::string to_string() const {
        std::ostringstream oss;
        
        // Top border
        oss << "+" << std::string(width_, '-') << "+\n";
        
        // Content
        for (size_t y = 0; y < height_; ++y) {
            oss << "|";
            for (size_t x = 0; x < width_; ++x) {
                oss << buffer_[y * width_ + x];
            }
            oss << "|\n";
        }
        
        // Bottom border
        oss << "+" << std::string(width_, '-') << "+\n";
        
        return oss.str();
    }
    
private:
    size_t width_, height_;
    std::vector<char> buffer_;
    std::vector<float> depth_;
};

/**
 * @brief ASCII Renderer
 */
class AsciiRenderer {
public:
    AsciiRenderer(size_t width = 80, size_t height = 40)
        : width_(width), height_(height),
          framebuffer_(width, height) {
        // Caractères pour profondeur (proche → loin)
        depth_chars_ = {'@', '#', '%', '*', '+', '=', '-', '.', ' '};
    }
    
    // Render scene
    void render(const OctoSpace& space, const Camera& camera) {
        framebuffer_.clear();
        
        // Query nodes LOD
        auto visible_nodes = space.query_lod(camera);
        
        // Render each node
        for (const auto* node : visible_nodes) {
            if (node->is_leaf()) {
                render_points(node->points(), camera);
            } else {
                render_bbox(node->bounds(), camera);
            }
        }
    }
    
    // Display frame
    std::string to_string() const {
        return framebuffer_.to_string();
    }
    
    void display(const Camera& camera, const OctoSpace& space) const {
        // Clear screen (simple)
        std::cout << "\033[2J\033[H";  // ANSI clear + home
        
        // Frame
        std::cout << framebuffer_.to_string();
        
        // Stats
        std::cout << "\n";
        std::cout << "╔═══════════════════════════════════════════╗\n";
        std::cout << "║         QUANTUMLENS ASCII VIEWER          ║\n";
        std::cout << "╠═══════════════════════════════════════════╣\n";
        
        std::cout << "║ Position: (" 
                  << std::fixed << std::setprecision(2) << std::setw(6) << camera.position().x << ", "
                  << std::setw(6) << camera.position().y << ", "
                  << std::setw(6) << camera.position().z << ") ║\n";
        
        std::cout << "║ Level   : L" << std::setw(2) << static_cast<int>(camera.level())
                  << " (" << level_name(camera.level()) << ")";
        
        // Pad to 30 chars total
        std::string level_str = level_name(camera.level());
        size_t padding = 30 - 10 - level_str.length();
        std::cout << std::string(padding, ' ') << "║\n";
        
        std::cout << "║ Points  : " << std::setw(8) << space.num_points() 
                  << "                          ║\n";
        std::cout << "║ Nodes   : " << std::setw(8) << space.num_nodes()
                  << "                          ║\n";
        std::cout << "║ Depth   : " << std::setw(2) << static_cast<int>(space.max_depth())
                  << "                                  ║\n";
        
        std::cout << "╠═══════════════════════════════════════════╣\n";
        std::cout << "║ Controls:                                 ║\n";
        std::cout << "║  W/S/A/D : Move   Q/E : Up/Down          ║\n";
        std::cout << "║  ↑/↓/←/→ : Rotate  +/- : Zoom            ║\n";
        std::cout << "║  [/]     : Level   R   : Reset           ║\n";
        std::cout << "║  ESC     : Quit                           ║\n";
        std::cout << "╚═══════════════════════════════════════════╝\n";
    }
    
private:
    size_t width_, height_;
    AsciiFrameBuffer framebuffer_;
    std::vector<char> depth_chars_;
    
    // Project 3D → 2D
    bool project(const Vector3& world_pos, const Camera& camera, 
                 float& screen_x, float& screen_y, float& depth) const {
        // Simple orthographic projection
        Vector3 forward = camera.forward();
        Vector3 right = camera.right();
        Vector3 up = camera.up_normalized();
        
        Vector3 relative = world_pos - camera.position();
        
        // Depth (distance from camera)
        depth = relative.dot(forward);
        
        if (depth <= 0) return false;  // Behind camera
        
        // Screen space
        float x = relative.dot(right);
        float y = relative.dot(up);
        
        // Scale to screen
        float scale = 10.0f / (depth + 1.0f);  // Perspective-ish
        screen_x = (x * scale + 1.0f) * width_ * 0.5f;
        screen_y = (1.0f - y * scale) * height_ * 0.5f;
        
        return screen_x >= 0 && screen_x < width_ && 
               screen_y >= 0 && screen_y < height_;
    }
    
    // Render points
    void render_points(const std::vector<OctoDataPoint>& points, const Camera& camera) {
        for (const auto& point : points) {
            float sx, sy, depth;
            if (project(point.position, camera, sx, sy, depth)) {
                char c = depth_char(depth);
                framebuffer_.set_pixel(static_cast<size_t>(sx), 
                                      static_cast<size_t>(sy), c, depth);
            }
        }
    }
    
    // Render bounding box (wireframe)
    void render_bbox(const BBox3D& bbox, const Camera& camera) {
        // 8 corners
        Vector3 corners[8] = {
            Vector3(bbox.min.x, bbox.min.y, bbox.min.z),
            Vector3(bbox.max.x, bbox.min.y, bbox.min.z),
            Vector3(bbox.min.x, bbox.max.y, bbox.min.z),
            Vector3(bbox.max.x, bbox.max.y, bbox.min.z),
            Vector3(bbox.min.x, bbox.min.y, bbox.max.z),
            Vector3(bbox.max.x, bbox.min.y, bbox.max.z),
            Vector3(bbox.min.x, bbox.max.y, bbox.max.z),
            Vector3(bbox.max.x, bbox.max.y, bbox.max.z)
        };
        
        // Project corners
        float sx[8], sy[8], depth[8];
        bool visible[8];
        
        for (int i = 0; i < 8; ++i) {
            visible[i] = project(corners[i], camera, sx[i], sy[i], depth[i]);
        }
        
        // Draw edges (12 edges of cube)
        int edges[12][2] = {
            {0,1}, {2,3}, {4,5}, {6,7},  // Bottom/top faces
            {0,2}, {1,3}, {4,6}, {5,7},  // Vertical
            {0,4}, {1,5}, {2,6}, {3,7}   // Sides
        };
        
        for (const auto& edge : edges) {
            int i0 = edge[0], i1 = edge[1];
            if (visible[i0] && visible[i1]) {
                draw_line(sx[i0], sy[i0], sx[i1], sy[i1], 
                         (depth[i0] + depth[i1]) * 0.5f, '.');
            }
        }
    }
    
    // Draw line (Bresenham)
    void draw_line(float x0, float y0, float x1, float y1, float depth, char c) {
        int ix0 = static_cast<int>(x0);
        int iy0 = static_cast<int>(y0);
        int ix1 = static_cast<int>(x1);
        int iy1 = static_cast<int>(y1);
        
        int dx = std::abs(ix1 - ix0);
        int dy = std::abs(iy1 - iy0);
        int sx = ix0 < ix1 ? 1 : -1;
        int sy = iy0 < iy1 ? 1 : -1;
        int err = dx - dy;
        
        int x = ix0, y = iy0;
        
        for (int steps = 0; steps < 1000; ++steps) {  // Safety limit
            if (x >= 0 && x < static_cast<int>(width_) && 
                y >= 0 && y < static_cast<int>(height_)) {
                framebuffer_.set_pixel(x, y, c, depth);
            }
            
            if (x == ix1 && y == iy1) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
    
    // Depth → character
    char depth_char(float depth) const {
        int idx = static_cast<int>(depth / 2.0f);
        idx = std::max(0, std::min(static_cast<int>(depth_chars_.size()) - 1, idx));
        return depth_chars_[idx];
    }
    
    // Level name
    const char* level_name(uint8_t level) const {
        static const char* names[13] = {
            "spike",        // L0
            "synapse",      // L1
            "circuit",      // L2
            "column",       // L3
            "area",         // L4
            "lobe",         // L5
            "hemisphere",   // L6
            "brain",        // L7
            "consciousness",// L8
            "social",       // L9
            "culture",      // L10
            "civilization", // L11
            "noosphere"     // L12
        };
        return names[std::min(level, static_cast<uint8_t>(12))];
    }
};

}} // namespace ods::quantumlens
