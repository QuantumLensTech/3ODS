// quantumlens_core.hpp - QUANTUMLENS CORE ENGINE
#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <memory>
#include <cmath>
#include <algorithm>
#include <functional>

namespace ods {
namespace quantumlens {

/**
 * @brief Vector3 pour positions/directions 3D
 */
struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    Vector3 normalized() const {
        float len = length();
        if (len > 0) {
            return Vector3(x / len, y / len, z / len);
        }
        return Vector3(0, 0, 1);  // Default forward
    }
    
    float dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
};

/**
 * @brief BoundingBox 3D
 */
struct BBox3D {
    Vector3 min, max;
    
    BBox3D() : min(0, 0, 0), max(1, 1, 1) {}
    BBox3D(const Vector3& min_, const Vector3& max_) : min(min_), max(max_) {}
    
    Vector3 center() const {
        return Vector3(
            (min.x + max.x) * 0.5f,
            (min.y + max.y) * 0.5f,
            (min.z + max.z) * 0.5f
        );
    }
    
    Vector3 size() const {
        return Vector3(max.x - min.x, max.y - min.y, max.z - min.z);
    }
    
    float volume() const {
        Vector3 s = size();
        return s.x * s.y * s.z;
    }
    
    bool contains(const Vector3& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }
    
    bool intersects(const BBox3D& other) const {
        return !(max.x < other.min.x || min.x > other.max.x ||
                 max.y < other.min.y || min.y > other.max.y ||
                 max.z < other.min.z || min.z > other.max.z);
    }
    
    float distance_to(const Vector3& point) const {
        Vector3 closest = point;
        closest.x = std::max(min.x, std::min(max.x, point.x));
        closest.y = std::max(min.y, std::min(max.y, point.y));
        closest.z = std::max(min.z, std::min(max.z, point.z));
        return (point - closest).length();
    }
};

/**
 * @brief Octant 3D (8 positions spatiales)
 */
enum class Octant : uint8_t {
    NW_BOTTOM = 0,  // (-, -, -)
    NE_BOTTOM = 1,  // (+, -, -)
    SW_BOTTOM = 2,  // (-, +, -)
    SE_BOTTOM = 3,  // (+, +, -)
    NW_TOP    = 4,  // (-, -, +)
    NE_TOP    = 5,  // (+, -, +)
    SW_TOP    = 6,  // (-, +, +)
    SE_TOP    = 7   // (+, +, +)
};

/**
 * @brief Data point dans l'espace octovalent
 */
struct OctoDataPoint {
    Vector3 position;
    float value;          // Scalar data
    uint8_t octant;       // Octant assignment (0-7)
    uint32_t id;          // Unique identifier
    
    OctoDataPoint() : value(0), octant(0), id(0) {}
    OctoDataPoint(const Vector3& pos, float val, uint8_t oct = 0, uint32_t id_ = 0)
        : position(pos), value(val), octant(oct), id(id_) {}
};

/**
 * @brief Octree Node 3D
 */
class OctreeNode3D {
public:
    static constexpr size_t MAX_POINTS_PER_NODE = 8;
    static constexpr size_t NUM_CHILDREN = 8;  // Vraiment octree (3D)
    
    OctreeNode3D(const BBox3D& bounds, uint8_t level = 0)
        : bounds_(bounds), level_(level), is_leaf_(true) {
        points_.reserve(MAX_POINTS_PER_NODE);
    }
    
    // Insertion point
    bool insert(const OctoDataPoint& point) {
        if (!bounds_.contains(point.position)) {
            return false;
        }
        
        if (!is_leaf_) {
            for (auto& child : children_) {
                if (child && child->insert(point)) {
                    return true;
                }
            }
            return false;
        }
        
        points_.push_back(point);
        
        if (points_.size() > MAX_POINTS_PER_NODE && can_subdivide()) {
            subdivide();
        }
        
        return true;
    }
    
    // Query points dans bbox
    void query(const BBox3D& query_bbox, std::vector<OctoDataPoint>& results) const {
        if (!bounds_.intersects(query_bbox)) {
            return;
        }
        
        if (is_leaf_) {
            for (const auto& p : points_) {
                if (query_bbox.contains(p.position)) {
                    results.push_back(p);
                }
            }
        } else {
            for (const auto& child : children_) {
                if (child) {
                    child->query(query_bbox, results);
                }
            }
        }
    }
    
    // Query avec LOD (level of detail)
    void query_lod(const Vector3& camera_pos, float lod_distance,
                   std::vector<const OctreeNode3D*>& visible_nodes) const {
        float dist = bounds_.distance_to(camera_pos);
        
        // Si trop loin, représenter ce nœud entier (LOD bas)
        if (dist > lod_distance && !is_leaf_) {
            visible_nodes.push_back(this);
            return;
        }
        
        // Si proche ou feuille, descendre/afficher
        if (is_leaf_) {
            visible_nodes.push_back(this);
        } else {
            for (const auto& child : children_) {
                if (child) {
                    child->query_lod(camera_pos, lod_distance, visible_nodes);
                }
            }
        }
    }
    
    // Accessors
    const BBox3D& bounds() const { return bounds_; }
    uint8_t level() const { return level_; }
    bool is_leaf() const { return is_leaf_; }
    size_t num_points() const { return points_.size(); }
    const std::vector<OctoDataPoint>& points() const { return points_; }
    
    // Statistics
    size_t count_nodes() const {
        if (is_leaf_) return 1;
        size_t total = 1;
        for (const auto& child : children_) {
            if (child) total += child->count_nodes();
        }
        return total;
    }
    
    size_t count_points() const {
        if (is_leaf_) return points_.size();
        size_t total = 0;
        for (const auto& child : children_) {
            if (child) total += child->count_points();
        }
        return total;
    }
    
    uint8_t max_depth() const {
        if (is_leaf_) return level_;
        uint8_t max_child = level_;
        for (const auto& child : children_) {
            if (child) max_child = std::max(max_child, child->max_depth());
        }
        return max_child;
    }
    
private:
    BBox3D bounds_;
    uint8_t level_;
    bool is_leaf_;
    std::vector<OctoDataPoint> points_;
    std::array<std::unique_ptr<OctreeNode3D>, NUM_CHILDREN> children_;
    
    bool can_subdivide() const {
        Vector3 size = bounds_.size();
        return size.x > 0.01f && size.y > 0.01f && size.z > 0.01f && level_ < 12;
    }
    
    void subdivide() {
        if (!can_subdivide() || !is_leaf_) return;
        
        Vector3 center = bounds_.center();
        
        // Créer 8 octants
        children_[0] = std::make_unique<OctreeNode3D>(
            BBox3D(bounds_.min, center), level_ + 1
        );
        children_[1] = std::make_unique<OctreeNode3D>(
            BBox3D(Vector3(center.x, bounds_.min.y, bounds_.min.z),
                   Vector3(bounds_.max.x, center.y, center.z)), level_ + 1
        );
        children_[2] = std::make_unique<OctreeNode3D>(
            BBox3D(Vector3(bounds_.min.x, center.y, bounds_.min.z),
                   Vector3(center.x, bounds_.max.y, center.z)), level_ + 1
        );
        children_[3] = std::make_unique<OctreeNode3D>(
            BBox3D(Vector3(center.x, center.y, bounds_.min.z),
                   Vector3(bounds_.max.x, bounds_.max.y, center.z)), level_ + 1
        );
        children_[4] = std::make_unique<OctreeNode3D>(
            BBox3D(Vector3(bounds_.min.x, bounds_.min.y, center.z),
                   Vector3(center.x, center.y, bounds_.max.z)), level_ + 1
        );
        children_[5] = std::make_unique<OctreeNode3D>(
            BBox3D(Vector3(center.x, bounds_.min.y, center.z),
                   Vector3(bounds_.max.x, center.y, bounds_.max.z)), level_ + 1
        );
        children_[6] = std::make_unique<OctreeNode3D>(
            BBox3D(Vector3(bounds_.min.x, center.y, center.z),
                   Vector3(center.x, bounds_.max.y, bounds_.max.z)), level_ + 1
        );
        children_[7] = std::make_unique<OctreeNode3D>(
            BBox3D(center, bounds_.max), level_ + 1
        );
        
        // Redistribuer points
        for (const auto& point : points_) {
            for (auto& child : children_) {
                if (child && child->insert(point)) {
                    break;
                }
            }
        }
        
        points_.clear();
        points_.shrink_to_fit();
        is_leaf_ = false;
    }
};

/**
 * @brief Camera pour navigation 3D
 */
class Camera {
public:
    Camera()
        : position_(0, 0, 5),
          target_(0, 0, 0),
          up_(0, 1, 0),
          fov_(60.0f),
          near_plane_(0.1f),
          far_plane_(1000.0f),
          current_level_(0) {}
    
    // Position & orientation
    void set_position(const Vector3& pos) { position_ = pos; }
    void set_target(const Vector3& target) { target_ = target; }
    void set_up(const Vector3& up) { up_ = up; }
    
    const Vector3& position() const { return position_; }
    const Vector3& target() const { return target_; }
    const Vector3& up() const { return up_; }
    
    // Direction vectors
    Vector3 forward() const {
        return (target_ - position_).normalized();
    }
    
    Vector3 right() const {
        return forward().cross(up_).normalized();
    }
    
    Vector3 up_normalized() const {
        return up_.normalized();
    }
    
    // Movement
    void move_forward(float distance) {
        Vector3 dir = forward();
        position_ = position_ + dir * distance;
        target_ = target_ + dir * distance;
    }
    
    void move_right(float distance) {
        Vector3 dir = right();
        position_ = position_ + dir * distance;
        target_ = target_ + dir * distance;
    }
    
    void move_up(float distance) {
        Vector3 dir = up_normalized();
        position_ = position_ + dir * distance;
        target_ = target_ + dir * distance;
    }
    
    void rotate_around_target(float yaw_deg, float pitch_deg) {
        // Simplification : rotation autour target
        Vector3 offset = position_ - target_;
        float radius = offset.length();
        
        // Convert to spherical
        float theta = std::atan2(offset.z, offset.x) + yaw_deg * 3.14159f / 180.0f;
        float phi = std::asin(offset.y / radius) + pitch_deg * 3.14159f / 180.0f;
        
        // Clamp phi
        phi = std::max(-1.5f, std::min(1.5f, phi));
        
        // Back to cartesian
        position_.x = target_.x + radius * std::cos(phi) * std::cos(theta);
        position_.y = target_.y + radius * std::sin(phi);
        position_.z = target_.z + radius * std::cos(phi) * std::sin(theta);
    }
    
    void zoom(float factor) {
        Vector3 dir = position_ - target_;
        float dist = dir.length();
        dist *= factor;
        dist = std::max(0.1f, std::min(1000.0f, dist));  // Clamp
        position_ = target_ + dir.normalized() * dist;
    }
    
    // Level of detail
    void set_level(uint8_t level) {
        current_level_ = std::min(level, static_cast<uint8_t>(12));
    }
    
    uint8_t level() const { return current_level_; }
    
    void level_up() {
        if (current_level_ < 12) current_level_++;
    }
    
    void level_down() {
        if (current_level_ > 0) current_level_--;
    }
    
    // FOV & planes
    float fov() const { return fov_; }
    float near_plane() const { return near_plane_; }
    float far_plane() const { return far_plane_; }
    
    void set_fov(float fov) { fov_ = std::max(10.0f, std::min(120.0f, fov)); }
    
    // Distance pour LOD
    float lod_distance() const {
        // Distance augmente avec level
        return std::pow(2.0f, static_cast<float>(current_level_));
    }
    
private:
    Vector3 position_;
    Vector3 target_;
    Vector3 up_;
    float fov_;           // Field of view (degrees)
    float near_plane_;
    float far_plane_;
    uint8_t current_level_;  // L0-L12
};

/**
 * @brief OctoSpace - Espace 3D octovalent avec octree
 */
class OctoSpace {
public:
    OctoSpace(const BBox3D& bounds = BBox3D(Vector3(-10, -10, -10), Vector3(10, 10, 10)))
        : root_(std::make_unique<OctreeNode3D>(bounds)) {}
    
    // Data management
    void add_point(const OctoDataPoint& point) {
        root_->insert(point);
    }
    
    void add_points(const std::vector<OctoDataPoint>& points) {
        for (const auto& p : points) {
            root_->insert(p);
        }
    }
    
    // Query
    std::vector<OctoDataPoint> query_bbox(const BBox3D& bbox) const {
        std::vector<OctoDataPoint> results;
        root_->query(bbox, results);
        return results;
    }
    
    std::vector<const OctreeNode3D*> query_lod(const Camera& camera) const {
        std::vector<const OctreeNode3D*> results;
        root_->query_lod(camera.position(), camera.lod_distance(), results);
        return results;
    }
    
    // Statistics
    size_t num_points() const {
        return root_->count_points();
    }
    
    size_t num_nodes() const {
        return root_->count_nodes();
    }
    
    uint8_t max_depth() const {
        return root_->max_depth();
    }
    
    const BBox3D& bounds() const {
        return root_->bounds();
    }
    
private:
    std::unique_ptr<OctreeNode3D> root_;
};

}} // namespace ods::quantumlens
