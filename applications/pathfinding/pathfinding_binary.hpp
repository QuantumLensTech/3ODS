#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <functional>

namespace ods {
namespace pathfinding {
namespace binary {

// ============================================================================
// Grid 3D classique avec float coordinates
// ============================================================================

struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    
    // Distance euclidienne
    float distance_to(const Vector3& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Distance Manhattan
    float manhattan_distance(const Vector3& other) const {
        return std::abs(x - other.x) + std::abs(y - other.y) + std::abs(z - other.z);
    }
    
    // Opérateurs
    bool operator==(const Vector3& other) const {
        constexpr float epsilon = 1e-6f;
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }
    
    bool operator!=(const Vector3& other) const {
        return !(*this == other);
    }
};

// Hash pour unordered_map
struct Vector3Hash {
    size_t operator()(const Vector3& v) const {
        // Quantize to integer grid for hashing
        int ix = static_cast<int>(std::round(v.x));
        int iy = static_cast<int>(std::round(v.y));
        int iz = static_cast<int>(std::round(v.z));
        
        size_t h1 = std::hash<int>{}(ix);
        size_t h2 = std::hash<int>{}(iy);
        size_t h3 = std::hash<int>{}(iz);
        
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// ============================================================================
// Grid Node
// ============================================================================

struct GridNode {
    Vector3 position;
    bool is_obstacle;
    
    GridNode() : position(), is_obstacle(false) {}
    GridNode(const Vector3& pos, bool blocked = false)
        : position(pos), is_obstacle(blocked) {}
};

// ============================================================================
// Grid 3D Structure
// ============================================================================

class Grid3D {
public:
    explicit Grid3D(uint8_t size = 8) : size_(size) {
        // Créer grille régulière float[3]
        nodes_.reserve(size_ * size_ * size_);
        
        for (uint8_t z = 0; z < size_; ++z) {
            for (uint8_t y = 0; y < size_; ++y) {
                for (uint8_t x = 0; x < size_; ++x) {
                    Vector3 pos(static_cast<float>(x),
                               static_cast<float>(y),
                               static_cast<float>(z));
                    nodes_.emplace_back(pos);
                    position_map_[pos] = nodes_.size() - 1;
                }
            }
        }
    }
    
    // Accès nœud par position
    GridNode* get_node(const Vector3& pos) {
        auto it = position_map_.find(pos);
        if (it != position_map_.end()) {
            return &nodes_[it->second];
        }
        return nullptr;
    }
    
    const GridNode* get_node(const Vector3& pos) const {
        auto it = position_map_.find(pos);
        if (it != position_map_.end()) {
            return &nodes_[it->second];
        }
        return nullptr;
    }
    
    // Accès par index discret
    GridNode* get_node(uint8_t x, uint8_t y, uint8_t z) {
        Vector3 pos(static_cast<float>(x),
                   static_cast<float>(y),
                   static_cast<float>(z));
        return get_node(pos);
    }
    
    // Définir obstacles
    void set_obstacle(const Vector3& pos, bool blocked = true) {
        auto* node = get_node(pos);
        if (node) {
            node->is_obstacle = blocked;
        }
    }
    
    void set_obstacle(uint8_t x, uint8_t y, uint8_t z, bool blocked = true) {
        Vector3 pos(static_cast<float>(x),
                   static_cast<float>(y),
                   static_cast<float>(z));
        set_obstacle(pos, blocked);
    }
    
    // Voisins (26-connectivité)
    std::vector<Vector3> get_neighbors(const Vector3& pos) const {
        std::vector<Vector3> neighbors;
        neighbors.reserve(26);
        
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    
                    float nx = pos.x + dx;
                    float ny = pos.y + dy;
                    float nz = pos.z + dz;
                    
                    // Vérifier limites
                    if (nx < 0.0f || nx >= size_ ||
                        ny < 0.0f || ny >= size_ ||
                        nz < 0.0f || nz >= size_) {
                        continue;
                    }
                    
                    Vector3 neighbor_pos(nx, ny, nz);
                    const auto* neighbor_node = get_node(neighbor_pos);
                    
                    // Filtrer obstacles
                    if (neighbor_node && !neighbor_node->is_obstacle) {
                        neighbors.push_back(neighbor_pos);
                    }
                }
            }
        }
        
        return neighbors;
    }
    
    // Getters
    uint8_t size() const { return size_; }
    size_t num_nodes() const { return nodes_.size(); }

private:
    uint8_t size_;
    std::vector<GridNode> nodes_;
    std::unordered_map<Vector3, size_t, Vector3Hash> position_map_;
};

// ============================================================================
// A* Pathfinding Binaire (float[3])
// ============================================================================

struct PathNodeBinary {
    Vector3 position;
    float g_cost;        // Coût depuis départ
    float h_cost;        // Heuristique vers arrivée
    float f_cost() const { return g_cost + h_cost; }
    Vector3 parent;      // Position parent
    
    bool operator>(const PathNodeBinary& other) const {
        return f_cost() > other.f_cost();
    }
};

class AStarBinary {
public:
    explicit AStarBinary(Grid3D& grid) : grid_(grid) {}
    
    // Recherche A*
    std::vector<Vector3> find_path(const Vector3& start, const Vector3& goal) {
        // Reset
        visited_.clear();
        came_from_.clear();
        g_score_.clear();
        
        // Priority queue
        std::priority_queue<PathNodeBinary, 
                          std::vector<PathNodeBinary>, 
                          std::greater<PathNodeBinary>> open_set;
        
        // Initialiser départ
        PathNodeBinary start_node;
        start_node.position = start;
        start_node.g_cost = 0.0f;
        start_node.h_cost = heuristic(start, goal);
        start_node.parent = Vector3(-1.0f, -1.0f, -1.0f); // Sentinelle
        
        open_set.push(start_node);
        g_score_[start] = 0.0f;
        
        // Boucle A*
        while (!open_set.empty()) {
            PathNodeBinary current = open_set.top();
            open_set.pop();
            
            Vector3 current_pos = current.position;
            
            // Déjà visité ?
            if (visited_.count(current_pos)) {
                continue;
            }
            
            visited_.insert(current_pos);
            
            // Arrivée ?
            if (current_pos == goal) {
                return reconstruct_path(start, goal);
            }
            
            // Explorer voisins
            auto neighbors = grid_.get_neighbors(current_pos);
            
            for (const Vector3& neighbor_pos : neighbors) {
                if (visited_.count(neighbor_pos)) {
                    continue;
                }
                
                // Coût mouvement : distance euclidienne
                float move_cost = current_pos.distance_to(neighbor_pos);
                float tentative_g = current.g_cost + move_cost;
                
                // Meilleur chemin ?
                if (g_score_.find(neighbor_pos) == g_score_.end() ||
                    tentative_g < g_score_[neighbor_pos]) {
                    
                    g_score_[neighbor_pos] = tentative_g;
                    came_from_[neighbor_pos] = current_pos;
                    
                    PathNodeBinary neighbor_path;
                    neighbor_path.position = neighbor_pos;
                    neighbor_path.g_cost = tentative_g;
                    neighbor_path.h_cost = heuristic(neighbor_pos, goal);
                    neighbor_path.parent = current_pos;
                    
                    open_set.push(neighbor_path);
                }
            }
        }
        
        // Pas de chemin
        return {};
    }
    
    // Statistiques
    size_t nodes_explored() const { return visited_.size(); }

private:
    Grid3D& grid_;
    std::unordered_set<Vector3, Vector3Hash> visited_;
    std::unordered_map<Vector3, Vector3, Vector3Hash> came_from_;
    std::unordered_map<Vector3, float, Vector3Hash> g_score_;
    
    // Heuristique : Manhattan
    float heuristic(const Vector3& from, const Vector3& to) const {
        return from.manhattan_distance(to);
    }
    
    // Reconstruire chemin
    std::vector<Vector3> reconstruct_path(const Vector3& start, const Vector3& goal) const {
        std::vector<Vector3> path;
        Vector3 current = goal;
        Vector3 sentinel(-1.0f, -1.0f, -1.0f);
        
        while (current != start) {
            path.push_back(current);
            auto it = came_from_.find(current);
            if (it == came_from_.end()) {
                return {}; // Erreur
            }
            current = it->second;
            
            // Sécurité contre boucles infinies
            if (current == sentinel) {
                return {};
            }
        }
        
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

} // namespace binary
} // namespace pathfinding
} // namespace ods
