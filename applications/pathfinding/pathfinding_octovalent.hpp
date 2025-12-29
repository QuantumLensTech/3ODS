#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace ods {
namespace pathfinding {

// ============================================================================
// Octree 3D pour pathfinding octovalent
// ============================================================================

struct OctantNode {
    uint16_t index;      // Index dans octree (0-511 pour depth=3)
    uint8_t x, y, z;     // Position discrète [0-7]
    bool is_obstacle;    // Bloqué ou libre
    
    OctantNode() : index(0), x(0), y(0), z(0), is_obstacle(false) {}
    
    OctantNode(uint16_t idx, uint8_t _x, uint8_t _y, uint8_t _z, bool blocked = false)
        : index(idx), x(_x), y(_y), z(_z), is_obstacle(blocked) {}
    
    // Distance euclidienne exacte vers autre octant
    float distance_to(const OctantNode& other) const {
        int dx = static_cast<int>(x) - static_cast<int>(other.x);
        int dy = static_cast<int>(y) - static_cast<int>(other.y);
        int dz = static_cast<int>(z) - static_cast<int>(other.z);
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Distance Manhattan (heuristique admissible)
    uint8_t manhattan_distance(const OctantNode& other) const {
        return std::abs(static_cast<int>(x) - static_cast<int>(other.x))
             + std::abs(static_cast<int>(y) - static_cast<int>(other.y))
             + std::abs(static_cast<int>(z) - static_cast<int>(other.z));
    }
};

// ============================================================================
// Octree structure
// ============================================================================

class Octree3D {
public:
    explicit Octree3D(uint8_t depth = 3) 
        : depth_(depth), size_(1 << depth) // size = 2^depth
    {
        // Créer tous les nœuds (8^depth)
        uint16_t total = size_ * size_ * size_;
        nodes_.reserve(total);
        
        for (uint8_t z = 0; z < size_; ++z) {
            for (uint8_t y = 0; y < size_; ++y) {
                for (uint8_t x = 0; x < size_; ++x) {
                    uint16_t index = encode(x, y, z);
                    nodes_.emplace_back(index, x, y, z);
                }
            }
        }
    }
    
    // Encoder position (x,y,z) → index
    uint16_t encode(uint8_t x, uint8_t y, uint8_t z) const {
        return z * size_ * size_ + y * size_ + x;
    }
    
    // Décoder index → position (x,y,z)
    void decode(uint16_t index, uint8_t& x, uint8_t& y, uint8_t& z) const {
        z = index / (size_ * size_);
        uint16_t remainder = index % (size_ * size_);
        y = remainder / size_;
        x = remainder % size_;
    }
    
    // Accès nœud
    OctantNode& get_node(uint16_t index) { return nodes_[index]; }
    const OctantNode& get_node(uint16_t index) const { return nodes_[index]; }
    
    OctantNode& get_node(uint8_t x, uint8_t y, uint8_t z) {
        return nodes_[encode(x, y, z)];
    }
    
    // Définir obstacles
    void set_obstacle(uint16_t index, bool blocked = true) {
        nodes_[index].is_obstacle = blocked;
    }
    
    void set_obstacle(uint8_t x, uint8_t y, uint8_t z, bool blocked = true) {
        get_node(x, y, z).is_obstacle = blocked;
    }
    
    // Voisins (26-connectivité : cube 3×3×3 - centre)
    std::vector<uint16_t> get_neighbors(uint16_t index) const {
        std::vector<uint16_t> neighbors;
        neighbors.reserve(26);
        
        uint8_t x, y, z;
        decode(index, x, y, z);
        
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue; // Pas le centre
                    
                    int nx = static_cast<int>(x) + dx;
                    int ny = static_cast<int>(y) + dy;
                    int nz = static_cast<int>(z) + dz;
                    
                    // Vérifier limites
                    if (nx < 0 || nx >= size_ || 
                        ny < 0 || ny >= size_ || 
                        nz < 0 || nz >= size_) {
                        continue;
                    }
                    
                    uint16_t neighbor_idx = encode(nx, ny, nz);
                    
                    // Filtrer obstacles
                    if (!nodes_[neighbor_idx].is_obstacle) {
                        neighbors.push_back(neighbor_idx);
                    }
                }
            }
        }
        
        return neighbors;
    }
    
    // Getters
    uint8_t depth() const { return depth_; }
    uint8_t size() const { return size_; }
    size_t num_nodes() const { return nodes_.size(); }

private:
    uint8_t depth_;                    // Profondeur octree (3 → 8³=512)
    uint8_t size_;                     // Taille par dimension (2^depth)
    std::vector<OctantNode> nodes_;    // Tous les nœuds
};

// ============================================================================
// A* Pathfinding Octovalent
// ============================================================================

struct PathNode {
    uint16_t index;
    float g_cost;        // Coût depuis départ
    float h_cost;        // Heuristique vers arrivée
    float f_cost() const { return g_cost + h_cost; }
    uint16_t parent;     // Index parent (-1 si racine)
    
    bool operator>(const PathNode& other) const {
        return f_cost() > other.f_cost();
    }
};

class AStarOctovalent {
public:
    explicit AStarOctovalent(Octree3D& octree) : octree_(octree) {}
    
    // Recherche A*
    std::vector<uint16_t> find_path(uint16_t start_idx, uint16_t goal_idx) {
        // Reset
        visited_.clear();
        came_from_.clear();
        g_score_.clear();
        
        // Priority queue (min-heap sur f_cost)
        std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> open_set;
        
        // Initialiser départ
        PathNode start_node;
        start_node.index = start_idx;
        start_node.g_cost = 0.0f;
        start_node.h_cost = heuristic(start_idx, goal_idx);
        start_node.parent = static_cast<uint16_t>(-1);
        
        open_set.push(start_node);
        g_score_[start_idx] = 0.0f;
        
        // Boucle A*
        while (!open_set.empty()) {
            PathNode current = open_set.top();
            open_set.pop();
            
            uint16_t current_idx = current.index;
            
            // Déjà visité ? (peut arriver si plusieurs insertions)
            if (visited_.count(current_idx)) {
                continue;
            }
            
            visited_.insert(current_idx);
            
            // Arrivée ?
            if (current_idx == goal_idx) {
                return reconstruct_path(start_idx, goal_idx);
            }
            
            // Explorer voisins
            auto neighbors = octree_.get_neighbors(current_idx);
            
            for (uint16_t neighbor_idx : neighbors) {
                if (visited_.count(neighbor_idx)) {
                    continue; // Déjà exploré
                }
                
                // Coût mouvement : distance euclidienne exacte
                const auto& current_node = octree_.get_node(current_idx);
                const auto& neighbor_node = octree_.get_node(neighbor_idx);
                float move_cost = current_node.distance_to(neighbor_node);
                
                float tentative_g = current.g_cost + move_cost;
                
                // Meilleur chemin vers ce voisin ?
                if (g_score_.find(neighbor_idx) == g_score_.end() || 
                    tentative_g < g_score_[neighbor_idx]) {
                    
                    g_score_[neighbor_idx] = tentative_g;
                    came_from_[neighbor_idx] = current_idx;
                    
                    PathNode neighbor_path;
                    neighbor_path.index = neighbor_idx;
                    neighbor_path.g_cost = tentative_g;
                    neighbor_path.h_cost = heuristic(neighbor_idx, goal_idx);
                    neighbor_path.parent = current_idx;
                    
                    open_set.push(neighbor_path);
                }
            }
        }
        
        // Pas de chemin trouvé
        return {};
    }
    
    // Statistiques dernière recherche
    size_t nodes_explored() const { return visited_.size(); }

private:
    Octree3D& octree_;
    std::unordered_set<uint16_t> visited_;
    std::unordered_map<uint16_t, uint16_t> came_from_;
    std::unordered_map<uint16_t, float> g_score_;
    
    // Heuristique : distance Manhattan (admissible)
    float heuristic(uint16_t from_idx, uint16_t to_idx) const {
        const auto& from = octree_.get_node(from_idx);
        const auto& to = octree_.get_node(to_idx);
        return static_cast<float>(from.manhattan_distance(to));
    }
    
    // Reconstruire chemin
    std::vector<uint16_t> reconstruct_path(uint16_t start, uint16_t goal) const {
        std::vector<uint16_t> path;
        uint16_t current = goal;
        
        while (current != start) {
            path.push_back(current);
            auto it = came_from_.find(current);
            if (it == came_from_.end()) {
                return {}; // Erreur
            }
            current = it->second;
        }
        
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }
};

} // namespace pathfinding
} // namespace ods
