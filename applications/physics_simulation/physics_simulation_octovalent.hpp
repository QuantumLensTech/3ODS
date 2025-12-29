// physics_simulation_octovalent.hpp
#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace ods {
namespace physics {

/**
 * @brief Particule dans espace octovalent 3D
 * 
 * Position encodée comme octant {0..7} + offset fractionnaire.
 * Permet simulations physiques avec distances euclidiennes exactes.
 */
struct OctoParticle {
    // Position (octant + offset [0.0-1.0] dans chaque octant)
    uint8_t octant_id;      // {0..7} - octant de base
    float offset_x;         // [0.0, 1.0] - offset intra-octant
    float offset_y;
    float offset_z;
    
    // Vélocité
    float vx, vy, vz;
    
    // Propriétés physiques
    float mass;
    float charge;          // Pour forces électrostatiques
    
    // État
    bool active;
    
    OctoParticle() 
        : octant_id(0), offset_x(0), offset_y(0), offset_z(0),
          vx(0), vy(0), vz(0), mass(1.0f), charge(0.0f), active(true) {}
    
    OctoParticle(uint8_t oct, float ox, float oy, float oz, float m = 1.0f)
        : octant_id(oct), offset_x(ox), offset_y(oy), offset_z(oz),
          vx(0), vy(0), vz(0), mass(m), charge(0.0f), active(true) {}
    
    // Position absolue (coordonnées continues)
    void get_position(float& x, float& y, float& z) const {
        uint8_t base_x = octant_id & 1;
        uint8_t base_y = (octant_id >> 1) & 1;
        uint8_t base_z = (octant_id >> 2) & 1;
        
        x = base_x + offset_x;
        y = base_y + offset_y;
        z = base_z + offset_z;
    }
    
    // Définir position depuis coordonnées continues
    void set_position(float x, float y, float z) {
        // Extraire octant de base
        uint8_t base_x = static_cast<uint8_t>(std::floor(x));
        uint8_t base_y = static_cast<uint8_t>(std::floor(y));
        uint8_t base_z = static_cast<uint8_t>(std::floor(z));
        
        // Limiter à [0,1] (2 octants par axe)
        base_x = std::min(base_x, uint8_t(1));
        base_y = std::min(base_y, uint8_t(1));
        base_z = std::min(base_z, uint8_t(1));
        
        octant_id = (base_z << 2) | (base_y << 1) | base_x;
        
        // Offsets fractionnaires
        offset_x = x - base_x;
        offset_y = y - base_y;
        offset_z = z - base_z;
        
        // Clamp [0.0, 1.0]
        offset_x = std::max(0.0f, std::min(1.0f, offset_x));
        offset_y = std::max(0.0f, std::min(1.0f, offset_y));
        offset_z = std::max(0.0f, std::min(1.0f, offset_z));
    }
    
    // Distance euclidienne vers autre particule (EXACTE)
    float distance_to(const OctoParticle& other) const {
        float x1, y1, z1, x2, y2, z2;
        get_position(x1, y1, z1);
        other.get_position(x2, y2, z2);
        
        float dx = x2 - x1;
        float dy = y2 - y1;
        float dz = z2 - z1;
        
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Vecteur direction vers autre particule (normalisé)
    void direction_to(const OctoParticle& other, float& dx, float& dy, float& dz) const {
        float x1, y1, z1, x2, y2, z2;
        get_position(x1, y1, z1);
        other.get_position(x2, y2, z2);
        
        dx = x2 - x1;
        dy = y2 - y1;
        dz = z2 - z1;
        
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist > 1e-6f) {
            dx /= dist;
            dy /= dist;
            dz /= dist;
        }
    }
};

/**
 * @brief Nœud octree pour accélération spatiale
 */
struct OctreeNode {
    // Bounding box (octant space)
    uint8_t octant_id;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    
    // Centre de masse
    float center_mass_x, center_mass_y, center_mass_z;
    float total_mass;
    float total_charge;
    
    // Particules dans ce nœud (si leaf)
    std::vector<size_t> particle_indices;
    
    // Enfants (nullptr si leaf)
    std::array<std::unique_ptr<OctreeNode>, 8> children;
    
    bool is_leaf() const { return children[0] == nullptr; }
    
    OctreeNode(uint8_t oct, float minx, float miny, float minz,
               float maxx, float maxy, float maxz)
        : octant_id(oct), 
          min_x(minx), min_y(miny), min_z(minz),
          max_x(maxx), max_y(maxy), max_z(maxz),
          center_mass_x(0), center_mass_y(0), center_mass_z(0),
          total_mass(0), total_charge(0) {
        // children default-constructed as nullptr
    }
};

/**
 * @brief Forces physiques disponibles
 */
enum class ForceType {
    GRAVITY,          // F = G * m1 * m2 / r²
    ELECTROSTATIC,    // F = k * q1 * q2 / r²
    LENNARD_JONES,    // F = 4ε[(σ/r)¹² - (σ/r)⁶]
    SPRING            // F = -k * (r - r0)
};

/**
 * @brief Configuration simulation
 */
struct SimulationConfig {
    // Paramètres physiques
    float gravitational_constant;  // G
    float coulomb_constant;        // k
    float lennard_jones_epsilon;   // ε
    float lennard_jones_sigma;     // σ
    float spring_constant;         // k
    float damping;                 // Amortissement vélocité
    
    // Paramètres numériques
    float dt;                      // Pas de temps
    float softening;               // Softening distance (évite singularités)
    float max_force;               // Force maximale (clamp)
    
    // Octree
    size_t max_particles_per_leaf; // Subdivision threshold
    float theta;                   // Barnes-Hut approximation (0.5 = bon compromis)
    
    // Conditions limites
    bool periodic_boundaries;
    bool collision_detection;
    float collision_radius;
    
    SimulationConfig()
        : gravitational_constant(1.0f),
          coulomb_constant(1.0f),
          lennard_jones_epsilon(1.0f),
          lennard_jones_sigma(1.0f),
          spring_constant(1.0f),
          damping(0.99f),
          dt(0.01f),
          softening(0.01f),
          max_force(100.0f),
          max_particles_per_leaf(8),
          theta(0.5f),
          periodic_boundaries(false),
          collision_detection(false),
          collision_radius(0.05f) {}
};

/**
 * @brief Classe principale de simulation physique octovalente
 */
class PhysicsSimulationOctovalent {
public:
    PhysicsSimulationOctovalent(const SimulationConfig& config = SimulationConfig());
    
    // === GESTION PARTICULES ===
    
    size_t add_particle(const OctoParticle& particle);
    void remove_particle(size_t index);
    void clear_particles();
    
    size_t num_particles() const { return particles_.size(); }
    const OctoParticle& get_particle(size_t index) const { return particles_[index]; }
    OctoParticle& get_particle(size_t index) { return particles_[index]; }
    
    // === FORCES ===
    
    void enable_force(ForceType type, bool enabled = true);
    bool is_force_enabled(ForceType type) const;
    
    // === SIMULATION ===
    
    void step();  // Un pas de simulation (dt)
    void simulate(size_t num_steps);  // Multiple steps
    
    // === OCTREE ===
    
    void build_octree();
    const OctreeNode* get_octree_root() const { return octree_root_.get(); }
    
    // === STATISTIQUES ===
    
    struct Statistics {
        float total_kinetic_energy;
        float total_potential_energy;
        float total_energy;
        float momentum_x, momentum_y, momentum_z;
        size_t num_active_particles;
        size_t num_octree_nodes;
        size_t num_force_calculations;
        float avg_distance;
        float min_distance;
        float max_distance;
    };
    
    Statistics compute_statistics() const;
    
    float get_total_energy() const;
    float get_kinetic_energy() const;
    float get_potential_energy() const;
    
    // === MÉTRIQUES ===
    
    float get_memory_usage_kb() const;
    float get_last_step_time_ms() const { return last_step_time_ms_; }
    
private:
    // État
    std::vector<OctoParticle> particles_;
    SimulationConfig config_;
    
    // Forces activées
    bool forces_enabled_[4];  // GRAVITY, ELECTROSTATIC, LENNARD_JONES, SPRING
    
    // Octree (Barnes-Hut)
    std::unique_ptr<OctreeNode> octree_root_;
    
    // Métriques
    float last_step_time_ms_;
    size_t total_steps_;
    
    // Helpers
    void compute_forces(std::vector<std::array<float, 3>>& forces);
    void compute_force_direct(size_t i, size_t j, std::array<float, 3>& force);
    void compute_force_barnes_hut(size_t i, const OctreeNode* node, std::array<float, 3>& force);
    void integrate_verlet(const std::vector<std::array<float, 3>>& forces);
    void apply_boundaries();
    void detect_collisions();
    
    void build_octree_recursive(OctreeNode* node, const std::vector<size_t>& particle_indices);
    void compute_center_of_mass(OctreeNode* node);
    bool should_use_approximation(size_t particle_idx, const OctreeNode* node) const;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline PhysicsSimulationOctovalent::PhysicsSimulationOctovalent(const SimulationConfig& config)
    : config_(config), last_step_time_ms_(0.0f), total_steps_(0) {
    forces_enabled_[0] = true;   // GRAVITY par défaut
    forces_enabled_[1] = false;  // ELECTROSTATIC
    forces_enabled_[2] = false;  // LENNARD_JONES
    forces_enabled_[3] = false;  // SPRING
}

inline size_t PhysicsSimulationOctovalent::add_particle(const OctoParticle& particle) {
    size_t index = particles_.size();
    particles_.push_back(particle);
    return index;
}

inline void PhysicsSimulationOctovalent::remove_particle(size_t index) {
    if (index < particles_.size()) {
        particles_[index].active = false;
    }
}

inline void PhysicsSimulationOctovalent::clear_particles() {
    particles_.clear();
}

inline void PhysicsSimulationOctovalent::enable_force(ForceType type, bool enabled) {
    forces_enabled_[static_cast<size_t>(type)] = enabled;
}

inline bool PhysicsSimulationOctovalent::is_force_enabled(ForceType type) const {
    return forces_enabled_[static_cast<size_t>(type)];
}

inline void PhysicsSimulationOctovalent::step() {
    auto start = std::chrono::high_resolution_clock::now();
    
    // 1. Construire octree (Barnes-Hut)
    build_octree();
    
    // 2. Calculer forces
    std::vector<std::array<float, 3>> forces(particles_.size(), {0.0f, 0.0f, 0.0f});
    compute_forces(forces);
    
    // 3. Intégration (Verlet)
    integrate_verlet(forces);
    
    // 4. Appliquer conditions limites
    apply_boundaries();
    
    // 5. Détection collisions (optionnel)
    if (config_.collision_detection) {
        detect_collisions();
    }
    
    total_steps_++;
    
    auto end = std::chrono::high_resolution_clock::now();
    last_step_time_ms_ = std::chrono::duration<float, std::milli>(end - start).count();
}

inline void PhysicsSimulationOctovalent::simulate(size_t num_steps) {
    for (size_t i = 0; i < num_steps; ++i) {
        step();
    }
}

inline void PhysicsSimulationOctovalent::build_octree() {
    // Créer racine (couvre tout espace [0,2] × [0,2] × [0,2])
    octree_root_ = std::make_unique<OctreeNode>(0, 0.0f, 0.0f, 0.0f, 2.0f, 2.0f, 2.0f);
    
    // Collecter indices particules actives
    std::vector<size_t> active_indices;
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (particles_[i].active) {
            active_indices.push_back(i);
        }
    }
    
    // Construction récursive
    build_octree_recursive(octree_root_.get(), active_indices);
    
    // Calculer centres de masse
    compute_center_of_mass(octree_root_.get());
}

inline void PhysicsSimulationOctovalent::build_octree_recursive(
    OctreeNode* node,
    const std::vector<size_t>& particle_indices) {
    
    // Si peu de particules, c'est une feuille
    if (particle_indices.size() <= config_.max_particles_per_leaf) {
        node->particle_indices = particle_indices;
        return;
    }
    
    // Sinon, subdiviser en 8 octants
    float mid_x = (node->min_x + node->max_x) / 2.0f;
    float mid_y = (node->min_y + node->max_y) / 2.0f;
    float mid_z = (node->min_z + node->max_z) / 2.0f;
    
    std::array<std::vector<size_t>, 8> child_particles;
    
    for (size_t idx : particle_indices) {
        float x, y, z;
        particles_[idx].get_position(x, y, z);
        
        uint8_t child_oct = 0;
        if (x >= mid_x) child_oct |= 1;
        if (y >= mid_y) child_oct |= 2;
        if (z >= mid_z) child_oct |= 4;
        
        child_particles[child_oct].push_back(idx);
    }
    
    // Créer enfants
    for (size_t i = 0; i < 8; ++i) {
        if (child_particles[i].empty()) continue;
        
        float min_x = (i & 1) ? mid_x : node->min_x;
        float max_x = (i & 1) ? node->max_x : mid_x;
        float min_y = (i & 2) ? mid_y : node->min_y;
        float max_y = (i & 2) ? node->max_y : mid_y;
        float min_z = (i & 4) ? mid_z : node->min_z;
        float max_z = (i & 4) ? node->max_z : mid_z;
        
        node->children[i] = std::make_unique<OctreeNode>(
            i, min_x, min_y, min_z, max_x, max_y, max_z
        );
        
        build_octree_recursive(node->children[i].get(), child_particles[i]);
    }
}

inline void PhysicsSimulationOctovalent::compute_center_of_mass(OctreeNode* node) {
    if (node->is_leaf()) {
        // Feuille : calculer depuis particules
        float total_mass = 0.0f;
        float total_charge = 0.0f;
        float cx = 0.0f, cy = 0.0f, cz = 0.0f;
        
        for (size_t idx : node->particle_indices) {
            const auto& p = particles_[idx];
            float x, y, z;
            p.get_position(x, y, z);
            
            cx += p.mass * x;
            cy += p.mass * y;
            cz += p.mass * z;
            total_mass += p.mass;
            total_charge += p.charge;
        }
        
        if (total_mass > 0) {
            node->center_mass_x = cx / total_mass;
            node->center_mass_y = cy / total_mass;
            node->center_mass_z = cz / total_mass;
        }
        
        node->total_mass = total_mass;
        node->total_charge = total_charge;
    } else {
        // Nœud interne : agréger depuis enfants
        float total_mass = 0.0f;
        float total_charge = 0.0f;
        float cx = 0.0f, cy = 0.0f, cz = 0.0f;
        
        for (auto& child : node->children) {
            if (child) {
                compute_center_of_mass(child.get());
                
                cx += child->total_mass * child->center_mass_x;
                cy += child->total_mass * child->center_mass_y;
                cz += child->total_mass * child->center_mass_z;
                total_mass += child->total_mass;
                total_charge += child->total_charge;
            }
        }
        
        if (total_mass > 0) {
            node->center_mass_x = cx / total_mass;
            node->center_mass_y = cy / total_mass;
            node->center_mass_z = cz / total_mass;
        }
        
        node->total_mass = total_mass;
        node->total_charge = total_charge;
    }
}

inline void PhysicsSimulationOctovalent::compute_forces(
    std::vector<std::array<float, 3>>& forces) {
    
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (!particles_[i].active) continue;
        
        // Barnes-Hut (si octree construit)
        if (octree_root_) {
            compute_force_barnes_hut(i, octree_root_.get(), forces[i]);
        } else {
            // Direct N² (si pas d'octree)
            for (size_t j = 0; j < particles_.size(); ++j) {
                if (i == j || !particles_[j].active) continue;
                
                std::array<float, 3> force = {0, 0, 0};
                compute_force_direct(i, j, force);
                
                forces[i][0] += force[0];
                forces[i][1] += force[1];
                forces[i][2] += force[2];
            }
        }
    }
}

inline void PhysicsSimulationOctovalent::compute_force_direct(
    size_t i, size_t j,
    std::array<float, 3>& force) {
    
    const auto& pi = particles_[i];
    const auto& pj = particles_[j];
    
    float dist = pi.distance_to(pj);
    if (dist < config_.softening) dist = config_.softening;
    
    float dx, dy, dz;
    pi.direction_to(pj, dx, dy, dz);
    
    float force_mag = 0.0f;
    
    // Gravité
    if (forces_enabled_[0]) {
        force_mag += config_.gravitational_constant * pi.mass * pj.mass / (dist * dist);
    }
    
    // Électrostatique
    if (forces_enabled_[1]) {
        force_mag += config_.coulomb_constant * pi.charge * pj.charge / (dist * dist);
    }
    
    // Clamp
    force_mag = std::min(force_mag, config_.max_force);
    
    force[0] = force_mag * dx;
    force[1] = force_mag * dy;
    force[2] = force_mag * dz;
}

inline void PhysicsSimulationOctovalent::compute_force_barnes_hut(
    size_t i, const OctreeNode* node,
    std::array<float, 3>& force) {
    
    if (!node) return;
    
    // Si feuille et contient particule i, skip
    if (node->is_leaf()) {
        for (size_t idx : node->particle_indices) {
            if (idx == i) continue;
            
            std::array<float, 3> f = {0, 0, 0};
            compute_force_direct(i, idx, f);
            force[0] += f[0];
            force[1] += f[1];
            force[2] += f[2];
        }
        return;
    }
    
    // Approximation Barnes-Hut ?
    if (should_use_approximation(i, node)) {
        // Traiter nœud comme particule unique au centre de masse
        float px, py, pz;
        particles_[i].get_position(px, py, pz);
        
        float dx = node->center_mass_x - px;
        float dy = node->center_mass_y - py;
        float dz = node->center_mass_z - pz;
        
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < config_.softening) dist = config_.softening;
        
        float force_mag = 0.0f;
        
        if (forces_enabled_[0]) {
            force_mag += config_.gravitational_constant * 
                        particles_[i].mass * node->total_mass / (dist * dist);
        }
        
        if (forces_enabled_[1]) {
            force_mag += config_.coulomb_constant *
                        particles_[i].charge * node->total_charge / (dist * dist);
        }
        
        force_mag = std::min(force_mag, config_.max_force);
        
        if (dist > 0) {
            force[0] += force_mag * dx / dist;
            force[1] += force_mag * dy / dist;
            force[2] += force_mag * dz / dist;
        }
    } else {
        // Descendre récursivement
        for (const auto& child : node->children) {
            if (child) {
                compute_force_barnes_hut(i, child.get(), force);
            }
        }
    }
}

inline bool PhysicsSimulationOctovalent::should_use_approximation(
    size_t particle_idx, const OctreeNode* node) const {
    
    float px, py, pz;
    particles_[particle_idx].get_position(px, py, pz);
    
    float dx = node->center_mass_x - px;
    float dy = node->center_mass_y - py;
    float dz = node->center_mass_z - pz;
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    float size = node->max_x - node->min_x;  // Taille nœud
    
    return (size / dist) < config_.theta;
}

inline void PhysicsSimulationOctovalent::integrate_verlet(
    const std::vector<std::array<float, 3>>& forces) {
    
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (!particles_[i].active) continue;
        
        auto& p = particles_[i];
        
        // a = F / m
        float ax = forces[i][0] / p.mass;
        float ay = forces[i][1] / p.mass;
        float az = forces[i][2] / p.mass;
        
        // v' = v + a*dt
        p.vx += ax * config_.dt;
        p.vy += ay * config_.dt;
        p.vz += az * config_.dt;
        
        // Damping
        p.vx *= config_.damping;
        p.vy *= config_.damping;
        p.vz *= config_.damping;
        
        // x' = x + v'*dt
        float x, y, z;
        p.get_position(x, y, z);
        
        x += p.vx * config_.dt;
        y += p.vy * config_.dt;
        z += p.vz * config_.dt;
        
        p.set_position(x, y, z);
    }
}

inline void PhysicsSimulationOctovalent::apply_boundaries() {
    for (auto& p : particles_) {
        if (!p.active) continue;
        
        float x, y, z;
        p.get_position(x, y, z);
        
        if (config_.periodic_boundaries) {
            // Périodique [0, 2]
            if (x < 0.0f) x += 2.0f;
            if (x >= 2.0f) x -= 2.0f;
            if (y < 0.0f) y += 2.0f;
            if (y >= 2.0f) y -= 2.0f;
            if (z < 0.0f) z += 2.0f;
            if (z >= 2.0f) z -= 2.0f;
            
            p.set_position(x, y, z);
        } else {
            // Réflexion sur bords
            if (x < 0.0f || x >= 2.0f) p.vx = -p.vx;
            if (y < 0.0f || y >= 2.0f) p.vy = -p.vy;
            if (z < 0.0f || z >= 2.0f) p.vz = -p.vz;
            
            x = std::max(0.0f, std::min(1.999f, x));
            y = std::max(0.0f, std::min(1.999f, y));
            z = std::max(0.0f, std::min(1.999f, z));
            
            p.set_position(x, y, z);
        }
    }
}

inline void PhysicsSimulationOctovalent::detect_collisions() {
    // Simple collision detection O(N²)
    // TODO: Utiliser octree pour accélération
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (!particles_[i].active) continue;
        
        for (size_t j = i + 1; j < particles_.size(); ++j) {
            if (!particles_[j].active) continue;
            
            float dist = particles_[i].distance_to(particles_[j]);
            
            if (dist < config_.collision_radius) {
                // Collision élastique simple (conservation momentum)
                float dx, dy, dz;
                particles_[i].direction_to(particles_[j], dx, dy, dz);
                
                float v_rel = (particles_[i].vx - particles_[j].vx) * dx +
                             (particles_[i].vy - particles_[j].vy) * dy +
                             (particles_[i].vz - particles_[j].vz) * dz;
                
                float m1 = particles_[i].mass;
                float m2 = particles_[j].mass;
                float impulse = 2.0f * v_rel / (m1 + m2);
                
                particles_[i].vx -= impulse * m2 * dx;
                particles_[i].vy -= impulse * m2 * dy;
                particles_[i].vz -= impulse * m2 * dz;
                
                particles_[j].vx += impulse * m1 * dx;
                particles_[j].vy += impulse * m1 * dy;
                particles_[j].vz += impulse * m1 * dz;
            }
        }
    }
}

inline float PhysicsSimulationOctovalent::get_kinetic_energy() const {
    float energy = 0.0f;
    for (const auto& p : particles_) {
        if (p.active) {
            float v2 = p.vx*p.vx + p.vy*p.vy + p.vz*p.vz;
            energy += 0.5f * p.mass * v2;
        }
    }
    return energy;
}

inline float PhysicsSimulationOctovalent::get_potential_energy() const {
    float energy = 0.0f;
    
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (!particles_[i].active) continue;
        
        for (size_t j = i + 1; j < particles_.size(); ++j) {
            if (!particles_[j].active) continue;
            
            float dist = particles_[i].distance_to(particles_[j]);
            if (dist < config_.softening) dist = config_.softening;
            
            // Gravité
            if (forces_enabled_[0]) {
                energy -= config_.gravitational_constant *
                         particles_[i].mass * particles_[j].mass / dist;
            }
            
            // Électrostatique
            if (forces_enabled_[1]) {
                energy += config_.coulomb_constant *
                         particles_[i].charge * particles_[j].charge / dist;
            }
        }
    }
    
    return energy;
}

inline float PhysicsSimulationOctovalent::get_total_energy() const {
    return get_kinetic_energy() + get_potential_energy();
}

inline PhysicsSimulationOctovalent::Statistics 
PhysicsSimulationOctovalent::compute_statistics() const {
    Statistics stats;
    stats.total_kinetic_energy = get_kinetic_energy();
    stats.total_potential_energy = get_potential_energy();
    stats.total_energy = stats.total_kinetic_energy + stats.total_potential_energy;
    
    stats.momentum_x = 0.0f;
    stats.momentum_y = 0.0f;
    stats.momentum_z = 0.0f;
    stats.num_active_particles = 0;
    stats.num_force_calculations = 0;
    
    stats.min_distance = std::numeric_limits<float>::max();
    stats.max_distance = 0.0f;
    float total_distance = 0.0f;
    size_t num_pairs = 0;
    
    for (const auto& p : particles_) {
        if (p.active) {
            stats.momentum_x += p.mass * p.vx;
            stats.momentum_y += p.mass * p.vy;
            stats.momentum_z += p.mass * p.vz;
            stats.num_active_particles++;
        }
    }
    
    // Distances inter-particules
    for (size_t i = 0; i < particles_.size(); ++i) {
        if (!particles_[i].active) continue;
        for (size_t j = i + 1; j < particles_.size(); ++j) {
            if (!particles_[j].active) continue;
            
            float dist = particles_[i].distance_to(particles_[j]);
            stats.min_distance = std::min(stats.min_distance, dist);
            stats.max_distance = std::max(stats.max_distance, dist);
            total_distance += dist;
            num_pairs++;
        }
    }
    
    stats.avg_distance = (num_pairs > 0) ? (total_distance / num_pairs) : 0.0f;
    stats.num_octree_nodes = 0;  // TODO: Compter nœuds octree
    
    return stats;
}

inline float PhysicsSimulationOctovalent::get_memory_usage_kb() const {
    size_t bytes = 0;
    
    // Particules
    bytes += particles_.size() * sizeof(OctoParticle);
    
    // TODO: Compter mémoire octree récursivement
    
    return bytes / 1024.0f;
}

}} // namespace ods::physics
