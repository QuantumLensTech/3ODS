// pattern_recognition_octovalent.hpp
// Pattern Recognition Géométrique - Encodage Octovalent Natif
// Author: Jean-Christophe Ané
// Date: December 2025

#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <string>
#include <memory>

namespace ods {
namespace pattern_recognition {

// ============================================================================
// STRUCTURES DE BASE
// ============================================================================

/**
 * @brief Point 3D octovalent (octant position)
 * 
 * Encodage : uint8_t ∈ {0..7} correspond à octant (x, y, z) ∈ {0,1}³
 * 
 * Mapping:
 *   0 = (0,0,0) = (-, -, -)
 *   1 = (1,0,0) = (+, -, -)
 *   2 = (0,1,0) = (-, +, -)
 *   3 = (1,1,0) = (+, +, -)
 *   4 = (0,0,1) = (-, -, +)
 *   5 = (1,0,1) = (+, -, +)
 *   6 = (0,1,1) = (-, +, +)
 *   7 = (1,1,1) = (+, +, +)
 */
struct OctoPoint {
    uint8_t octant;  // {0..7}
    
    OctoPoint() : octant(0) {}
    explicit OctoPoint(uint8_t oct) : octant(oct & 0x7) {}
    OctoPoint(uint8_t x, uint8_t y, uint8_t z) : octant(encode(x, y, z)) {}
    
    // Encodage (x,y,z) → octant
    static uint8_t encode(uint8_t x, uint8_t y, uint8_t z) {
        return ((z & 1) << 2) | ((y & 1) << 1) | (x & 1);
    }
    
    // Décodage octant → (x,y,z)
    void decode(uint8_t& x, uint8_t& y, uint8_t& z) const {
        x = octant & 1;
        y = (octant >> 1) & 1;
        z = (octant >> 2) & 1;
    }
    
    // Distance euclidienne exacte
    float distance_to(const OctoPoint& other) const {
        uint8_t x1, y1, z1, x2, y2, z2;
        decode(x1, y1, z1);
        other.decode(x2, y2, z2);
        
        int dx = static_cast<int>(x2) - static_cast<int>(x1);
        int dy = static_cast<int>(y2) - static_cast<int>(y1);
        int dz = static_cast<int>(z2) - static_cast<int>(z1);
        
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // Distance Hamming (nombre de bits différents)
    uint8_t hamming_distance(const OctoPoint& other) const {
        return __builtin_popcount(octant ^ other.octant);
    }
    
    bool operator==(const OctoPoint& other) const {
        return octant == other.octant;
    }
    
    bool operator!=(const OctoPoint& other) const {
        return octant != other.octant;
    }
};

/**
 * @brief Pattern géométrique 3D (ensemble de points octovalents)
 * 
 * Un pattern est défini par:
 * - Un ensemble de N points (octants)
 * - Un centroïde (centre géométrique)
 * - Des invariants géométriques (distances, angles)
 */
struct OctoPattern {
    std::vector<OctoPoint> points;
    std::string label;  // Nom du pattern (optionnel)
    
    OctoPattern() = default;
    
    explicit OctoPattern(const std::vector<uint8_t>& octants, 
                         const std::string& name = "") 
        : label(name) {
        points.reserve(octants.size());
        for (uint8_t oct : octants) {
            points.emplace_back(oct);
        }
    }
    
    explicit OctoPattern(std::initializer_list<uint8_t> octants,
                         const std::string& name = "")
        : OctoPattern(std::vector<uint8_t>(octants), name) {}
    
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    
    // Centroïde (centre géométrique)
    OctoPoint centroid() const {
        if (empty()) return OctoPoint(0);
        
        uint32_t sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& pt : points) {
            uint8_t x, y, z;
            pt.decode(x, y, z);
            sum_x += x;
            sum_y += y;
            sum_z += z;
        }
        
        uint8_t cx = (sum_x >= points.size() / 2) ? 1 : 0;
        uint8_t cy = (sum_y >= points.size() / 2) ? 1 : 0;
        uint8_t cz = (sum_z >= points.size() / 2) ? 1 : 0;
        
        return OctoPoint(cx, cy, cz);
    }
    
    // Distance moyenne intra-pattern
    float average_distance() const {
        if (points.size() < 2) return 0.0f;
        
        float sum_dist = 0.0f;
        size_t count = 0;
        
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                sum_dist += points[i].distance_to(points[j]);
                ++count;
            }
        }
        
        return (count > 0) ? (sum_dist / count) : 0.0f;
    }
    
    // Rayon (distance max du centroïde)
    float radius() const {
        if (empty()) return 0.0f;
        
        auto center = centroid();
        float max_dist = 0.0f;
        
        for (const auto& pt : points) {
            float dist = pt.distance_to(center);
            if (dist > max_dist) {
                max_dist = dist;
            }
        }
        
        return max_dist;
    }
    
    // Similarity avec autre pattern (0.0 = différent, 1.0 = identique)
    float similarity(const OctoPattern& other) const {
        if (size() != other.size() || empty()) return 0.0f;
        
        // Méthode: comparer distances inter-points (invariant géométrique)
        float dist1 = average_distance();
        float dist2 = other.average_distance();
        
        float diff = std::abs(dist1 - dist2);
        float max_dist = std::max(dist1, dist2);
        
        if (max_dist < 1e-6f) return 1.0f;  // Les deux sont vides
        
        return 1.0f - (diff / max_dist);
    }
};

// ============================================================================
// PATTERNS PRÉDÉFINIS (Géométries Classiques)
// ============================================================================

namespace patterns {

// Patterns géométriques de base (tous les octants du cube)
inline OctoPattern cube() {
    return OctoPattern({0, 1, 2, 3, 4, 5, 6, 7}, "cube");
}

// Tétraèdre (4 sommets alternés du cube)
inline OctoPattern tetrahedron() {
    return OctoPattern({0, 3, 5, 6}, "tetrahedron");
}

// Octaèdre (6 sommets: centres faces cube)
// Approximation avec octants disponibles
inline OctoPattern octahedron() {
    return OctoPattern({1, 2, 4, 3, 5, 6}, "octahedron");
}

// Face inférieure (z=0)
inline OctoPattern bottom_face() {
    return OctoPattern({0, 1, 2, 3}, "bottom_face");
}

// Face supérieure (z=1)
inline OctoPattern top_face() {
    return OctoPattern({4, 5, 6, 7}, "top_face");
}

// Diagonale principale
inline OctoPattern main_diagonal() {
    return OctoPattern({0, 7}, "main_diagonal");
}

// Diagonale secondaire
inline OctoPattern anti_diagonal() {
    return OctoPattern({3, 4}, "anti_diagonal");
}

// Ligne X
inline OctoPattern line_x() {
    return OctoPattern({0, 1}, "line_x");
}

// Ligne Y
inline OctoPattern line_y() {
    return OctoPattern({0, 2}, "line_y");
}

// Ligne Z
inline OctoPattern line_z() {
    return OctoPattern({0, 4}, "line_z");
}

// Pattern aléatoire (pour tests)
inline OctoPattern random(size_t num_points, unsigned seed = 42) {
    std::vector<uint8_t> octants;
    octants.reserve(num_points);
    
    // Pseudo-random simple (sans <random> pour garder header-only)
    unsigned state = seed;
    for (size_t i = 0; i < num_points && i < 8; ++i) {
        state = state * 1664525u + 1013904223u;
        octants.push_back(static_cast<uint8_t>(state % 8));
    }
    
    return OctoPattern(octants, "random");
}

} // namespace patterns

// ============================================================================
// HOPFIELD-POTTS (Mémoire Associative)
// ============================================================================

/**
 * @brief Réseau Hopfield-Potts pour patterns octovalents
 * 
 * Capacité théorique: ~0.47N patterns (N=8 → ~3.8 patterns)
 * États par neurone: 8 (octovalent)
 */
class HopfieldPotts {
public:
    HopfieldPotts(size_t num_neurons = 8) 
        : N_(num_neurons), num_patterns_(0) {
        // Initialiser poids synaptiques [i][j][a][b]
        // i,j: neurones (0..N-1)
        // a,b: états (0..7)
        weights_.resize(N_);
        for (auto& w_i : weights_) {
            w_i.resize(N_);
            for (auto& w_ij : w_i) {
                w_ij.resize(8);
                for (auto& w_ija : w_ij) {
                    w_ija.resize(8, 0.0f);
                }
            }
        }
    }
    
    // Apprendre un pattern (règle de Hebb généralisée)
    void learn(const std::array<uint8_t, 8>& pattern) {
        num_patterns_++;
        float P = static_cast<float>(num_patterns_);
        
        for (size_t i = 0; i < N_; ++i) {
            for (size_t j = i + 1; j < N_; ++j) {
                uint8_t a = pattern[i];
                uint8_t b = pattern[j];
                
                // Règle Hebb: w[i][j][a][b] += (δ(ξ_i, a) - 1/8)(δ(ξ_j, b) - 1/8)
                float delta_i = 1.0f - (1.0f / 8.0f);  // δ(ξ_i, a) = 1
                float delta_j = 1.0f - (1.0f / 8.0f);  // δ(ξ_j, b) = 1
                
                // Incrément pondéré par 1/P
                weights_[i][j][a][b] += (delta_i * delta_j) / P;
                weights_[j][i][b][a] = weights_[i][j][a][b];  // Symétrie
                
                // Décrementer autres états
                for (uint8_t s1 = 0; s1 < 8; ++s1) {
                    for (uint8_t s2 = 0; s2 < 8; ++s2) {
                        if (s1 != a || s2 != b) {
                            float d1 = (s1 == a) ? 1.0f : 0.0f;
                            float d2 = (s2 == b) ? 1.0f : 0.0f;
                            weights_[i][j][s1][s2] += 
                                ((d1 - 1.0f/8.0f) * (d2 - 1.0f/8.0f)) / P;
                            weights_[j][i][s2][s1] = weights_[i][j][s1][s2];
                        }
                    }
                }
            }
        }
    }
    
    // Rappel (recall) à partir d'un pattern bruité
    std::array<uint8_t, 8> recall(std::array<uint8_t, 8> state, 
                                   int max_iterations = 100) const {
        bool converged = false;
        int iter = 0;
        
        while (!converged && iter < max_iterations) {
            converged = true;
            
            for (size_t i = 0; i < N_; ++i) {
                // Calculer énergie pour chaque état possible
                std::array<float, 8> energies{};
                
                for (uint8_t s = 0; s < 8; ++s) {
                    float energy = 0.0f;
                    for (size_t j = 0; j < N_; ++j) {
                        if (j != i) {
                            energy += weights_[i][j][s][state[j]];
                        }
                    }
                    energies[s] = energy;
                }
                
                // Choisir état avec énergie maximale
                uint8_t best_state = 0;
                float max_energy = energies[0];
                for (uint8_t s = 1; s < 8; ++s) {
                    if (energies[s] > max_energy) {
                        max_energy = energies[s];
                        best_state = s;
                    }
                }
                
                if (best_state != state[i]) {
                    state[i] = best_state;
                    converged = false;
                }
            }
            
            ++iter;
        }
        
        return state;
    }
    
    size_t num_neurons() const { return N_; }
    size_t num_patterns() const { return num_patterns_; }
    
private:
    size_t N_;  // Nombre de neurones
    size_t num_patterns_;
    
    // Poids synaptiques: weights_[i][j][a][b]
    // i,j ∈ [0..N-1] (neurones)
    // a,b ∈ [0..7] (états octovalents)
    std::vector<std::vector<std::vector<std::vector<float>>>> weights_;
};

// ============================================================================
// DATABASE DE PATTERNS
// ============================================================================

/**
 * @brief Base de données de patterns avec mémoire associative
 */
class PatternDatabase {
public:
    PatternDatabase() : hopfield_(8) {}
    
    // Ajouter un pattern à la database
    void add_pattern(const OctoPattern& pattern) {
        patterns_.push_back(pattern);
        
        // Convertir pattern en array[8] pour Hopfield
        std::array<uint8_t, 8> hopfield_pattern{};
        for (size_t i = 0; i < 8 && i < pattern.points.size(); ++i) {
            hopfield_pattern[i] = pattern.points[i].octant;
        }
        
        // Apprendre dans Hopfield
        hopfield_.learn(hopfield_pattern);
    }
    
    // Nombre de patterns stockés
    size_t size() const { return patterns_.size(); }
    bool empty() const { return patterns_.empty(); }
    
    // Accès patterns
    const OctoPattern& get_pattern(size_t index) const {
        return patterns_.at(index);
    }
    
    // Rechercher pattern le plus similaire
    size_t find_closest(const OctoPattern& query) const {
        if (empty()) return 0;
        
        size_t best_idx = 0;
        float best_similarity = 0.0f;
        
        for (size_t i = 0; i < patterns_.size(); ++i) {
            float sim = patterns_[i].similarity(query);
            if (sim > best_similarity) {
                best_similarity = sim;
                best_idx = i;
            }
        }
        
        return best_idx;
    }
    
    // Reconnaissance via Hopfield
    std::array<uint8_t, 8> recognize_hopfield(
        const std::array<uint8_t, 8>& noisy_pattern) const {
        return hopfield_.recall(noisy_pattern);
    }
    
    // Statistiques mémoire
    float memory_usage_kb() const {
        // Patterns: N × points × 1 byte
        float patterns_mem = 0.0f;
        for (const auto& p : patterns_) {
            patterns_mem += p.size() * sizeof(uint8_t);
        }
        
        // Hopfield: N × N × 8 × 8 × sizeof(float)
        float hopfield_mem = 8 * 8 * 8 * 8 * sizeof(float);
        
        return (patterns_mem + hopfield_mem) / 1024.0f;
    }
    
private:
    std::vector<OctoPattern> patterns_;
    HopfieldPotts hopfield_;
};

// ============================================================================
// PATTERN RECOGNIZER (Classe Principale)
// ============================================================================

/**
 * @brief Reconnaissance de patterns géométriques octovalents
 */
class PatternRecognizer {
public:
    PatternRecognizer() = default;
    
    // Charger database de patterns
    void load_database(const PatternDatabase& db) {
        database_ = std::make_unique<PatternDatabase>(db);
    }
    
    // Ajouter pattern à database interne
    void add_pattern(const OctoPattern& pattern) {
        if (!database_) {
            database_ = std::make_unique<PatternDatabase>();
        }
        database_->add_pattern(pattern);
    }
    
    // Reconnaître pattern (retourne index + similarity score)
    std::pair<size_t, float> recognize(const OctoPattern& query) const {
        if (!database_ || database_->empty()) {
            return {0, 0.0f};
        }
        
        size_t best_idx = database_->find_closest(query);
        float similarity = database_->get_pattern(best_idx).similarity(query);
        
        return {best_idx, similarity};
    }
    
    // Reconnaître avec label
    std::string recognize_label(const OctoPattern& query) const {
        auto [idx, sim] = recognize(query);
        if (sim < 0.5f) return "unknown";
        return database_->get_pattern(idx).label;
    }
    
    // Statistiques
    size_t num_patterns() const {
        return database_ ? database_->size() : 0;
    }
    
    float memory_usage_kb() const {
        return database_ ? database_->memory_usage_kb() : 0.0f;
    }
    
private:
    std::unique_ptr<PatternDatabase> database_;
};

// ============================================================================
// MÉTRIQUES DE VALIDATION
// ============================================================================

/**
 * @brief Calcul densité sémantique (bits d'information géométrique par byte)
 */
inline float semantic_density_octovalent(const OctoPattern& pattern) {
    if (pattern.empty()) return 0.0f;
    
    // Information géométrique:
    // - Chaque point: 3 bits (x, y, z ∈ {0,1})
    // - Relations inter-points: log2(distances possibles) ≈ 1.58 bits
    //   (3 distances: 1, √2, √3)
    
    float info_points = pattern.size() * 3.0f;  // 3 bits/point
    
    size_t num_pairs = (pattern.size() * (pattern.size() - 1)) / 2;
    float info_relations = num_pairs * 1.58f;  // log2(3) ≈ 1.58
    
    float total_info = info_points + info_relations;
    float storage_bytes = pattern.size() * sizeof(uint8_t);
    
    return total_info / storage_bytes;  // bits/byte
}

/**
 * @brief Calcul densité sémantique binaire (pour comparaison)
 */
inline float semantic_density_binary(size_t num_points) {
    if (num_points == 0) return 0.0f;
    
    // Binary float[3] encoding:
    // - Chaque point: 3 × 32 bits = 96 bits (mais info utile ~3 bits)
    // - Relations: calculées en float (imprécis)
    
    float info_points = num_points * 3.0f;  // Même 3 bits utiles
    
    size_t num_pairs = (num_points * (num_points - 1)) / 2;
    float info_relations = num_pairs * 1.0f;  // Distances continues (moins précis)
    
    float total_info = info_points + info_relations;
    float storage_bytes = num_points * 3 * sizeof(float);  // 12 bytes/point
    
    return total_info / storage_bytes;  // bits/byte
}

}} // namespace ods::pattern_recognition
