#include "pathfinding_octovalent.hpp"
#include <iostream>
#include <iomanip>

using namespace ods::pathfinding;

// ============================================================================
// Affichage ASCII Art d'une couche Z
// ============================================================================

void print_layer(const Octree3D& octree, uint8_t z, 
                 const std::vector<uint16_t>& path = {}) {
    std::cout << "Layer Z=" << static_cast<int>(z) << ":\n";
    std::cout << "  ";
    for (uint8_t x = 0; x < octree.size(); ++x) {
        std::cout << static_cast<int>(x) << " ";
    }
    std::cout << "\n";
    
    for (uint8_t y = 0; y < octree.size(); ++y) {
        std::cout << static_cast<int>(y) << " ";
        for (uint8_t x = 0; x < octree.size(); ++x) {
            uint16_t idx = octree.encode(x, y, z);
            const auto& node = octree.get_node(idx);
            
            char symbol = '.';
            
            // Vérifier si dans le chemin
            bool in_path = std::find(path.begin(), path.end(), idx) != path.end();
            
            if (node.is_obstacle) {
                symbol = '#';
            } else if (in_path) {
                if (idx == path.front()) {
                    symbol = 'S'; // Start
                } else if (idx == path.back()) {
                    symbol = 'E'; // End
                } else {
                    symbol = '*'; // Chemin
                }
            }
            
            std::cout << symbol << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// ============================================================================
// Test 1 : Création Octree
// ============================================================================

bool test_octree_creation() {
    std::cout << "=== TEST 1 : Création Octree ===\n\n";
    
    Octree3D octree(3); // 8×8×8 = 512 nœuds
    
    std::cout << "Profondeur : " << static_cast<int>(octree.depth()) << "\n";
    std::cout << "Taille par dimension : " << static_cast<int>(octree.size()) << "\n";
    std::cout << "Nombre total de nœuds : " << octree.num_nodes() << "\n";
    
    bool success = (octree.num_nodes() == 512);
    std::cout << "Résultat : " << (success ? "✓ PASS" : "✗ FAIL") << "\n\n";
    
    return success;
}

// ============================================================================
// Test 2 : Encodage/Décodage
// ============================================================================

bool test_encoding() {
    std::cout << "=== TEST 2 : Encodage/Décodage ===\n\n";
    
    Octree3D octree(3);
    
    // Test positions connues
    struct TestCase {
        uint8_t x, y, z;
        uint16_t expected_index;
    };
    
    std::vector<TestCase> cases = {
        {0, 0, 0, 0},     // Origine
        {7, 7, 7, 511},   // Coin opposé
        {1, 0, 0, 1},     // +X
        {0, 1, 0, 8},     // +Y
        {0, 0, 1, 64},    // +Z
        {3, 4, 2, 163}    // Position arbitraire (2*64 + 4*8 + 3)
    };
    
    bool all_pass = true;
    
    for (const auto& tc : cases) {
        uint16_t encoded = octree.encode(tc.x, tc.y, tc.z);
        
        uint8_t dx, dy, dz;
        octree.decode(encoded, dx, dy, dz);
        
        bool encode_ok = (encoded == tc.expected_index);
        bool decode_ok = (dx == tc.x && dy == tc.y && dz == tc.z);
        
        std::cout << "Position (" << static_cast<int>(tc.x) << "," 
                  << static_cast<int>(tc.y) << "," << static_cast<int>(tc.z) << ") → Index "
                  << encoded << " (attendu " << tc.expected_index << ") → ("
                  << static_cast<int>(dx) << "," << static_cast<int>(dy) << "," 
                  << static_cast<int>(dz) << ") : "
                  << (encode_ok && decode_ok ? "✓" : "✗") << "\n";
        
        all_pass &= (encode_ok && decode_ok);
    }
    
    std::cout << "Résultat : " << (all_pass ? "✓ PASS" : "✗ FAIL") << "\n\n";
    
    return all_pass;
}

// ============================================================================
// Test 3 : Distance Euclidienne
// ============================================================================

bool test_distances() {
    std::cout << "=== TEST 3 : Distances Euclidiennes ===\n\n";
    
    Octree3D octree(3);
    
    // Test distances connues
    struct DistanceTest {
        uint8_t x1, y1, z1;
        uint8_t x2, y2, z2;
        float expected_distance;
    };
    
    std::vector<DistanceTest> tests = {
        {0, 0, 0, 1, 0, 0, 1.0f},           // Arête X
        {0, 0, 0, 0, 1, 0, 1.0f},           // Arête Y
        {0, 0, 0, 0, 0, 1, 1.0f},           // Arête Z
        {0, 0, 0, 1, 1, 0, std::sqrt(2.0f)}, // Diagonale face
        {0, 0, 0, 1, 1, 1, std::sqrt(3.0f)}, // Diagonale espace
        {0, 0, 0, 7, 7, 7, std::sqrt(147.0f)} // Max distance
    };
    
    bool all_pass = true;
    
    for (const auto& dt : tests) {
        const auto& n1 = octree.get_node(dt.x1, dt.y1, dt.z1);
        const auto& n2 = octree.get_node(dt.x2, dt.y2, dt.z2);
        
        float computed = n1.distance_to(n2);
        float error = std::abs(computed - dt.expected_distance);
        bool pass = (error < 1e-5f);
        
        std::cout << "Distance (" << static_cast<int>(dt.x1) << "," 
                  << static_cast<int>(dt.y1) << "," << static_cast<int>(dt.z1) << ") → ("
                  << static_cast<int>(dt.x2) << "," << static_cast<int>(dt.y2) << "," 
                  << static_cast<int>(dt.z2) << ") : "
                  << computed << " (attendu " << dt.expected_distance << ") : "
                  << (pass ? "✓" : "✗") << "\n";
        
        all_pass &= pass;
    }
    
    std::cout << "Résultat : " << (all_pass ? "✓ PASS" : "✗ FAIL") << "\n\n";
    
    return all_pass;
}

// ============================================================================
// Test 4 : Voisinage
// ============================================================================

bool test_neighbors() {
    std::cout << "=== TEST 4 : Voisinage ===\n\n";
    
    Octree3D octree(3);
    
    // Centre du cube (3,3,3)
    uint16_t center = octree.encode(3, 3, 3);
    auto neighbors = octree.get_neighbors(center);
    
    std::cout << "Nœud central (3,3,3) - Index " << center << "\n";
    std::cout << "Nombre de voisins : " << neighbors.size() << " (attendu 26)\n";
    
    bool pass = (neighbors.size() == 26);
    
    // Coin (0,0,0) - seulement 7 voisins
    uint16_t corner = octree.encode(0, 0, 0);
    auto corner_neighbors = octree.get_neighbors(corner);
    
    std::cout << "Nœud coin (0,0,0) - Index " << corner << "\n";
    std::cout << "Nombre de voisins : " << corner_neighbors.size() << " (attendu 7)\n";
    
    pass &= (corner_neighbors.size() == 7);
    
    std::cout << "Résultat : " << (pass ? "✓ PASS" : "✗ FAIL") << "\n\n";
    
    return pass;
}

// ============================================================================
// Test 5 : Pathfinding Simple (ligne droite)
// ============================================================================

bool test_pathfinding_simple() {
    std::cout << "=== TEST 5 : Pathfinding Simple ===\n\n";
    
    Octree3D octree(3);
    AStarOctovalent astar(octree);
    
    // Chemin (0,0,0) → (7,7,7) sans obstacles
    uint16_t start = octree.encode(0, 0, 0);
    uint16_t goal = octree.encode(7, 7, 7);
    
    std::cout << "Recherche chemin : (0,0,0) → (7,7,7)\n";
    std::cout << "Obstacles : Aucun\n\n";
    
    auto path = astar.find_path(start, goal);
    
    std::cout << "Chemin trouvé : " << (path.empty() ? "NON" : "OUI") << "\n";
    std::cout << "Longueur chemin : " << path.size() << " nœuds\n";
    std::cout << "Nœuds explorés : " << astar.nodes_explored() << "\n";
    
    if (!path.empty()) {
        // Calculer distance totale
        float total_distance = 0.0f;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            const auto& n1 = octree.get_node(path[i]);
            const auto& n2 = octree.get_node(path[i+1]);
            total_distance += n1.distance_to(n2);
        }
        
        std::cout << "Distance totale : " << total_distance << "\n";
        std::cout << "Distance directe : " << octree.get_node(start).distance_to(octree.get_node(goal)) << "\n";
        
        // Afficher chemin
        std::cout << "\nChemin détaillé :\n";
        for (size_t i = 0; i < std::min(path.size(), size_t(10)); ++i) {
            uint8_t x, y, z;
            octree.decode(path[i], x, y, z);
            std::cout << "  " << i << ": Index " << path[i] << " → ("
                      << static_cast<int>(x) << "," << static_cast<int>(y) << "," 
                      << static_cast<int>(z) << ")\n";
        }
        if (path.size() > 10) {
            std::cout << "  ... (" << (path.size() - 10) << " nœuds supplémentaires)\n";
        }
    }
    
    bool pass = !path.empty() && path.front() == start && path.back() == goal;
    std::cout << "\nRésultat : " << (pass ? "✓ PASS" : "✗ FAIL") << "\n\n";
    
    return pass;
}

// ============================================================================
// Test 6 : Pathfinding avec Obstacles
// ============================================================================

bool test_pathfinding_obstacles() {
    std::cout << "=== TEST 6 : Pathfinding avec Obstacles ===\n\n";
    
    Octree3D octree(3);
    
    // Créer un mur vertical en X=3, Z=0
    for (uint8_t y = 0; y < 8; ++y) {
        octree.set_obstacle(3, y, 0, true);
    }
    
    AStarOctovalent astar(octree);
    
    // Chemin (0,0,0) → (7,0,0) - doit contourner le mur
    uint16_t start = octree.encode(0, 0, 0);
    uint16_t goal = octree.encode(7, 0, 0);
    
    std::cout << "Recherche chemin : (0,0,0) → (7,0,0)\n";
    std::cout << "Obstacles : Mur vertical en X=3, Z=0\n\n";
    
    auto path = astar.find_path(start, goal);
    
    std::cout << "Chemin trouvé : " << (path.empty() ? "NON" : "OUI") << "\n";
    
    if (!path.empty()) {
        std::cout << "Longueur chemin : " << path.size() << " nœuds\n";
        std::cout << "Nœuds explorés : " << astar.nodes_explored() << "\n";
        
        // Vérifier que le chemin contourne (monte en Z)
        bool went_up = false;
        for (uint16_t idx : path) {
            uint8_t x, y, z;
            octree.decode(idx, x, y, z);
            if (z > 0) {
                went_up = true;
                break;
            }
        }
        
        std::cout << "Contournement en Z : " << (went_up ? "OUI" : "NON") << "\n";
        
        // Afficher layer Z=0 avec chemin
        std::cout << "\nVisualisation Layer Z=0 :\n";
        print_layer(octree, 0, path);
    }
    
    bool pass = !path.empty();
    std::cout << "Résultat : " << (pass ? "✓ PASS" : "✗ FAIL") << "\n\n";
    
    return pass;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  TEST SUITE : Pathfinding Octovalent                      ║\n";
    std::cout << "║  3ODS - Layer 5 : OctoIA                                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
    
    int passed = 0;
    int total = 6;
    
    passed += test_octree_creation() ? 1 : 0;
    passed += test_encoding() ? 1 : 0;
    passed += test_distances() ? 1 : 0;
    passed += test_neighbors() ? 1 : 0;
    passed += test_pathfinding_simple() ? 1 : 0;
    passed += test_pathfinding_obstacles() ? 1 : 0;
    
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  RÉSUMÉ                                                    ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
    std::cout << "Tests réussis : " << passed << "/" << total << "\n";
    std::cout << "Taux de succès : " << (100 * passed / total) << "%\n";
    std::cout << "\n";
    
    if (passed == total) {
        std::cout << "✓ TOUS LES TESTS PASSENT - Implémentation validée !\n";
        return 0;
    } else {
        std::cout << "✗ ÉCHECS DÉTECTÉS - Vérifier l'implémentation\n";
        return 1;
    }
}
