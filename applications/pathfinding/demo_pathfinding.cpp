#include "pathfinding_octovalent.hpp"
#include <iostream>
#include <iomanip>

using namespace ods::pathfinding;

// ============================================================================
// Affichage ASCII Art 3D (toutes les couches Z)
// ============================================================================

void print_all_layers(const Octree3D& octree, const std::vector<uint16_t>& path = {}) {
    for (uint8_t z = 0; z < octree.size(); ++z) {
        std::cout << "╔";
        for (uint8_t x = 0; x < octree.size(); ++x) {
            std::cout << "══";
        }
        std::cout << "═╗\n";
        
        std::cout << "║ Layer Z=" << static_cast<int>(z) << " ";
        for (uint8_t x = 1; x < octree.size(); ++x) {
            std::cout << "  ";
        }
        std::cout << "║\n";
        
        std::cout << "╠";
        for (uint8_t x = 0; x < octree.size(); ++x) {
            std::cout << "══";
        }
        std::cout << "═╣\n";
        
        for (uint8_t y = 0; y < octree.size(); ++y) {
            std::cout << "║ ";
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
            std::cout << "║\n";
        }
        
        std::cout << "╚";
        for (uint8_t x = 0; x < octree.size(); ++x) {
            std::cout << "══";
        }
        std::cout << "═╝\n\n";
    }
}

// ============================================================================
// Statistiques Chemin
// ============================================================================

void print_path_stats(const Octree3D& octree, 
                      const std::vector<uint16_t>& path,
                      const AStarOctovalent& astar) {
    if (path.empty()) {
        std::cout << "❌ Aucun chemin trouvé !\n";
        return;
    }
    
    // Calculer distance totale
    float total_distance = 0.0f;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const auto& n1 = octree.get_node(path[i]);
        const auto& n2 = octree.get_node(path[i+1]);
        total_distance += n1.distance_to(n2);
    }
    
    // Distance directe (à vol d'oiseau)
    const auto& start_node = octree.get_node(path.front());
    const auto& goal_node = octree.get_node(path.back());
    float direct_distance = start_node.distance_to(goal_node);
    
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  STATISTIQUES CHEMIN                                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "✓ Chemin trouvé : OUI\n";
    std::cout << "  Longueur        : " << path.size() << " nœuds\n";
    std::cout << "  Distance totale : " << std::fixed << std::setprecision(3) 
              << total_distance << "\n";
    std::cout << "  Distance directe: " << std::fixed << std::setprecision(3)
              << direct_distance << "\n";
    std::cout << "  Ratio détour    : " << std::fixed << std::setprecision(2)
              << (total_distance / direct_distance) << "×\n";
    std::cout << "  Nœuds explorés  : " << astar.nodes_explored() << "\n";
    
    // Afficher chemin détaillé
    std::cout << "\n  Chemin détaillé :\n";
    for (size_t i = 0; i < path.size(); ++i) {
        uint8_t x, y, z;
        octree.decode(path[i], x, y, z);
        
        std::cout << "    " << std::setw(2) << i << ": "
                  << "Index " << std::setw(3) << path[i] << " → ("
                  << static_cast<int>(x) << "," 
                  << static_cast<int>(y) << "," 
                  << static_cast<int>(z) << ")";
        
        if (i == 0) {
            std::cout << "  [START]";
        } else if (i == path.size() - 1) {
            std::cout << "  [GOAL]";
        }
        
        std::cout << "\n";
    }
    
    std::cout << "\n";
}

// ============================================================================
// Légende
// ============================================================================

void print_legend() {
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  LÉGENDE                                                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    std::cout << "  S = Start (départ)\n";
    std::cout << "  E = End (arrivée)\n";
    std::cout << "  * = Chemin trouvé\n";
    std::cout << "  # = Obstacle (mur)\n";
    std::cout << "  . = Espace libre\n\n";
}

// ============================================================================
// Scénarios Prédéfinis
// ============================================================================

void scenario_1_straight_line() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  SCÉNARIO 1 : LIGNE DROITE                                 ║\n";
    std::cout << "║  (0,0,0) → (7,7,7) sans obstacles                          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    
    Octree3D octree(3);
    AStarOctovalent astar(octree);
    
    uint16_t start = octree.encode(0, 0, 0);
    uint16_t goal = octree.encode(7, 7, 7);
    
    auto path = astar.find_path(start, goal);
    
    print_legend();
    print_all_layers(octree, path);
    print_path_stats(octree, path, astar);
}

void scenario_2_maze() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  SCÉNARIO 2 : LABYRINTHE SIMPLE                            ║\n";
    std::cout << "║  (0,0,0) → (7,0,0) avec mur vertical                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    
    Octree3D octree(3);
    
    // Créer mur vertical en X=3, Z=0
    for (uint8_t y = 0; y < 8; ++y) {
        octree.set_obstacle(3, y, 0, true);
    }
    
    AStarOctovalent astar(octree);
    
    uint16_t start = octree.encode(0, 0, 0);
    uint16_t goal = octree.encode(7, 0, 0);
    
    auto path = astar.find_path(start, goal);
    
    print_legend();
    
    // Afficher seulement layers Z=0 et Z=1 (pertinents)
    std::cout << "Layer Z=0 (avec mur) :\n";
    for (uint8_t y = 0; y < octree.size(); ++y) {
        std::cout << "  ";
        for (uint8_t x = 0; x < octree.size(); ++x) {
            uint16_t idx = octree.encode(x, y, 0);
            const auto& node = octree.get_node(idx);
            
            char symbol = '.';
            bool in_path = std::find(path.begin(), path.end(), idx) != path.end();
            
            if (node.is_obstacle) {
                symbol = '#';
            } else if (in_path) {
                if (idx == path.front()) symbol = 'S';
                else if (idx == path.back()) symbol = 'E';
                else symbol = '*';
            }
            
            std::cout << symbol << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
    
    print_path_stats(octree, path, astar);
}

void scenario_3_complex_maze() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  SCÉNARIO 3 : LABYRINTHE COMPLEXE                          ║\n";
    std::cout << "║  (0,0,0) → (7,7,0) avec passages étroits                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    
    Octree3D octree(3);
    
    // Créer labyrinthe avec passages
    // Murs horizontaux
    for (uint8_t x = 1; x < 7; ++x) {
        octree.set_obstacle(x, 2, 0, true);
        octree.set_obstacle(x, 5, 0, true);
    }
    
    // Murs verticaux
    for (uint8_t y = 0; y < 8; ++y) {
        if (y != 2 && y != 5) {
            octree.set_obstacle(3, y, 0, true);
        }
    }
    
    AStarOctovalent astar(octree);
    
    uint16_t start = octree.encode(0, 0, 0);
    uint16_t goal = octree.encode(7, 7, 0);
    
    auto path = astar.find_path(start, goal);
    
    print_legend();
    
    std::cout << "Layer Z=0 (labyrinthe) :\n";
    for (uint8_t y = 0; y < octree.size(); ++y) {
        std::cout << "  ";
        for (uint8_t x = 0; x < octree.size(); ++x) {
            uint16_t idx = octree.encode(x, y, 0);
            const auto& node = octree.get_node(idx);
            
            char symbol = '.';
            bool in_path = std::find(path.begin(), path.end(), idx) != path.end();
            
            if (node.is_obstacle) {
                symbol = '#';
            } else if (in_path) {
                if (idx == path.front()) symbol = 'S';
                else if (idx == path.back()) symbol = 'E';
                else symbol = '*';
            }
            
            std::cout << symbol << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
    
    print_path_stats(octree, path, astar);
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  DÉMO : Pathfinding Octovalent 3D                          ║\n";
    std::cout << "║  3ODS - Layer 5 : OctoIA                                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    
    // Scénarios
    scenario_1_straight_line();
    std::cout << "\n\n";
    
    scenario_2_maze();
    std::cout << "\n\n";
    
    scenario_3_complex_maze();
    std::cout << "\n\n";
    
    // Conclusion
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CONCLUSION                                                ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    std::cout << "✓ Pathfinding octovalent natif dans octree 3D\n";
    std::cout << "✓ Distances euclidiennes exactes (1, √2, √3)\n";
    std::cout << "✓ Chemins optimaux garantis (A* admissible)\n";
    std::cout << "✓ Structure native (pas d'émulation float)\n";
    std::cout << "✓ Mémoire compacte (uint8_t vs float[3])\n";
    std::cout << "\n";
    
    return 0;
}
