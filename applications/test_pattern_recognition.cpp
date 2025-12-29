// test_pattern_recognition.cpp
// Tests Unitaires - Pattern Recognition Géométrique
// Author: Jean-Christophe Ané
// Date: December 2025

#include <gtest/gtest.h>
#include "pattern_recognition_octovalent.hpp"
#include <cmath>

using namespace ods::pattern_recognition;

// ============================================================================
// TEST 1 : OctoPoint Encoding/Decoding
// ============================================================================

TEST(PatternRecognition, OctoPointEncodingDecoding) {
    // Test tous les octants {0..7}
    for (uint8_t x = 0; x <= 1; ++x) {
        for (uint8_t y = 0; y <= 1; ++y) {
            for (uint8_t z = 0; z <= 1; ++z) {
                OctoPoint pt(x, y, z);
                
                uint8_t dx, dy, dz;
                pt.decode(dx, dy, dz);
                
                EXPECT_EQ(dx, x);
                EXPECT_EQ(dy, y);
                EXPECT_EQ(dz, z);
            }
        }
    }
}

// ============================================================================
// TEST 2 : Distances Euclidiennes
// ============================================================================

TEST(PatternRecognition, EuclideanDistances) {
    OctoPoint pt0(0);  // (0,0,0)
    OctoPoint pt1(1);  // (1,0,0)
    OctoPoint pt3(3);  // (1,1,0)
    OctoPoint pt7(7);  // (1,1,1)
    
    // Arête (Hamming = 1)
    EXPECT_FLOAT_EQ(pt0.distance_to(pt1), 1.0f);
    
    // Diagonale face (Hamming = 2)
    EXPECT_FLOAT_EQ(pt0.distance_to(pt3), std::sqrt(2.0f));
    
    // Diagonale espace (Hamming = 3)
    EXPECT_FLOAT_EQ(pt0.distance_to(pt7), std::sqrt(3.0f));
    
    // Distance symétrique
    EXPECT_FLOAT_EQ(pt1.distance_to(pt0), pt0.distance_to(pt1));
}

// ============================================================================
// TEST 3 : Distance Hamming
// ============================================================================

TEST(PatternRecognition, HammingDistance) {
    OctoPoint pt0(0);  // 000
    OctoPoint pt1(1);  // 001
    OctoPoint pt3(3);  // 011
    OctoPoint pt7(7);  // 111
    
    EXPECT_EQ(pt0.hamming_distance(pt1), 1);
    EXPECT_EQ(pt0.hamming_distance(pt3), 2);
    EXPECT_EQ(pt0.hamming_distance(pt7), 3);
    
    EXPECT_EQ(pt1.hamming_distance(pt3), 1);
    EXPECT_EQ(pt3.hamming_distance(pt7), 1);
}

// ============================================================================
// TEST 4 : OctoPattern Création
// ============================================================================

TEST(PatternRecognition, PatternCreation) {
    // Pattern vide
    OctoPattern empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_EQ(empty.size(), 0);
    
    // Pattern avec initializer_list
    OctoPattern cube = patterns::cube();
    EXPECT_FALSE(cube.empty());
    EXPECT_EQ(cube.size(), 8);
    EXPECT_EQ(cube.label, "cube");
    
    // Pattern tétraèdre
    OctoPattern tetra = patterns::tetrahedron();
    EXPECT_EQ(tetra.size(), 4);
    EXPECT_EQ(tetra.label, "tetrahedron");
}

// ============================================================================
// TEST 5 : Centroïde
// ============================================================================

TEST(PatternRecognition, Centroid) {
    // Cube complet → centroïde au centre (0.5, 0.5, 0.5) → (1,1,1)
    OctoPattern cube = patterns::cube();
    OctoPoint center = cube.centroid();
    
    uint8_t cx, cy, cz;
    center.decode(cx, cy, cz);
    
    // Attendu: (1,1,1) car majorité des points > 0.5
    EXPECT_EQ(cx, 1);
    EXPECT_EQ(cy, 1);
    EXPECT_EQ(cz, 1);
    
    // Face inférieure → centroïde (0.5, 0.5, 0) → (1,1,0)
    OctoPattern bottom = patterns::bottom_face();
    OctoPoint bottom_center = bottom.centroid();
    bottom_center.decode(cx, cy, cz);
    
    EXPECT_EQ(cx, 1);
    EXPECT_EQ(cy, 1);
    EXPECT_EQ(cz, 0);
}

// ============================================================================
// TEST 6 : Distance Moyenne Intra-Pattern
// ============================================================================

TEST(PatternRecognition, AverageDistance) {
    // Diagonale (0, 7) → distance = √3
    OctoPattern diag = patterns::main_diagonal();
    float avg_dist = diag.average_distance();
    EXPECT_FLOAT_EQ(avg_dist, std::sqrt(3.0f));
    
    // Ligne X (0, 1) → distance = 1
    OctoPattern line = patterns::line_x();
    float line_dist = line.average_distance();
    EXPECT_FLOAT_EQ(line_dist, 1.0f);
}

// ============================================================================
// TEST 7 : Rayon (Distance Max du Centroïde)
// ============================================================================

TEST(PatternRecognition, Radius) {
    // Cube → rayon ≈ √3 (du centre aux coins)
    OctoPattern cube = patterns::cube();
    float radius = cube.radius();
    
    // Rayon attendu proche de √3 / 2 ≈ 0.866
    // Mais avec octants discrets, peut varier
    EXPECT_GT(radius, 0.0f);
    EXPECT_LE(radius, std::sqrt(3.0f));
}

// ============================================================================
// TEST 8 : Similarity Entre Patterns
// ============================================================================

TEST(PatternRecognition, Similarity) {
    OctoPattern cube1 = patterns::cube();
    OctoPattern cube2 = patterns::cube();
    OctoPattern tetra = patterns::tetrahedron();
    
    // Identiques
    float sim_identical = cube1.similarity(cube2);
    EXPECT_FLOAT_EQ(sim_identical, 1.0f);
    
    // Différents (mais pas totalement)
    float sim_different = cube1.similarity(tetra);
    EXPECT_GE(sim_different, 0.0f);
    EXPECT_LT(sim_different, 1.0f);
}

// ============================================================================
// TEST 9 : Hopfield-Potts Learning
// ============================================================================

TEST(PatternRecognition, HopfieldLearning) {
    HopfieldPotts net(8);
    
    EXPECT_EQ(net.num_neurons(), 8);
    EXPECT_EQ(net.num_patterns(), 0);
    
    // Apprendre pattern
    std::array<uint8_t, 8> pattern1 = {0, 1, 2, 3, 4, 5, 6, 7};
    net.learn(pattern1);
    
    EXPECT_EQ(net.num_patterns(), 1);
    
    // Apprendre deuxième pattern
    std::array<uint8_t, 8> pattern2 = {7, 6, 5, 4, 3, 2, 1, 0};
    net.learn(pattern2);
    
    EXPECT_EQ(net.num_patterns(), 2);
}

// ============================================================================
// TEST 10 : Hopfield-Potts Recall (Exact)
// ============================================================================

TEST(PatternRecognition, HopfieldRecallExact) {
    HopfieldPotts net(8);
    
    // Pattern appris
    std::array<uint8_t, 8> learned = {0, 1, 2, 3, 4, 5, 6, 7};
    net.learn(learned);
    
    // Rappel exact
    auto recalled = net.recall(learned);
    
    for (size_t i = 0; i < 8; ++i) {
        EXPECT_EQ(recalled[i], learned[i]);
    }
}

// ============================================================================
// TEST 11 : Hopfield-Potts Recall (Bruité)
// ============================================================================

TEST(PatternRecognition, HopfieldRecallNoisy) {
    HopfieldPotts net(8);
    
    // Pattern appris (cube)
    std::array<uint8_t, 8> learned = {0, 1, 2, 3, 4, 5, 6, 7};
    net.learn(learned);
    
    // Pattern bruité (2/8 bits corrompus)
    std::array<uint8_t, 8> noisy = {0, 1, 7, 3, 4, 0, 6, 7};  // 2 et 5 modifiés
    
    auto recalled = net.recall(noisy);
    
    // Devrait converger vers pattern appris
    size_t num_correct = 0;
    for (size_t i = 0; i < 8; ++i) {
        if (recalled[i] == learned[i]) {
            ++num_correct;
        }
    }
    
    // Au moins 75% correct
    EXPECT_GE(num_correct, 6);
}

// ============================================================================
// TEST 12 : PatternDatabase Add & Size
// ============================================================================

TEST(PatternRecognition, DatabaseAddPattern) {
    PatternDatabase db;
    
    EXPECT_TRUE(db.empty());
    EXPECT_EQ(db.size(), 0);
    
    // Ajouter patterns
    db.add_pattern(patterns::cube());
    EXPECT_EQ(db.size(), 1);
    
    db.add_pattern(patterns::tetrahedron());
    EXPECT_EQ(db.size(), 2);
    
    db.add_pattern(patterns::octahedron());
    EXPECT_EQ(db.size(), 3);
}

// ============================================================================
// TEST 13 : PatternDatabase Find Closest
// ============================================================================

TEST(PatternRecognition, DatabaseFindClosest) {
    PatternDatabase db;
    
    db.add_pattern(patterns::cube());
    db.add_pattern(patterns::tetrahedron());
    db.add_pattern(patterns::octahedron());
    
    // Query identique à cube
    OctoPattern query_cube = patterns::cube();
    size_t closest = db.find_closest(query_cube);
    EXPECT_EQ(closest, 0);  // Index du cube
    EXPECT_EQ(db.get_pattern(closest).label, "cube");
    
    // Query identique à tetrahedron
    OctoPattern query_tetra = patterns::tetrahedron();
    closest = db.find_closest(query_tetra);
    EXPECT_EQ(closest, 1);  // Index du tetrahedron
    EXPECT_EQ(db.get_pattern(closest).label, "tetrahedron");
}

// ============================================================================
// TEST 14 : PatternRecognizer Basic
// ============================================================================

TEST(PatternRecognition, RecognizerBasic) {
    PatternRecognizer recognizer;
    
    EXPECT_EQ(recognizer.num_patterns(), 0);
    
    // Ajouter patterns
    recognizer.add_pattern(patterns::cube());
    recognizer.add_pattern(patterns::tetrahedron());
    
    EXPECT_EQ(recognizer.num_patterns(), 2);
}

// ============================================================================
// TEST 15 : PatternRecognizer Recognition
// ============================================================================

TEST(PatternRecognition, RecognizerRecognition) {
    PatternRecognizer recognizer;
    
    recognizer.add_pattern(patterns::cube());
    recognizer.add_pattern(patterns::tetrahedron());
    recognizer.add_pattern(patterns::bottom_face());
    
    // Reconnaître cube
    OctoPattern query_cube = patterns::cube();
    auto [idx, similarity] = recognizer.recognize(query_cube);
    
    EXPECT_EQ(idx, 0);  // Cube est le premier
    EXPECT_FLOAT_EQ(similarity, 1.0f);  // Identique
    
    // Reconnaître avec label
    std::string label = recognizer.recognize_label(query_cube);
    EXPECT_EQ(label, "cube");
}

// ============================================================================
// TEST 16 : Semantic Density Octovalent
// ============================================================================

TEST(PatternRecognition, SemanticDensityOctovalent) {
    OctoPattern cube = patterns::cube();
    
    float density = semantic_density_octovalent(cube);
    
    // Densité attendue: élevée (3+ bits/byte)
    EXPECT_GT(density, 2.0f);  // Au moins 2 bits/byte
    
    std::cout << "\n[Semantic Density Octovalent]" << std::endl;
    std::cout << "  Pattern: " << cube.label << " (" << cube.size() << " points)" << std::endl;
    std::cout << "  Density: " << density << " bits/byte" << std::endl;
}

// ============================================================================
// TEST 17 : Semantic Density Binary (Comparaison)
// ============================================================================

TEST(PatternRecognition, SemanticDensityBinary) {
    size_t num_points = 8;  // Cube
    
    float density = semantic_density_binary(num_points);
    
    // Densité attendue: faible (<1 bit/byte)
    EXPECT_LT(density, 1.0f);
    
    std::cout << "\n[Semantic Density Binary]" << std::endl;
    std::cout << "  Num points: " << num_points << std::endl;
    std::cout << "  Density: " << density << " bits/byte" << std::endl;
    
    // Ratio octovalent vs binaire
    float density_octo = semantic_density_octovalent(patterns::cube());
    float ratio = density_octo / density;
    
    std::cout << "\n[RATIO Octovalent/Binary]" << std::endl;
    std::cout << "  Ratio: " << ratio << "× (octovalent better)" << std::endl;
    
    EXPECT_GT(ratio, 3.0f);  // Au moins 3× meilleur
}

// ============================================================================
// TEST 18 : Memory Usage Database
// ============================================================================

TEST(PatternRecognition, MemoryUsage) {
    PatternDatabase db;
    
    db.add_pattern(patterns::cube());
    db.add_pattern(patterns::tetrahedron());
    db.add_pattern(patterns::octahedron());
    
    float mem_kb = db.memory_usage_kb();
    
    std::cout << "\n[Memory Usage]" << std::endl;
    std::cout << "  Patterns: " << db.size() << std::endl;
    std::cout << "  Memory: " << mem_kb << " KB" << std::endl;
    
    // Devrait être < 10 KB
    EXPECT_LT(mem_kb, 10.0f);
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    
    std::cout << "\n=== PATTERN RECOGNITION - TESTS UNITAIRES ===" << std::endl;
    std::cout << "Encoding octovalent géométrique natif\n" << std::endl;
    
    int result = RUN_ALL_TESTS();
    
    std::cout << "\n=== FIN DES TESTS ===" << std::endl;
    
    return result;
}
