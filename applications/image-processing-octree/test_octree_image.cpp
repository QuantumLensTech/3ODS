// test_octree_image.cpp - TESTS COMPLETS OCTREE
#include "image_processing_octree_v2.hpp"
#include <iostream>
#include <cassert>
#include <cstring>
#include <cmath>

using namespace ods::image_processing;

// Helpers
void print_test(const char* name, bool passed) {
    std::cout << (passed ? "✅" : "❌") << " " << name << "\n";
}

// Test 1 : Création image
bool test_image_creation() {
    OctoImage img(64, 64);
    return img.width() == 64 && img.height() == 64 && img.num_pixels() == 0;
}

// Test 2 : Insertion pixel unique
bool test_single_pixel() {
    OctoImage img(64, 64);
    img.set_pixel(10, 20, 128);
    
    return img.num_pixels() == 1 && img.get_pixel(10, 20) == 128;
}

// Test 3 : Insertion multiples pixels
bool test_multiple_pixels() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 20, 100);
    img.set_pixel(30, 40, 200);
    img.set_pixel(50, 60, 150);
    
    return img.num_pixels() == 3 &&
           img.get_pixel(10, 20) == 100 &&
           img.get_pixel(30, 40) == 200 &&
           img.get_pixel(50, 60) == 150;
}

// Test 4 : Update pixel existant
bool test_pixel_update() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 20, 100);
    assert(img.get_pixel(10, 20) == 100);
    
    img.set_pixel(10, 20, 200);  // Update
    
    return img.num_pixels() == 1 && img.get_pixel(10, 20) == 200;
}

// Test 5 : Pixels hors limites
bool test_out_of_bounds() {
    OctoImage img(64, 64);
    
    img.set_pixel(100, 100, 128);  // Devrait être ignoré
    
    return img.num_pixels() == 0 && img.get_pixel(100, 100) == 0;
}

// Test 6 : Sparse storage (pixels noirs ignorés)
bool test_sparse_storage() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 20, 0);    // Noir : ignoré
    img.set_pixel(30, 40, 100);  // Stocké
    
    return img.num_pixels() == 1;
}

// Test 7 : Subdivision octree
bool test_octree_subdivision() {
    OctoImage img(64, 64);
    
    // Insérer beaucoup de pixels pour forcer subdivision
    for (int i = 0; i < 20; ++i) {
        img.set_pixel(i, i, 100 + i);
    }
    
    // Vérifier que subdivision a eu lieu
    bool has_subdivided = img.num_nodes() > 1;
    
    // Vérifier que tous les pixels sont accessibles
    bool all_accessible = true;
    for (int i = 0; i < 20; ++i) {
        if (img.get_pixel(i, i) != static_cast<uint8_t>(100 + i)) {
            all_accessible = false;
            break;
        }
    }
    
    return has_subdivided && all_accessible && img.num_pixels() == 20;
}

// Test 8 : Query rectangle simple
bool test_query_rect_simple() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 10, 100);
    img.set_pixel(20, 20, 200);
    img.set_pixel(30, 30, 150);
    
    // Query (5, 5) → (25, 25) devrait capturer 2 pixels
    auto results = img.query_rect(5, 5, 25, 25);
    
    return results.size() == 2;
}

// Test 9 : Query rectangle vide
bool test_query_rect_empty() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 10, 100);
    img.set_pixel(20, 20, 200);
    
    // Query zone sans pixels
    auto results = img.query_rect(40, 40, 50, 50);
    
    return results.empty();
}

// Test 10 : Query rectangle complète
bool test_query_rect_full() {
    OctoImage img(64, 64);
    
    for (int i = 0; i < 10; ++i) {
        img.set_pixel(i * 6, i * 6, 100 + i * 10);
    }
    
    // Query toute l'image
    auto results = img.query_rect(0, 0, 63, 63);
    
    return results.size() == 10;
}

// Test 11 : Get all pixels
bool test_get_all_pixels() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 10, 100);
    img.set_pixel(20, 20, 200);
    img.set_pixel(30, 30, 150);
    
    auto all = img.get_all_pixels();
    
    return all.size() == 3;
}

// Test 12 : Statistiques octree
bool test_statistics() {
    OctoImage img(64, 64);
    
    // Forcer subdivision avec beaucoup de pixels
    for (int i = 0; i < 50; ++i) {
        img.set_pixel(i, i, 100);
    }
    
    return img.num_pixels() == 50 &&
           img.num_nodes() >= 1 &&
           img.num_leaves() >= 1 &&
           img.tree_depth() >= 0;
}

// Test 13 : Sparsity calculation
bool test_sparsity() {
    OctoImage img(64, 64);
    
    // 10 pixels sur 64×64 = 4096 total
    for (int i = 0; i < 10; ++i) {
        img.set_pixel(i * 6, i * 6, 100);
    }
    
    float sparsity = img.sparsity();
    float expected = 1.0f - 10.0f / 4096.0f;
    
    return std::abs(sparsity - expected) < 0.001f;
}

// Test 14 : Clear image
bool test_clear() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 10, 100);
    img.set_pixel(20, 20, 200);
    
    img.clear();
    
    return img.num_pixels() == 0 && img.get_pixel(10, 10) == 0;
}

// Test 15 : Invert operation
bool test_invert() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 10, 100);
    img.set_pixel(20, 20, 200);
    
    img.invert();
    
    return img.get_pixel(10, 10) == 155 && img.get_pixel(20, 20) == 55;
}

// Test 16 : Threshold operation
bool test_threshold() {
    OctoImage img(64, 64);
    
    img.set_pixel(10, 10, 50);
    img.set_pixel(20, 20, 150);
    img.set_pixel(30, 30, 200);
    
    img.threshold(100);  // Seuil à 100
    
    // 50 → 0 (supprimé car sparse)
    // 150 → 255
    // 200 → 255
    
    return img.num_pixels() == 2 &&
           img.get_pixel(10, 10) == 0 &&
           img.get_pixel(20, 20) == 255 &&
           img.get_pixel(30, 30) == 255;
}

// Test 17 : From grayscale buffer
bool test_from_grayscale() {
    const int size = 16;
    uint8_t buffer[size * size];
    std::memset(buffer, 0, sizeof(buffer));
    
    // Quelques pixels non-zéro
    buffer[5 * size + 5] = 100;
    buffer[10 * size + 10] = 200;
    buffer[15 * size + 15] = 150;
    
    OctoImage img = OctoImage::from_grayscale(buffer, size, size);
    
    return img.width() == size &&
           img.height() == size &&
           img.num_pixels() == 3 &&
           img.get_pixel(5, 5) == 100 &&
           img.get_pixel(10, 10) == 200;
}

// Test 18 : To grayscale buffer
bool test_to_grayscale() {
    OctoImage img(16, 16);
    
    img.set_pixel(5, 5, 100);
    img.set_pixel(10, 10, 200);
    
    uint8_t buffer[16 * 16];
    img.to_grayscale(buffer);
    
    return buffer[5 * 16 + 5] == 100 &&
           buffer[10 * 16 + 10] == 200 &&
           buffer[0] == 0;  // Le reste est noir
}

// Test 19 : Large image stress test
bool test_large_image() {
    OctoImage img(256, 256);
    
    // Insérer 1000 pixels aléatoires
    for (int i = 0; i < 1000; ++i) {
        uint16_t x = (i * 73) % 256;
        uint16_t y = (i * 97) % 256;
        img.set_pixel(x, y, (i % 200) + 50);
    }
    
    // Vérifier subdivision octree
    bool subdivided = img.num_nodes() > 10;
    
    // Vérifier récupération pixels
    auto all = img.get_all_pixels();
    bool correct_count = all.size() <= 1000;  // <= car duplicates possibles
    
    return subdivided && correct_count;
}

// Test 20 : BBox intersection logic
bool test_bbox_intersection() {
    BBox b1(0, 0, 10, 10);
    BBox b2(5, 5, 15, 15);
    BBox b3(20, 20, 30, 30);
    
    return b1.intersects(b2) && !b1.intersects(b3);
}

// Test 21 : BBox contains logic
bool test_bbox_contains() {
    BBox b(10, 10, 20, 20);
    
    return b.contains(15, 15) && !b.contains(5, 5) && !b.contains(25, 25);
}

// Test 22 : Octree depth progression
bool test_depth_progression() {
    OctoImage img(64, 64);
    
    // Insérer progressivement pour voir depth augmenter
    img.set_pixel(0, 0, 100);
    uint8_t depth1 = img.tree_depth();
    
    for (int i = 0; i < 10; ++i) {
        img.set_pixel(i, i, 100);
    }
    uint8_t depth2 = img.tree_depth();
    
    for (int i = 0; i < 100; ++i) {
        img.set_pixel(i % 64, i / 64, 100);
    }
    uint8_t depth3 = img.tree_depth();
    
    // Depth devrait augmenter avec plus de pixels
    return depth1 <= depth2 && depth2 <= depth3;
}

int main() {
    std::cout << "\n=== OCTREE IMAGE PROCESSING TESTS ===\n\n";
    
    int total = 0, passed = 0;
    
    auto run_test = [&](const char* name, bool (*test_fn)()) {
        total++;
        bool result = test_fn();
        if (result) passed++;
        print_test(name, result);
    };
    
    // Tests de base
    run_test("Image Creation", test_image_creation);
    run_test("Single Pixel", test_single_pixel);
    run_test("Multiple Pixels", test_multiple_pixels);
    run_test("Pixel Update", test_pixel_update);
    run_test("Out of Bounds", test_out_of_bounds);
    run_test("Sparse Storage", test_sparse_storage);
    
    // Tests octree
    run_test("Octree Subdivision", test_octree_subdivision);
    run_test("Query Rect Simple", test_query_rect_simple);
    run_test("Query Rect Empty", test_query_rect_empty);
    run_test("Query Rect Full", test_query_rect_full);
    run_test("Get All Pixels", test_get_all_pixels);
    
    // Tests statistiques
    run_test("Statistics", test_statistics);
    run_test("Sparsity Calculation", test_sparsity);
    
    // Tests opérations
    run_test("Clear Image", test_clear);
    run_test("Invert Operation", test_invert);
    run_test("Threshold Operation", test_threshold);
    
    // Tests import/export
    run_test("From Grayscale Buffer", test_from_grayscale);
    run_test("To Grayscale Buffer", test_to_grayscale);
    
    // Tests stress
    run_test("Large Image (1000 pixels)", test_large_image);
    
    // Tests géométrie
    run_test("BBox Intersection", test_bbox_intersection);
    run_test("BBox Contains", test_bbox_contains);
    run_test("Depth Progression", test_depth_progression);
    
    std::cout << "\n=== RÉSULTATS ===\n";
    std::cout << "Tests passés : " << passed << "/" << total << "\n";
    std::cout << (passed == total ? "✅ ALL PASS\n" : "⚠️ SOME FAILURES\n");
    
    return (passed == total) ? 0 : 1;
}
