// test_quantumlens.cpp - TESTS COMPLETS
#include "quantumlens_core.hpp"
#include "quantumlens_ascii.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace ods::quantumlens;

void print_test(const char* name, bool passed) {
    std::cout << (passed ? "✅" : "❌") << " " << name << "\n";
}

// Test 1 : Vector3 operations
bool test_vector3_operations() {
    Vector3 v1(1, 2, 3);
    Vector3 v2(4, 5, 6);
    
    Vector3 sum = v1 + v2;
    Vector3 diff = v2 - v1;
    Vector3 scaled = v1 * 2.0f;
    
    return sum.x == 5 && sum.y == 7 && sum.z == 9 &&
           diff.x == 3 && diff.y == 3 && diff.z == 3 &&
           scaled.x == 2 && scaled.y == 4 && scaled.z == 6;
}

// Test 2 : Vector3 length & normalization
bool test_vector3_length() {
    Vector3 v(3, 4, 0);
    float len = v.length();
    Vector3 norm = v.normalized();
    
    return std::abs(len - 5.0f) < 0.01f &&
           std::abs(norm.length() - 1.0f) < 0.01f;
}

// Test 3 : Vector3 dot & cross product
bool test_vector3_dot_cross() {
    Vector3 v1(1, 0, 0);
    Vector3 v2(0, 1, 0);
    
    float dot = v1.dot(v2);
    Vector3 cross = v1.cross(v2);
    
    return std::abs(dot) < 0.01f &&
           std::abs(cross.x) < 0.01f &&
           std::abs(cross.y) < 0.01f &&
           std::abs(cross.z - 1.0f) < 0.01f;
}

// Test 4 : BBox3D creation
bool test_bbox_creation() {
    BBox3D bbox(Vector3(0, 0, 0), Vector3(10, 10, 10));
    Vector3 center = bbox.center();
    Vector3 size = bbox.size();
    
    return std::abs(center.x - 5.0f) < 0.01f &&
           std::abs(size.x - 10.0f) < 0.01f &&
           std::abs(bbox.volume() - 1000.0f) < 0.1f;
}

// Test 5 : BBox3D contains
bool test_bbox_contains() {
    BBox3D bbox(Vector3(0, 0, 0), Vector3(10, 10, 10));
    
    return bbox.contains(Vector3(5, 5, 5)) &&
           !bbox.contains(Vector3(15, 15, 15)) &&
           !bbox.contains(Vector3(-1, 5, 5));
}

// Test 6 : BBox3D intersection
bool test_bbox_intersection() {
    BBox3D b1(Vector3(0, 0, 0), Vector3(10, 10, 10));
    BBox3D b2(Vector3(5, 5, 5), Vector3(15, 15, 15));
    BBox3D b3(Vector3(20, 20, 20), Vector3(30, 30, 30));
    
    return b1.intersects(b2) && !b1.intersects(b3);
}

// Test 7 : BBox3D distance
bool test_bbox_distance() {
    BBox3D bbox(Vector3(0, 0, 0), Vector3(10, 10, 10));
    
    float dist_inside = bbox.distance_to(Vector3(5, 5, 5));
    float dist_outside = bbox.distance_to(Vector3(15, 5, 5));
    
    return std::abs(dist_inside) < 0.01f &&
           std::abs(dist_outside - 5.0f) < 0.01f;
}

// Test 8 : OctreeNode3D creation
bool test_octree_creation() {
    OctreeNode3D node(BBox3D(Vector3(0, 0, 0), Vector3(10, 10, 10)));
    
    return node.is_leaf() && 
           node.num_points() == 0 &&
           node.level() == 0;
}

// Test 9 : OctreeNode3D insertion
bool test_octree_insertion() {
    OctreeNode3D node(BBox3D(Vector3(0, 0, 0), Vector3(10, 10, 10)));
    
    OctoDataPoint p1(Vector3(5, 5, 5), 1.0f);
    OctoDataPoint p2(Vector3(7, 7, 7), 0.5f);
    
    bool inserted1 = node.insert(p1);
    bool inserted2 = node.insert(p2);
    
    return inserted1 && inserted2 && node.num_points() == 2;
}

// Test 10 : OctreeNode3D out of bounds
bool test_octree_out_of_bounds() {
    OctreeNode3D node(BBox3D(Vector3(0, 0, 0), Vector3(10, 10, 10)));
    
    OctoDataPoint p_outside(Vector3(15, 15, 15), 1.0f);
    bool inserted = node.insert(p_outside);
    
    return !inserted && node.num_points() == 0;
}

// Test 11 : OctreeNode3D subdivision
bool test_octree_subdivision() {
    OctreeNode3D node(BBox3D(Vector3(0, 0, 0), Vector3(10, 10, 10)));
    
    // Insert > 8 points to trigger subdivision
    for (int i = 0; i < 20; ++i) {
        OctoDataPoint p(Vector3(i * 0.5f, i * 0.5f, i * 0.5f), 1.0f);
        node.insert(p);
    }
    
    return !node.is_leaf() && node.count_nodes() > 1;
}

// Test 12 : OctreeNode3D query
bool test_octree_query() {
    OctreeNode3D node(BBox3D(Vector3(0, 0, 0), Vector3(10, 10, 10)));
    
    node.insert(OctoDataPoint(Vector3(2, 2, 2), 1.0f));
    node.insert(OctoDataPoint(Vector3(5, 5, 5), 1.0f));
    node.insert(OctoDataPoint(Vector3(8, 8, 8), 1.0f));
    
    std::vector<OctoDataPoint> results;
    BBox3D query(Vector3(0, 0, 0), Vector3(6, 6, 6));
    node.query(query, results);
    
    return results.size() == 2;  // Should find (2,2,2) and (5,5,5)
}

// Test 13 : OctreeNode3D statistics
bool test_octree_statistics() {
    OctreeNode3D node(BBox3D(Vector3(0, 0, 0), Vector3(10, 10, 10)));
    
    for (int i = 0; i < 50; ++i) {
        node.insert(OctoDataPoint(Vector3(i * 0.2f, i * 0.2f, i * 0.2f), 1.0f));
    }
    
    return node.count_points() == 50 &&
           node.count_nodes() >= 1 &&
           node.max_depth() >= 0;
}

// Test 14 : Camera creation
bool test_camera_creation() {
    Camera camera;
    
    return camera.position().z > 0 &&
           camera.level() == 0;
}

// Test 15 : Camera movement
bool test_camera_movement() {
    Camera camera;
    Vector3 initial_pos = camera.position();
    
    camera.move_forward(1.0f);
    Vector3 new_pos = camera.position();
    
    return (new_pos - initial_pos).length() > 0.5f;
}

// Test 16 : Camera rotation
bool test_camera_rotation() {
    Camera camera;
    camera.set_position(Vector3(0, 0, 10));
    camera.set_target(Vector3(0, 0, 0));
    
    Vector3 initial_pos = camera.position();
    float initial_dist = (initial_pos - camera.target()).length();
    
    camera.rotate_around_target(45, 0);
    Vector3 rotated_pos = camera.position();
    float rotated_dist = (rotated_pos - camera.target()).length();
    
    // Check distance preserved (within 10% tolerance)
    float dist_diff = std::abs(rotated_dist - initial_dist);
    return dist_diff < initial_dist * 0.1f;
}

// Test 17 : Camera zoom
bool test_camera_zoom() {
    Camera camera;
    Vector3 initial_pos = camera.position();
    float initial_dist = (initial_pos - camera.target()).length();
    
    camera.zoom(0.5f);  // Zoom in
    Vector3 zoomed_pos = camera.position();
    float zoomed_dist = (zoomed_pos - camera.target()).length();
    
    return zoomed_dist < initial_dist;
}

// Test 18 : Camera level
bool test_camera_level() {
    Camera camera;
    
    camera.set_level(5);
    assert(camera.level() == 5);
    
    camera.level_up();
    assert(camera.level() == 6);
    
    camera.level_down();
    return camera.level() == 5;
}

// Test 19 : OctoSpace creation
bool test_octospace_creation() {
    OctoSpace space;
    
    return space.num_points() == 0 &&
           space.num_nodes() == 1;
}

// Test 20 : OctoSpace add points
bool test_octospace_add_points() {
    OctoSpace space;
    
    space.add_point(OctoDataPoint(Vector3(0, 0, 0), 1.0f));
    space.add_point(OctoDataPoint(Vector3(5, 5, 5), 0.5f));
    
    return space.num_points() == 2;
}

// Test 21 : OctoSpace query bbox
bool test_octospace_query() {
    OctoSpace space;
    
    space.add_point(OctoDataPoint(Vector3(0, 0, 0), 1.0f));
    space.add_point(OctoDataPoint(Vector3(5, 5, 5), 0.5f));
    space.add_point(OctoDataPoint(Vector3(-5, -5, -5), 0.3f));
    
    BBox3D query(Vector3(-2, -2, -2), Vector3(6, 6, 6));
    auto results = space.query_bbox(query);
    
    return results.size() == 2;  // (0,0,0) and (5,5,5)
}

// Test 22 : OctoSpace LOD query
bool test_octospace_lod() {
    OctoSpace space;
    
    for (int i = 0; i < 100; ++i) {
        space.add_point(OctoDataPoint(
            Vector3(i * 0.2f - 10.0f, 0, 0), 1.0f
        ));
    }
    
    Camera camera;
    camera.set_position(Vector3(0, 0, 10));
    camera.set_level(2);
    
    auto lod_nodes = space.query_lod(camera);
    
    return lod_nodes.size() > 0;
}

// Test 23 : AsciiFrameBuffer creation
bool test_framebuffer_creation() {
    AsciiFrameBuffer fb(80, 40);
    
    return fb.width() == 80 && fb.height() == 40;
}

// Test 24 : AsciiFrameBuffer set/get pixel
bool test_framebuffer_pixels() {
    AsciiFrameBuffer fb(80, 40);
    
    fb.set_pixel(10, 20, '@');
    char c = fb.get_pixel(10, 20);
    
    return c == '@';
}

// Test 25 : AsciiFrameBuffer clear
bool test_framebuffer_clear() {
    AsciiFrameBuffer fb(80, 40);
    
    fb.set_pixel(10, 20, '@');
    fb.clear();
    char c = fb.get_pixel(10, 20);
    
    return c == ' ';
}

// Test 26 : AsciiFrameBuffer z-buffer
bool test_framebuffer_zbuffer() {
    AsciiFrameBuffer fb(80, 40);
    
    fb.set_pixel(10, 20, '@', 5.0f);  // Far
    fb.set_pixel(10, 20, '#', 2.0f);  // Near (should overwrite)
    
    char c = fb.get_pixel(10, 20);
    
    return c == '#';
}

// Test 27 : AsciiRenderer creation
bool test_renderer_creation() {
    AsciiRenderer renderer(80, 40);
    
    return true;  // Just check no crash
}

// Test 28 : AsciiRenderer render
bool test_renderer_render() {
    OctoSpace space;
    space.add_point(OctoDataPoint(Vector3(0, 0, 0), 1.0f));
    
    Camera camera;
    AsciiRenderer renderer(80, 40);
    
    renderer.render(space, camera);
    std::string frame = renderer.to_string();
    
    return frame.length() > 0;
}

int main() {
    std::cout << "\n=== QUANTUMLENS CORE TESTS ===\n\n";
    
    int total = 0, passed = 0;
    
    auto run_test = [&](const char* name, bool (*test_fn)()) {
        total++;
        bool result = test_fn();
        if (result) passed++;
        print_test(name, result);
    };
    
    // Vector3 tests
    run_test("Vector3 Operations", test_vector3_operations);
    run_test("Vector3 Length", test_vector3_length);
    run_test("Vector3 Dot & Cross", test_vector3_dot_cross);
    
    // BBox3D tests
    run_test("BBox3D Creation", test_bbox_creation);
    run_test("BBox3D Contains", test_bbox_contains);
    run_test("BBox3D Intersection", test_bbox_intersection);
    run_test("BBox3D Distance", test_bbox_distance);
    
    // OctreeNode3D tests
    run_test("OctreeNode3D Creation", test_octree_creation);
    run_test("OctreeNode3D Insertion", test_octree_insertion);
    run_test("OctreeNode3D Out of Bounds", test_octree_out_of_bounds);
    run_test("OctreeNode3D Subdivision", test_octree_subdivision);
    run_test("OctreeNode3D Query", test_octree_query);
    run_test("OctreeNode3D Statistics", test_octree_statistics);
    
    // Camera tests
    run_test("Camera Creation", test_camera_creation);
    run_test("Camera Movement", test_camera_movement);
    run_test("Camera Rotation", test_camera_rotation);
    run_test("Camera Zoom", test_camera_zoom);
    run_test("Camera Level", test_camera_level);
    
    // OctoSpace tests
    run_test("OctoSpace Creation", test_octospace_creation);
    run_test("OctoSpace Add Points", test_octospace_add_points);
    run_test("OctoSpace Query BBox", test_octospace_query);
    run_test("OctoSpace LOD Query", test_octospace_lod);
    
    // Renderer tests
    run_test("FrameBuffer Creation", test_framebuffer_creation);
    run_test("FrameBuffer Pixels", test_framebuffer_pixels);
    run_test("FrameBuffer Clear", test_framebuffer_clear);
    run_test("FrameBuffer Z-Buffer", test_framebuffer_zbuffer);
    run_test("Renderer Creation", test_renderer_creation);
    run_test("Renderer Render", test_renderer_render);
    
    std::cout << "\n=== RÉSULTATS ===\n";
    std::cout << "Tests passés : " << passed << "/" << total << "\n";
    std::cout << (passed == total ? "✅ ALL PASS\n" : "⚠️ SOME FAILURES\n");
    
    return (passed == total) ? 0 : 1;
}
