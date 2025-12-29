// test_physics_simulation.cpp
#include "physics_simulation_octovalent.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace ods::physics;

// Test counter
int tests_passed = 0;
int tests_failed = 0;

#define TEST(name) \
    void test_##name(); \
    void run_test_##name() { \
        try { \
            test_##name(); \
            tests_passed++; \
            std::cout << "✅ PhysicsSimulation." << #name << std::endl; \
        } catch (const std::exception& e) { \
            tests_failed++; \
            std::cout << "❌ PhysicsSimulation." << #name << " - " << e.what() << std::endl; \
        } \
    } \
    void test_##name()

#define EXPECT_TRUE(cond) \
    if (!(cond)) throw std::runtime_error("Expected true: " #cond)

#define EXPECT_FALSE(cond) \
    if (cond) throw std::runtime_error("Expected false: " #cond)

#define EXPECT_EQ(a, b) \
    if ((a) != (b)) { \
        throw std::runtime_error("Expected equal: " + std::to_string(a) + " vs " + std::to_string(b)); \
    }

#define EXPECT_FLOAT_EQ(a, b) \
    if (std::abs((a) - (b)) > 1e-5f) { \
        throw std::runtime_error("Expected float equal: " + std::to_string(a) + " vs " + std::to_string(b)); \
    }

#define EXPECT_NEAR(a, b, epsilon) \
    if (std::abs((a) - (b)) > (epsilon)) { \
        throw std::runtime_error("Expected near: " + std::to_string(a) + " vs " + std::to_string(b) + " (eps=" + std::to_string(epsilon) + ")"); \
    }

// ============================================================================
// TESTS
// ============================================================================

TEST(ParticleCreation) {
    OctoParticle p;
    EXPECT_EQ(p.octant_id, 0);
    EXPECT_FLOAT_EQ(p.mass, 1.0f);
    EXPECT_TRUE(p.active);
}

TEST(ParticlePositionEncoding) {
    OctoParticle p(0, 0.5f, 0.5f, 0.5f);
    
    float x, y, z;
    p.get_position(x, y, z);
    
    EXPECT_NEAR(x, 0.5f, 0.01f);
    EXPECT_NEAR(y, 0.5f, 0.01f);
    EXPECT_NEAR(z, 0.5f, 0.01f);
}

TEST(ParticlePositionSetting) {
    OctoParticle p;
    p.set_position(1.3f, 0.7f, 1.9f);
    
    // x=1.3→base=1, y=0.7→base=0, z=1.9→base=1
    // octant = (z<<2)|(y<<1)|x = (1<<2)|(0<<1)|1 = 4|0|1 = 5
    EXPECT_EQ(p.octant_id, 5);
    
    EXPECT_NEAR(p.offset_x, 0.3f, 0.01f);
    EXPECT_NEAR(p.offset_y, 0.7f, 0.01f);
    EXPECT_NEAR(p.offset_z, 0.9f, 0.01f);
}

TEST(ParticleDistance) {
    OctoParticle p1(0, 0.0f, 0.0f, 0.0f);  // (0,0,0)
    OctoParticle p2(7, 1.0f, 1.0f, 1.0f);  // (2,2,2)
    
    float dist = p1.distance_to(p2);
    EXPECT_NEAR(dist, std::sqrt(12.0f), 0.01f);  // √(2²+2²+2²) = √12 ≈ 3.464
}

TEST(ParticleDirection) {
    OctoParticle p1(0, 0.0f, 0.0f, 0.0f);
    OctoParticle p2(1, 0.0f, 0.0f, 0.0f);  // (1,0,0)
    
    float dx, dy, dz;
    p1.direction_to(p2, dx, dy, dz);
    
    EXPECT_NEAR(dx, 1.0f, 0.01f);
    EXPECT_NEAR(dy, 0.0f, 0.01f);
    EXPECT_NEAR(dz, 0.0f, 0.01f);
}

TEST(SimulationCreation) {
    PhysicsSimulationOctovalent sim;
    EXPECT_EQ(sim.num_particles(), 0);
}

TEST(AddParticle) {
    PhysicsSimulationOctovalent sim;
    
    OctoParticle p(0, 0.5f, 0.5f, 0.5f, 1.0f);
    size_t idx = sim.add_particle(p);
    
    EXPECT_EQ(idx, 0);
    EXPECT_EQ(sim.num_particles(), 1);
}

TEST(RemoveParticle) {
    PhysicsSimulationOctovalent sim;
    
    OctoParticle p;
    size_t idx = sim.add_particle(p);
    
    sim.remove_particle(idx);
    
    EXPECT_FALSE(sim.get_particle(idx).active);
}

TEST(TwoParticleGravity) {
    SimulationConfig config;
    config.gravitational_constant = 1.0f;
    config.dt = 0.01f;
    config.damping = 1.0f;  // Pas d'amortissement
    
    PhysicsSimulationOctovalent sim(config);
    sim.enable_force(ForceType::GRAVITY, true);
    
    // Deux particules séparées
    OctoParticle p1(0, 0.3f, 0.5f, 0.5f, 1.0f);
    OctoParticle p2(1, 0.7f, 0.5f, 0.5f, 1.0f);
    
    sim.add_particle(p1);
    sim.add_particle(p2);
    
    // Position initiale
    float x1_before, y1, z1;
    sim.get_particle(0).get_position(x1_before, y1, z1);
    
    // Simuler 10 steps
    sim.simulate(10);
    
    // Particules devraient se rapprocher (gravité attractive)
    float x1_after, y2, z2;
    sim.get_particle(0).get_position(x1_after, y2, z2);
    
    // p1 devrait avoir bougé vers la droite (+x)
    EXPECT_TRUE(x1_after > x1_before);
}

TEST(EnergyConservation) {
    SimulationConfig config;
    config.gravitational_constant = 1.0f;
    config.dt = 0.001f;  // Petit dt pour stabilité
    config.damping = 1.0f;  // Pas d'amortissement
    
    PhysicsSimulationOctovalent sim(config);
    sim.enable_force(ForceType::GRAVITY, true);
    
    // Deux particules
    OctoParticle p1(0, 0.3f, 0.5f, 0.5f, 1.0f);
    OctoParticle p2(1, 0.7f, 0.5f, 0.5f, 1.0f);
    
    sim.add_particle(p1);
    sim.add_particle(p2);
    
    float energy_initial = sim.get_total_energy();
    
    sim.simulate(100);
    
    float energy_final = sim.get_total_energy();
    
    // Énergie devrait être approximativement conservée
    // (petites erreurs numériques acceptables)
    float energy_error = std::abs(energy_final - energy_initial) / std::abs(energy_initial);
    EXPECT_TRUE(energy_error < 0.1f);  // < 10% erreur
}

TEST(OctreeConstruction) {
    PhysicsSimulationOctovalent sim;
    
    // Ajouter plusieurs particules
    for (int i = 0; i < 16; ++i) {
        OctoParticle p(i % 8, 0.5f, 0.5f, 0.5f);
        sim.add_particle(p);
    }
    
    sim.build_octree();
    
    const auto* root = sim.get_octree_root();
    EXPECT_TRUE(root != nullptr);
}

TEST(PeriodicBoundaries) {
    SimulationConfig config;
    config.periodic_boundaries = true;
    config.dt = 0.1f;
    
    PhysicsSimulationOctovalent sim(config);
    
    OctoParticle p(0, 0.1f, 0.5f, 0.5f);
    p.vx = -5.0f;  // Vélocité vers la gauche
    
    sim.add_particle(p);
    
    sim.step();
    
    // Particule devrait avoir "wraparound"
    float x, y, z;
    sim.get_particle(0).get_position(x, y, z);
    
    // Devrait être > 0 (wrap around)
    EXPECT_TRUE(x >= 0.0f && x < 2.0f);
}

TEST(ReflectiveBoundaries) {
    SimulationConfig config;
    config.periodic_boundaries = false;
    config.dt = 0.01f;
    
    PhysicsSimulationOctovalent sim(config);
    
    OctoParticle p(0, 0.01f, 0.5f, 0.5f);
    p.vx = -1.0f;  // Vers le bord gauche
    
    sim.add_particle(p);
    
    // Simuler plusieurs steps
    for (int i = 0; i < 10; ++i) {
        sim.step();
    }
    
    // Particule devrait rester dans limites
    float x, y, z;
    sim.get_particle(0).get_position(x, y, z);
    
    EXPECT_TRUE(x >= 0.0f && x < 2.0f);  // Dans les limites
}

TEST(KineticEnergy) {
    PhysicsSimulationOctovalent sim;
    
    OctoParticle p(0, 0.5f, 0.5f, 0.5f, 2.0f);
    p.vx = 1.0f;
    p.vy = 0.0f;
    p.vz = 0.0f;
    
    sim.add_particle(p);
    
    float ke = sim.get_kinetic_energy();
    
    // KE = 0.5 * m * v² = 0.5 * 2.0 * 1.0² = 1.0
    EXPECT_NEAR(ke, 1.0f, 0.01f);
}

TEST(MomentumConservation) {
    SimulationConfig config;
    config.gravitational_constant = 1.0f;
    config.dt = 0.01f;
    config.damping = 1.0f;
    
    PhysicsSimulationOctovalent sim(config);
    sim.enable_force(ForceType::GRAVITY, true);
    
    // Deux particules égales, opposées
    OctoParticle p1(0, 0.3f, 0.5f, 0.5f, 1.0f);
    p1.vx = 1.0f;
    
    OctoParticle p2(1, 0.7f, 0.5f, 0.5f, 1.0f);
    p2.vx = -1.0f;
    
    sim.add_particle(p1);
    sim.add_particle(p2);
    
    auto stats_initial = sim.compute_statistics();
    
    sim.simulate(10);
    
    auto stats_final = sim.compute_statistics();
    
    // Momentum total devrait être ~0 (conservation)
    EXPECT_NEAR(stats_final.momentum_x, stats_initial.momentum_x, 0.01f);
}

TEST(MultipleParticles) {
    PhysicsSimulationOctovalent sim;
    
    // Grille 2×2×2 = 8 particules
    for (uint8_t oct = 0; oct < 8; ++oct) {
        OctoParticle p(oct, 0.5f, 0.5f, 0.5f);
        sim.add_particle(p);
    }
    
    EXPECT_EQ(sim.num_particles(), 8);
    
    sim.simulate(10);
    
    // Toutes devraient être actives
    auto stats = sim.compute_statistics();
    EXPECT_EQ(stats.num_active_particles, 8);
}

TEST(Statistics) {
    PhysicsSimulationOctovalent sim;
    
    OctoParticle p1(0, 0.5f, 0.5f, 0.5f, 1.0f);
    OctoParticle p2(7, 0.5f, 0.5f, 0.5f, 1.0f);
    
    sim.add_particle(p1);
    sim.add_particle(p2);
    
    auto stats = sim.compute_statistics();
    
    EXPECT_EQ(stats.num_active_particles, 2);
    EXPECT_TRUE(stats.avg_distance > 0.0f);
}

TEST(MemoryUsage) {
    PhysicsSimulationOctovalent sim;
    
    for (int i = 0; i < 100; ++i) {
        OctoParticle p(i % 8, 0.5f, 0.5f, 0.5f);
        sim.add_particle(p);
    }
    
    float memory_kb = sim.get_memory_usage_kb();
    
    EXPECT_TRUE(memory_kb > 0.0f);
    EXPECT_TRUE(memory_kb < 100.0f);  // Devrait être raisonnable
}

TEST(PerformanceStep) {
    PhysicsSimulationOctovalent sim;
    
    // 50 particules
    for (int i = 0; i < 50; ++i) {
        float offset = (i % 10) * 0.1f;
        OctoParticle p(i % 8, offset, offset, offset);
        sim.add_particle(p);
    }
    
    sim.step();
    
    float time_ms = sim.get_last_step_time_ms();
    
    EXPECT_TRUE(time_ms > 0.0f);
    EXPECT_TRUE(time_ms < 100.0f);  // < 100ms pour 50 particules
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "\n╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  PHYSICS SIMULATION - TESTS UNITAIRES                        ║\n";
    std::cout << "║  N-body simulation avec octree Barnes-Hut                    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n\n";
    
    // Run all tests
    run_test_ParticleCreation();
    run_test_ParticlePositionEncoding();
    run_test_ParticlePositionSetting();
    run_test_ParticleDistance();
    run_test_ParticleDirection();
    run_test_SimulationCreation();
    run_test_AddParticle();
    run_test_RemoveParticle();
    run_test_TwoParticleGravity();
    run_test_EnergyConservation();
    run_test_OctreeConstruction();
    run_test_PeriodicBoundaries();
    run_test_ReflectiveBoundaries();
    run_test_KineticEnergy();
    run_test_MomentumConservation();
    run_test_MultipleParticles();
    run_test_Statistics();
    run_test_MemoryUsage();
    run_test_PerformanceStep();
    
    // Summary
    std::cout << "\n╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  RÉSUMÉ                                                       ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "Tests passed: " << tests_passed << "/" << (tests_passed + tests_failed) << std::endl;
    std::cout << "Tests failed: " << tests_failed << std::endl;
    
    if (tests_failed == 0) {
        std::cout << "\n✅ ALL TESTS PASSED! 🎉\n\n";
        return 0;
    } else {
        std::cout << "\n❌ SOME TESTS FAILED\n\n";
        return 1;
    }
}
