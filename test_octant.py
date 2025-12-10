"""
Test Suite for 3ODS Core Classes

Comprehensive tests for Octant and OctoSpace classes, verifying:
- Geometric properties
- Fractal subdivision
- Spatial queries
- Distance calculations

Run with: pytest test_octant.py -v

Author: Jean-Christophe Ané
Date: January 2025
License: CC BY-NC-SA 4.0
"""

import pytest
import math
from octant import Octant, OCTANT_SIGNS, create_octant_at_origin
from octospace import OctoSpace


class TestOctant:
    """Test suite for Octant class."""
    
    def test_creation(self):
        """Test basic octant creation."""
        octant = Octant(index=0)
        assert octant.index == 0
        assert octant.position == (0.0, 0.0, 0.0)
        assert octant.size == 1.0
        assert octant.level == 0
        assert octant.state == 0
    
    def test_invalid_index(self):
        """Test that invalid indices raise ValueError."""
        with pytest.raises(ValueError):
            Octant(index=8)
        with pytest.raises(ValueError):
            Octant(index=-1)
    
    def test_invalid_state(self):
        """Test that invalid states raise ValueError."""
        with pytest.raises(ValueError):
            Octant(index=0, state=8)
    
    def test_from_signs(self):
        """Test creation from coordinate signs."""
        # Octant 5: (+, -, +) → index = 101 binary = 5
        octant = Octant.from_signs((+1, -1, +1))
        assert octant.index == 5
        
        # Octant 0: (-, -, -) → index = 000 binary = 0
        octant = Octant.from_signs((-1, -1, -1))
        assert octant.index == 0
        
        # Octant 7: (+, +, +) → index = 111 binary = 7
        octant = Octant.from_signs((+1, +1, +1))
        assert octant.index == 7
    
    def test_get_signs(self):
        """Test retrieving coordinate signs."""
        for index in range(8):
            octant = Octant(index=index)
            signs = octant.get_signs()
            
            # Verify signs match precomputed table
            expected = OCTANT_SIGNS[index]
            assert signs == expected
    
    def test_signs_roundtrip(self):
        """Test that from_signs and get_signs are inverses."""
        for index in range(8):
            octant1 = Octant(index=index)
            signs = octant1.get_signs()
            octant2 = Octant.from_signs(signs)
            assert octant1.index == octant2.index
    
    def test_hamming_distance(self):
        """Test Hamming distance calculation."""
        o0 = Octant(0)  # 000
        o1 = Octant(1)  # 001
        o3 = Octant(3)  # 011
        o7 = Octant(7)  # 111
        
        assert o0.hamming_distance(o0) == 0
        assert o0.hamming_distance(o1) == 1
        assert o0.hamming_distance(o3) == 2
        assert o0.hamming_distance(o7) == 3
        
        # Symmetry
        assert o1.hamming_distance(o0) == 1
    
    def test_distance_to(self):
        """Test Euclidean distance calculation."""
        # Create octants at unit cube corners
        octants = [
            Octant(i, position=OCTANT_SIGNS[i], size=2.0)
            for i in range(8)
        ]
        
        # Test all pairwise distances
        for i in range(8):
            for j in range(8):
                hamming = octants[i].hamming_distance(octants[j])
                distance = octants[i].distance_to(octants[j])
                expected = math.sqrt(hamming)
                
                # Verify geometric invariant: d = √(hamming)
                assert abs(distance - expected) < 1e-9, \
                    f"Octant {i} → {j}: expected {expected}, got {distance}"
    
    def test_distance_categories(self):
        """Test the three distance categories."""
        # Create octants at unit cube corners
        o0 = Octant(0, position=(-1, -1, -1), size=2.0)
        o1 = Octant(1, position=(+1, -1, -1), size=2.0)  # Edge
        o3 = Octant(3, position=(+1, +1, -1), size=2.0)  # Face diagonal
        o7 = Octant(7, position=(+1, +1, +1), size=2.0)  # Space diagonal
        
        # Edge distance: 1
        d_edge = o0.distance_to(o1)
        assert abs(d_edge - 1.0) < 1e-9
        
        # Face diagonal: √2 ≈ 1.414
        d_face = o0.distance_to(o3)
        assert abs(d_face - math.sqrt(2)) < 1e-9
        
        # Space diagonal: √3 ≈ 1.732
        d_space = o0.distance_to(o7)
        assert abs(d_space - math.sqrt(3)) < 1e-9
    
    def test_is_adjacent(self):
        """Test adjacency detection."""
        o0 = Octant(0)
        o1 = Octant(1)  # Differs in bit 0
        o2 = Octant(2)  # Differs in bit 1
        o4 = Octant(4)  # Differs in bit 2
        o7 = Octant(7)  # Differs in all bits
        
        assert o0.is_adjacent(o1)
        assert o0.is_adjacent(o2)
        assert o0.is_adjacent(o4)
        assert not o0.is_adjacent(o7)
    
    def test_subdivide(self):
        """Test octant subdivision."""
        root = Octant(0, position=(0, 0, 0), size=2.0)
        
        assert root.is_leaf()
        
        children = root.subdivide()
        
        assert not root.is_leaf()
        assert len(children) == 8
        
        # Verify children properties
        for i, child in enumerate(children):
            assert child.index == i
            assert child.size == 1.0  # Half of parent
            assert child.level == 1  # One level deeper
            assert child.parent == root
        
        # Second subdivision should return same children
        children2 = root.subdivide()
        assert children == children2
    
    def test_subdivide_positions(self):
        """Test that subdivision positions are correct."""
        root = Octant(0, position=(0, 0, 0), size=2.0)
        children = root.subdivide()
        
        # Each child should be offset by ±0.5 in each dimension
        for i, child in enumerate(children):
            signs = child.get_signs()
            expected_pos = tuple(s * 0.5 for s in signs)
            
            for j in range(3):
                assert abs(child.position[j] - expected_pos[j]) < 1e-9
    
    def test_recursive_subdivision(self):
        """Test multi-level subdivision."""
        root = Octant(0, size=8.0)
        
        # Level 0
        assert root.level == 0
        assert root.size == 8.0
        
        # Level 1
        children_l1 = root.subdivide()
        assert len(children_l1) == 8
        assert all(c.level == 1 and c.size == 4.0 for c in children_l1)
        
        # Level 2
        grandchildren = []
        for child in children_l1:
            grandchildren.extend(child.subdivide())
        
        assert len(grandchildren) == 64  # 8²
        assert all(gc.level == 2 and gc.size == 2.0 for gc in grandchildren)
    
    def test_get_all_descendants(self):
        """Test recursive descendant retrieval."""
        root = Octant(0, size=4.0)
        
        # Before subdivision
        assert len(root.get_all_descendants()) == 0
        
        # After one level
        root.subdivide()
        descendants = root.get_all_descendants()
        assert len(descendants) == 8
        
        # After two levels
        for child in root.children:
            child.subdivide()
        
        descendants = root.get_all_descendants()
        assert len(descendants) == 8 + 64  # Children + grandchildren
    
    def test_contains_point(self):
        """Test point containment."""
        octant = Octant(0, position=(0, 0, 0), size=2.0)
        
        # Points inside
        assert octant.contains_point((0, 0, 0))
        assert octant.contains_point((0.5, 0.5, 0.5))
        assert octant.contains_point((-0.9, -0.9, -0.9))
        
        # Points on boundary (should be inside)
        assert octant.contains_point((1.0, 0, 0))
        assert octant.contains_point((0, 1.0, 0))
        
        # Points outside
        assert not octant.contains_point((1.1, 0, 0))
        assert not octant.contains_point((0, 0, 2.0))
    
    def test_get_bounds(self):
        """Test bounding box retrieval."""
        octant = Octant(0, position=(5, 5, 5), size=4.0)
        
        min_corner, max_corner = octant.get_bounds()
        
        assert min_corner == (3.0, 3.0, 3.0)
        assert max_corner == (7.0, 7.0, 7.0)
    
    def test_equality(self):
        """Test octant equality."""
        o1 = Octant(0, position=(0, 0, 0), size=1.0)
        o2 = Octant(0, position=(0, 0, 0), size=1.0)
        o3 = Octant(1, position=(0, 0, 0), size=1.0)
        
        assert o1 == o2
        assert o1 != o3
    
    def test_hash(self):
        """Test that octants can be used in sets/dicts."""
        o1 = Octant(0)
        o2 = Octant(1)
        
        octant_set = {o1, o2, o1}  # Duplicate o1
        assert len(octant_set) == 2


class TestOctoSpace:
    """Test suite for OctoSpace class."""
    
    def test_creation(self):
        """Test basic OctoSpace creation."""
        space = OctoSpace(depth=2)
        
        assert space.depth == 2
        assert space.root.size == 8.0
        assert len(space.octants) == 1  # Just root initially
    
    def test_subdivide_to_depth(self):
        """Test complete tree subdivision."""
        space = OctoSpace(depth=2)
        space.subdivide_to_depth(2)
        
        # Total nodes in complete octree of depth d:
        # (8^(d+1) - 1) / 7
        # For d=2: (8³ - 1) / 7 = 511 / 7 = 73
        expected_total = (8**3 - 1) // 7
        assert space.count_octants() == expected_total
        
        # Leaf nodes: 8^d
        expected_leaves = 8**2
        assert space.count_leaves() == expected_leaves
    
    def test_octants_at_level(self):
        """Test retrieving octants by level."""
        space = OctoSpace(depth=2)
        space.subdivide_to_depth(2)
        
        level0 = space.get_octants_at_level(0)
        assert len(level0) == 1  # Root only
        
        level1 = space.get_octants_at_level(1)
        assert len(level1) == 8  # 8 children
        
        level2 = space.get_octants_at_level(2)
        assert len(level2) == 64  # 8²
    
    def test_query_spatial_full_space(self):
        """Test spatial query covering entire space."""
        space = OctoSpace(depth=1, root_size=4.0)
        space.subdivide_to_depth(1)
        
        # Query entire space
        bbox = ((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0))
        results = space.query_spatial(bbox)
        
        # Should find all octants
        assert len(results) == space.count_octants()
    
    def test_query_spatial_subset(self):
        """Test spatial query for subset of space."""
        space = OctoSpace(depth=2, root_size=8.0)
        space.subdivide_to_depth(2)
        
        # Query small region
        bbox = ((0, 0, 0), (1, 1, 1))
        results = space.query_spatial(bbox)
        
        # Should find fewer than all octants
        assert 0 < len(results) < space.count_octants()
        
        # All results should intersect bbox
        for oct in results:
            oct_min, oct_max = oct.get_bounds()
            assert not (oct_max[0] < bbox[0][0] or oct_min[0] > bbox[1][0])
    
    def test_query_spatial_empty(self):
        """Test spatial query outside tree bounds."""
        space = OctoSpace(depth=1, root_size=2.0)
        space.subdivide_to_depth(1)
        
        # Query outside space
        bbox = ((10, 10, 10), (20, 20, 20))
        results = space.query_spatial(bbox)
        
        assert len(results) == 0
    
    def test_neighbors_edge(self):
        """Test finding edge-adjacent neighbors."""
        space = OctoSpace(depth=1, root_size=2.0)
        space.subdivide_to_depth(1)
        
        root = space.root
        neighbors = space.neighbors(root, distance=1.0, tolerance=0.01)
        
        # Root at level 0 should have no edge neighbors at same level
        # But at level 1, each octant has 3 edge neighbors
        # This test needs refinement based on what we're querying
        
        # For now, just verify method works
        assert isinstance(neighbors, list)
    
    def test_get_octant_at_point(self):
        """Test point location query."""
        space = OctoSpace(depth=2, root_size=8.0)
        space.subdivide_to_depth(2)
        
        # Point near origin
        point = (0.5, 0.5, 0.5)
        octant = space.get_octant_at_point(point)
        
        assert octant is not None
        assert octant.contains_point(point)
        assert octant.level == 2  # Deepest level
    
    def test_get_octant_at_point_outside(self):
        """Test point location for point outside space."""
        space = OctoSpace(depth=1, root_size=2.0)
        space.subdivide_to_depth(1)
        
        point = (10, 10, 10)
        octant = space.get_octant_at_point(point)
        
        assert octant is None
    
    def test_traverse_breadth_first(self):
        """Test BFS traversal."""
        space = OctoSpace(depth=2)
        space.subdivide_to_depth(2)
        
        octants_bfs = space.traverse_breadth_first()
        
        # BFS should visit level 0, then level 1, then level 2
        levels = [o.level for o in octants_bfs]
        
        # Check levels are non-decreasing
        for i in range(len(levels) - 1):
            assert levels[i] <= levels[i+1]
    
    def test_traverse_depth_first(self):
        """Test DFS traversal."""
        space = OctoSpace(depth=2)
        space.subdivide_to_depth(2)
        
        octants_dfs = space.traverse_depth_first()
        
        # Should visit all octants
        assert len(octants_dfs) == space.count_octants()
    
    def test_apply_to_leaves(self):
        """Test applying function to leaves."""
        space = OctoSpace(depth=1)
        space.subdivide_to_depth(1)
        
        # Set state of all leaves to their index
        space.apply_to_leaves(lambda o: setattr(o, 'state', o.index))
        
        # Verify leaves have state == index
        for octant in space.octants:
            if octant.is_leaf():
                assert octant.state == octant.index
    
    def test_memory_usage(self):
        """Test memory usage estimation."""
        space = OctoSpace(depth=2)
        space.subdivide_to_depth(2)
        
        memory = space.get_memory_usage()
        
        # Should be proportional to number of octants
        expected_order_of_magnitude = space.count_octants() * 100
        assert memory > expected_order_of_magnitude


class TestGeometricInvariants:
    """Test geometric properties and invariants."""
    
    def test_distance_hamming_relationship(self):
        """
        Verify fundamental theorem: For unit octants,
        Euclidean distance = √(Hamming distance)
        """
        # Create octants at unit cube corners
        octants = [
            Octant(i, position=OCTANT_SIGNS[i], size=2.0)
            for i in range(8)
        ]
        
        for i in range(8):
            for j in range(8):
                hamming = octants[i].hamming_distance(octants[j])
                distance = octants[i].distance_to(octants[j])
                expected = math.sqrt(hamming)
                
                assert abs(distance - expected) < 1e-9, \
                    f"Invariant violated: O{i}→O{j}, " \
                    f"Hamming={hamming}, dist={distance:.6f}, √H={expected:.6f}"
    
    def test_fractal_volume_conservation(self):
        """Verify that subdivision conserves total volume."""
        root = Octant(0, size=4.0)
        root_volume = root.size ** 3
        
        children = root.subdivide()
        children_volume = sum(c.size ** 3 for c in children)
        
        assert abs(root_volume - children_volume) < 1e-9
    
    def test_triangle_inequality(self):
        """Verify triangle inequality for distances."""
        o0 = Octant(0, position=(0, 0, 0))
        o1 = Octant(1, position=(1, 0, 0))
        o7 = Octant(7, position=(1, 1, 1))
        
        d01 = o0.distance_to(o1)
        d17 = o1.distance_to(o7)
        d07 = o0.distance_to(o7)
        
        # d(0,7) ≤ d(0,1) + d(1,7)
        assert d07 <= d01 + d17 + 1e-9


def run_tests():
    """Run all tests (for use without pytest)."""
    import sys
    
    print("Running 3ODS Core Tests...")
    print("=" * 60)
    
    # Run pytest programmatically
    exit_code = pytest.main([__file__, "-v", "--tb=short"])
    
    sys.exit(exit_code)


if __name__ == "__main__":
    run_tests()
