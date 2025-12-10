"""
OctoSpace: Octree Data Structure for 3ODS

This module implements OctoSpace, a hierarchical octree structure for
efficient spatial queries on octovalent data. An octree recursively
subdivides 3D space into 8 octants at each level.

Mathematical Foundation:
    An octree T(d) of depth d has:
    - Total nodes: (8^(d+1) - 1) / 7
    - Leaf nodes: 8^d
    - Each node: 8 children (unless leaf)

Author: Jean-Christophe Ané
Date: January 2025
License: CC BY-NC-SA 4.0
"""

from __future__ import annotations
from typing import List, Tuple, Optional, Callable
from octant import Octant
import math


class OctoSpace:
    """
    Hierarchical octree structure for spatial queries.
    
    Attributes:
        root (Octant): Root octant spanning entire space
        depth (int): Maximum depth of octree
        octants (List[Octant]): All octants in tree (flat list)
    
    Operations:
        - insert(octant): Add octant to space
        - query_spatial(bbox): Find octants in bounding box
        - neighbors(octant, distance): Find nearby octants
        - subdivide_to_depth(d): Subdivide entire tree to depth d
    """
    
    def __init__(
        self,
        depth: int = 3,
        root_position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        root_size: float = 8.0
    ):
        """
        Initialize OctoSpace.
        
        Args:
            depth: Maximum tree depth (0 = single octant)
            root_position: Center of root octant
            root_size: Edge length of root octant
        
        Example:
            >>> space = OctoSpace(depth=3)  # 8³ = 512 potential octants
            >>> space.root.size
            8.0
        """
        if depth < 0:
            raise ValueError(f"Depth must be non-negative, got {depth}")
        
        self.depth = depth
        self.root = Octant(
            index=0,
            position=root_position,
            size=root_size,
            level=0
        )
        self.octants: List[Octant] = [self.root]
    
    def insert(self, octant: Octant) -> bool:
        """
        Insert an octant into the tree.
        
        This is a placeholder for dynamic insertion. Current implementation
        uses pre-subdivision via subdivide_to_depth().
        
        Args:
            octant: Octant to insert
        
        Returns:
            True if inserted successfully
        """
        # Find appropriate parent and insert
        # For now, just add to list
        if octant not in self.octants:
            self.octants.append(octant)
            return True
        return False
    
    def subdivide_to_depth(self, target_depth: int) -> None:
        """
        Subdivide entire tree to target depth.
        
        This creates a complete octree where all leaves are at target_depth.
        Total octants created: (8^(target_depth+1) - 1) / 7
        
        Args:
            target_depth: Depth to subdivide to
        
        Example:
            >>> space = OctoSpace(depth=2)
            >>> space.subdivide_to_depth(2)
            >>> len(space.octants)
            73  # (8³ - 1)/7 + 8² + 8 + 1 = 73
        """
        if target_depth > self.depth:
            self.depth = target_depth
        
        self._subdivide_recursive(self.root, target_depth)
        
        # Rebuild flat octant list
        self.octants = [self.root]
        self.octants.extend(self.root.get_all_descendants())
    
    def _subdivide_recursive(self, octant: Octant, target_depth: int) -> None:
        """Recursive helper for subdivide_to_depth."""
        if octant.level < target_depth:
            children = octant.subdivide()
            for child in children:
                self._subdivide_recursive(child, target_depth)
    
    def query_spatial(
        self,
        bbox: Tuple[Tuple[float, float, float], Tuple[float, float, float]]
    ) -> List[Octant]:
        """
        Find all octants intersecting a bounding box.
        
        This is a fundamental spatial query operation. Uses recursive
        descent with pruning for efficiency.
        
        Args:
            bbox: ((min_x, min_y, min_z), (max_x, max_y, max_z))
        
        Returns:
            List of octants intersecting bbox
        
        Complexity:
            O(8^d_bbox) where d_bbox is depth of smallest octant
            fully containing bbox.
        
        Example:
            >>> space = OctoSpace(depth=2)
            >>> space.subdivide_to_depth(2)
            >>> bbox = ((0, 0, 0), (2, 2, 2))
            >>> octants = space.query_spatial(bbox)
            >>> len(octants) > 0
            True
        """
        min_corner, max_corner = bbox
        results: List[Octant] = []
        
        self._query_spatial_recursive(self.root, min_corner, max_corner, results)
        
        return results
    
    def _query_spatial_recursive(
        self,
        octant: Octant,
        bbox_min: Tuple[float, float, float],
        bbox_max: Tuple[float, float, float],
        results: List[Octant]
    ) -> None:
        """Recursive helper for query_spatial."""
        # Get octant bounds
        oct_min, oct_max = octant.get_bounds()
        
        # Check if octant intersects bbox
        if not self._boxes_intersect(oct_min, oct_max, bbox_min, bbox_max):
            return  # Prune this branch
        
        # If octant intersects, add it
        results.append(octant)
        
        # Recurse on children (if any)
        if not octant.is_leaf():
            for child in octant.children:
                self._query_spatial_recursive(child, bbox_min, bbox_max, results)
    
    @staticmethod
    def _boxes_intersect(
        box1_min: Tuple[float, float, float],
        box1_max: Tuple[float, float, float],
        box2_min: Tuple[float, float, float],
        box2_max: Tuple[float, float, float]
    ) -> bool:
        """Check if two axis-aligned boxes intersect."""
        return (
            box1_min[0] <= box2_max[0] and box1_max[0] >= box2_min[0] and
            box1_min[1] <= box2_max[1] and box1_max[1] >= box2_min[1] and
            box1_min[2] <= box2_max[2] and box1_max[2] >= box2_min[2]
        )
    
    def neighbors(
        self,
        octant: Octant,
        distance: float,
        tolerance: float = 1e-6
    ) -> List[Octant]:
        """
        Find all octants within a given distance.
        
        This implements geometric neighbor search with exact distance
        guarantees (not approximations).
        
        Args:
            octant: Query octant
            distance: Maximum distance (Euclidean)
            tolerance: Numerical tolerance for distance comparison
        
        Returns:
            List of octants within distance (excluding query octant itself)
        
        Example:
            >>> space = OctoSpace(depth=1)
            >>> space.subdivide_to_depth(1)
            >>> origin = space.root
            >>> neighbors = space.neighbors(origin, distance=1.0)
            >>> len(neighbors)  # Edge-adjacent octants
            3
        """
        results: List[Octant] = []
        
        for oct in self.octants:
            if oct != octant:
                d = octant.distance_to(oct)
                if d <= distance + tolerance:
                    results.append(oct)
        
        return results
    
    def get_octant_at_point(
        self,
        point: Tuple[float, float, float],
        max_level: Optional[int] = None
    ) -> Optional[Octant]:
        """
        Find the octant containing a point.
        
        Returns the deepest (smallest) octant containing the point,
        up to max_level if specified.
        
        Args:
            point: (x, y, z) coordinates
            max_level: Maximum depth to search (None = use tree depth)
        
        Returns:
            Octant containing point, or None if point outside root
        
        Example:
            >>> space = OctoSpace(depth=2)
            >>> space.subdivide_to_depth(2)
            >>> octant = space.get_octant_at_point((1.0, 1.0, 1.0))
            >>> octant is not None
            True
        """
        if max_level is None:
            max_level = self.depth
        
        if not self.root.contains_point(point):
            return None
        
        return self._get_octant_at_point_recursive(self.root, point, max_level)
    
    def _get_octant_at_point_recursive(
        self,
        octant: Octant,
        point: Tuple[float, float, float],
        max_level: int
    ) -> Octant:
        """Recursive helper for get_octant_at_point."""
        if octant.is_leaf() or octant.level >= max_level:
            return octant
        
        # Find which child contains point
        for child in octant.children:
            if child.contains_point(point):
                return self._get_octant_at_point_recursive(child, point, max_level)
        
        # Fallback (shouldn't happen if point truly inside octant)
        return octant
    
    def count_octants(self) -> int:
        """Count total number of octants in tree."""
        return len(self.octants)
    
    def count_leaves(self) -> int:
        """Count number of leaf octants."""
        return sum(1 for oct in self.octants if oct.is_leaf())
    
    def get_octants_at_level(self, level: int) -> List[Octant]:
        """
        Get all octants at a specific tree level.
        
        Args:
            level: Tree level (0 = root)
        
        Returns:
            List of octants at that level
        """
        return [oct for oct in self.octants if oct.level == level]
    
    def traverse_breadth_first(self) -> List[Octant]:
        """
        Traverse octree in breadth-first order.
        
        Returns:
            Octants in BFS order (level 0, level 1, ...)
        """
        result = []
        queue = [self.root]
        
        while queue:
            octant = queue.pop(0)
            result.append(octant)
            
            if not octant.is_leaf():
                queue.extend(octant.children)
        
        return result
    
    def traverse_depth_first(self) -> List[Octant]:
        """
        Traverse octree in depth-first order.
        
        Returns:
            Octants in DFS order
        """
        result = []
        self._traverse_dfs_recursive(self.root, result)
        return result
    
    def _traverse_dfs_recursive(self, octant: Octant, result: List[Octant]) -> None:
        """Recursive helper for DFS traversal."""
        result.append(octant)
        
        if not octant.is_leaf():
            for child in octant.children:
                self._traverse_dfs_recursive(child, result)
    
    def apply_to_leaves(self, func: Callable[[Octant], None]) -> None:
        """
        Apply a function to all leaf octants.
        
        Args:
            func: Function to apply (takes Octant, returns None)
        
        Example:
            >>> space = OctoSpace(depth=1)
            >>> space.subdivide_to_depth(1)
            >>> space.apply_to_leaves(lambda o: setattr(o, 'state', o.index))
        """
        for octant in self.octants:
            if octant.is_leaf():
                func(octant)
    
    def get_memory_usage(self) -> int:
        """
        Estimate memory usage in bytes.
        
        Returns:
            Approximate memory in bytes
        """
        # Rough estimate: ~200 bytes per Octant object
        return len(self.octants) * 200
    
    def __repr__(self) -> str:
        """String representation."""
        return (
            f"OctoSpace(depth={self.depth}, "
            f"octants={len(self.octants)}, "
            f"leaves={self.count_leaves()})"
        )


if __name__ == "__main__":
    print("3ODS OctoSpace Demonstration")
    print("=" * 50)
    
    # Create octree
    print("\n1. Creating octree:")
    space = OctoSpace(depth=2, root_size=8.0)
    print(f"  {space}")
    
    # Subdivide to depth 2
    print("\n2. Subdividing to depth 2:")
    space.subdivide_to_depth(2)
    print(f"  {space}")
    print(f"  Total octants: {space.count_octants()}")
    print(f"  Leaf octants: {space.count_leaves()}")
    print(f"  Memory usage: {space.get_memory_usage()} bytes")
    
    # Spatial query
    print("\n3. Spatial query:")
    bbox = ((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0))
    results = space.query_spatial(bbox)
    print(f"  Bounding box: {bbox}")
    print(f"  Octants found: {len(results)}")
    
    # Neighbor search
    print("\n4. Neighbor search:")
    origin = space.root
    
    # Edge neighbors (distance 1)
    neighbors_edge = space.neighbors(origin, distance=1.0)
    print(f"  Edge neighbors (d=1): {len(neighbors_edge)}")
    
    # Face diagonal neighbors (distance √2 ≈ 1.414)
    neighbors_face = space.neighbors(origin, distance=1.5)
    print(f"  Up to face diagonal (d=√2): {len(neighbors_face)}")
    
    # Space diagonal neighbors (distance √3 ≈ 1.732)
    neighbors_space = space.neighbors(origin, distance=2.0)
    print(f"  Up to space diagonal (d=√3): {len(neighbors_space)}")
    
    # Point location
    print("\n5. Point location:")
    point = (1.5, 1.5, 1.5)
    octant = space.get_octant_at_point(point)
    if octant:
        print(f"  Point {point} is in:")
        print(f"    {octant}")
    
    # Traversal
    print("\n6. Tree traversal:")
    bfs = space.traverse_breadth_first()
    print(f"  BFS order: {len(bfs)} octants")
    print(f"  First 5: {[f'L{o.level}' for o in bfs[:5]]}")
    
    dfs = space.traverse_depth_first()
    print(f"  DFS order: {len(dfs)} octants")
    print(f"  First 5: {[f'L{o.level}' for o in dfs[:5]]}")
    
    # Geometric verification
    print("\n7. Verifying geometric guarantees:")
    space_small = OctoSpace(depth=1, root_size=2.0)
    space_small.subdivide_to_depth(1)
    
    # For depth=1, we have 1 root + 8 children = 9 octants
    # Children should be at positions corresponding to octant signs
    children = space_small.get_octants_at_level(1)
    print(f"  Children at level 1: {len(children)}")
    
    for i, child in enumerate(children):
        signs = child.get_signs()
        expected_pos = tuple(s * 0.5 for s in signs)  # Half of child size
        actual_pos = child.position
        error = math.sqrt(sum((a-e)**2 for a, e in zip(actual_pos, expected_pos)))
        print(f"    Child {i}: signs={signs}, pos={actual_pos}, "
              f"expected={expected_pos}, error={error:.6f}")
