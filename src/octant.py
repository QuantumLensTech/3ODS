"""
Octant: Fundamental Geometric Unit of 3ODS

This module implements the core Octant class representing a single octant
in 3D Euclidean space. An octant is one of 8 spatial regions defined by
the signs of (x, y, z) coordinates.

Mathematical Foundation:
    An octant O_i (i ∈ [0,7]) is defined by:
    O_i = {(x,y,z) ∈ ℝ³ : sgn(x)=s_x(i), sgn(y)=s_y(i), sgn(z)=s_z(i)}
    
    where (s_x, s_y, s_z) encode i in binary as (bit_z, bit_y, bit_x)

Author: Jean-Christophe Ané
Date: January 2025
License: CC BY-NC-SA 4.0
"""

from __future__ import annotations
from typing import Tuple, List, Optional
import math


class Octant:
    """
    Represents a single octant in 3D Euclidean space.
    
    Attributes:
        index (int): Octant index in [0, 7]
        position (Tuple[float, float, float]): Center position (x, y, z)
        size (float): Edge length of octant
        level (int): Depth in octree hierarchy (0 = root)
        state (int): Computational state value in [0, 7]
        parent (Optional[Octant]): Parent octant (None for root)
        children (Optional[List[Octant]]): 8 child octants (None if not subdivided)
    
    Geometric Properties:
        - 8 octants partition ℝ³ (excluding coordinate planes)
        - Each octant defined by signs: (+,+,+), (+,+,-), etc.
        - Natural distances: 1 (edge), √2 (face diagonal), √3 (space diagonal)
    """
    
    def __init__(
        self,
        index: int,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        size: float = 1.0,
        level: int = 0,
        state: int = 0,
        parent: Optional[Octant] = None
    ):
        """
        Initialize an Octant.
        
        Args:
            index: Octant index in [0, 7]
            position: Center position (x, y, z)
            size: Edge length
            level: Depth in octree (0 = root)
            state: Computational state in [0, 7]
            parent: Parent octant (None for root)
        
        Raises:
            ValueError: If index or state not in [0, 7]
        """
        if not 0 <= index <= 7:
            raise ValueError(f"Octant index must be in [0, 7], got {index}")
        if not 0 <= state <= 7:
            raise ValueError(f"State must be in [0, 7], got {state}")
        
        self.index = index
        self.position = position
        self.size = size
        self.level = level
        self.state = state
        self.parent = parent
        self.children: Optional[List[Octant]] = None
    
    @staticmethod
    def from_signs(
        signs: Tuple[int, int, int],
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        size: float = 1.0
    ) -> Octant:
        """
        Create octant from coordinate signs.
        
        Args:
            signs: (sign_x, sign_y, sign_z) where each is +1 or -1
            position: Center position
            size: Edge length
        
        Returns:
            Octant with corresponding index
        
        Example:
            >>> octant = Octant.from_signs((+1, -1, +1))  # Octant 5
            >>> octant.index
            5
        """
        sx, sy, sz = signs
        
        # Convert signs to bits: +1 → 1, -1 → 0
        bit_x = 1 if sx > 0 else 0
        bit_y = 1 if sy > 0 else 0
        bit_z = 1 if sz > 0 else 0
        
        # Compute index: 4*bit_z + 2*bit_y + bit_x
        index = 4 * bit_z + 2 * bit_y + bit_x
        
        return Octant(index=index, position=position, size=size)
    
    def get_signs(self) -> Tuple[int, int, int]:
        """
        Get coordinate signs for this octant.
        
        Returns:
            (sign_x, sign_y, sign_z) where each is +1 or -1
        
        Example:
            >>> octant = Octant(index=5)
            >>> octant.get_signs()
            (1, -1, 1)
        """
        # Extract bits from index
        bit_x = (self.index >> 0) & 1
        bit_y = (self.index >> 1) & 1
        bit_z = (self.index >> 2) & 1
        
        # Convert bits to signs: 1 → +1, 0 → -1
        sign_x = 1 if bit_x else -1
        sign_y = 1 if bit_y else -1
        sign_z = 1 if bit_z else -1
        
        return (sign_x, sign_y, sign_z)
    
    def distance_to(self, other: Octant) -> float:
        """
        Compute Euclidean distance to another octant.
        
        This is the geometric distance between octant centers, which
        for adjacent octants satisfies:
            - Edge distance: d = 1 (differ in 1 coordinate)
            - Face diagonal: d = √2 (differ in 2 coordinates)
            - Space diagonal: d = √3 (differ in 3 coordinates)
        
        Args:
            other: Another octant
        
        Returns:
            Euclidean distance
        
        Example:
            >>> o1 = Octant(0)  # (-, -, -)
            >>> o2 = Octant(1)  # (+, -, -)
            >>> o1.distance_to(o2)  # Edge distance
            1.0
            >>> o3 = Octant(7)  # (+, +, +)
            >>> o1.distance_to(o3)  # Space diagonal
            1.732...
        """
        dx = self.position[0] - other.position[0]
        dy = self.position[1] - other.position[1]
        dz = self.position[2] - other.position[2]
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def hamming_distance(self, other: Octant) -> int:
        """
        Compute Hamming distance between octant indices.
        
        This counts how many coordinate signs differ, which relates to
        geometric distance: d_euclidean = √(hamming_distance)
        
        Args:
            other: Another octant
        
        Returns:
            Hamming distance in [0, 3]
        
        Example:
            >>> o1 = Octant(0)  # 000 binary
            >>> o2 = Octant(7)  # 111 binary
            >>> o1.hamming_distance(o2)
            3
        """
        # XOR indices and count set bits
        xor = self.index ^ other.index
        count = 0
        while xor:
            count += xor & 1
            xor >>= 1
        return count
    
    def is_adjacent(self, other: Octant, tolerance: float = 1e-9) -> bool:
        """
        Check if octants are edge-adjacent (differ in exactly 1 coordinate).
        
        Args:
            other: Another octant
            tolerance: Numerical tolerance for distance comparison
        
        Returns:
            True if octants are edge-adjacent
        
        Example:
            >>> o1 = Octant(0)
            >>> o2 = Octant(1)
            >>> o1.is_adjacent(o2)
            True
        """
        return self.hamming_distance(other) == 1
    
    def subdivide(self) -> List[Octant]:
        """
        Subdivide this octant into 8 child octants.
        
        This is the fundamental fractal operation in 3ODS. Each child
        occupies 1/8 of parent's volume, positioned at octant corners.
        
        Returns:
            List of 8 child octants
        
        Example:
            >>> root = Octant(0, position=(0, 0, 0), size=2.0)
            >>> children = root.subdivide()
            >>> len(children)
            8
            >>> children[0].size
            1.0
        """
        if self.children is not None:
            # Already subdivided
            return self.children
        
        child_size = self.size / 2.0
        half_size = child_size / 2.0
        
        self.children = []
        
        for i in range(8):
            # Extract bits from child index
            bit_x = (i >> 0) & 1
            bit_y = (i >> 1) & 1
            bit_z = (i >> 2) & 1
            
            # Compute child position (offset from parent center)
            offset_x = half_size if bit_x else -half_size
            offset_y = half_size if bit_y else -half_size
            offset_z = half_size if bit_z else -half_size
            
            child_pos = (
                self.position[0] + offset_x,
                self.position[1] + offset_y,
                self.position[2] + offset_z
            )
            
            child = Octant(
                index=i,
                position=child_pos,
                size=child_size,
                level=self.level + 1,
                state=0,
                parent=self
            )
            
            self.children.append(child)
        
        return self.children
    
    def is_leaf(self) -> bool:
        """Check if this octant is a leaf (not subdivided)."""
        return self.children is None
    
    def get_all_descendants(self) -> List[Octant]:
        """
        Get all descendant octants (recursive).
        
        Returns:
            List of all descendants (children, grandchildren, etc.)
        """
        if self.is_leaf():
            return []
        
        descendants = list(self.children)
        for child in self.children:
            descendants.extend(child.get_all_descendants())
        
        return descendants
    
    def contains_point(self, point: Tuple[float, float, float]) -> bool:
        """
        Check if a point lies within this octant.
        
        Args:
            point: (x, y, z) coordinates
        
        Returns:
            True if point is inside octant bounds
        """
        px, py, pz = point
        cx, cy, cz = self.position
        half = self.size / 2.0
        
        return (
            abs(px - cx) <= half and
            abs(py - cy) <= half and
            abs(pz - cz) <= half
        )
    
    def get_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Get bounding box of this octant.
        
        Returns:
            ((min_x, min_y, min_z), (max_x, max_y, max_z))
        """
        cx, cy, cz = self.position
        half = self.size / 2.0
        
        min_corner = (cx - half, cy - half, cz - half)
        max_corner = (cx + half, cy + half, cz + half)
        
        return (min_corner, max_corner)
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        signs = self.get_signs()
        sign_str = "".join("+" if s > 0 else "-" for s in signs)
        return (
            f"Octant(index={self.index}, signs={sign_str}, "
            f"pos={self.position}, level={self.level}, state={self.state})"
        )
    
    def __eq__(self, other: object) -> bool:
        """Equality based on index and position."""
        if not isinstance(other, Octant):
            return NotImplemented
        return (
            self.index == other.index and
            self.position == other.position and
            abs(self.size - other.size) < 1e-9
        )
    
    def __hash__(self) -> int:
        """Hash for use in sets/dicts."""
        return hash((self.index, self.position, self.level))


# Octant index to signs mapping (precomputed for efficiency)
OCTANT_SIGNS = {
    0: (-1, -1, -1),  # (---)
    1: (+1, -1, -1),  # (+--) 
    2: (-1, +1, -1),  # (-+-)
    3: (+1, +1, -1),  # (++-)
    4: (-1, -1, +1),  # (--+)
    5: (+1, -1, +1),  # (+-+)
    6: (-1, +1, +1),  # (-++)
    7: (+1, +1, +1),  # (+++)
}


def create_octant_at_origin(index: int, size: float = 1.0) -> Octant:
    """
    Convenience function to create an octant at origin.
    
    Args:
        index: Octant index [0, 7]
        size: Edge length
    
    Returns:
        Octant centered at origin
    """
    return Octant(index=index, position=(0.0, 0.0, 0.0), size=size)


if __name__ == "__main__":
    # Demonstration
    print("3ODS Octant Demonstration")
    print("=" * 50)
    
    # Create octants
    print("\n1. Creating octants:")
    o0 = Octant(0, position=(-0.5, -0.5, -0.5), size=1.0)  # (---)
    o7 = Octant(7, position=(+0.5, +0.5, +0.5), size=1.0)  # (+++)
    print(f"  {o0}")
    print(f"  {o7}")
    
    # Distance calculations
    print("\n2. Distance calculations:")
    o1 = Octant(1, position=(+0.5, -0.5, -0.5), size=1.0)  # Edge from o0
    print(f"  Distance O0 → O1 (edge): {o0.distance_to(o1):.3f} (expect 1.0)")
    o3 = Octant(3, position=(+0.5, +0.5, -0.5), size=1.0)  # Face diagonal
    print(f"  Distance O0 → O3 (face): {o0.distance_to(o3):.3f} (expect √2 ≈ 1.414)")
    o7_positioned = Octant(7, position=(+0.5, +0.5, +0.5), size=1.0)  # Space diagonal
    print(f"  Distance O0 → O7 (space): {o0.distance_to(o7_positioned):.3f} (expect √3 ≈ 1.732)")
    
    # Hamming distances
    print("\n3. Hamming distances:")
    print(f"  Hamming(O0, O1): {o0.hamming_distance(o1)} (differ in 1 bit)")
    print(f"  Hamming(O0, O3): {o0.hamming_distance(o3)} (differ in 2 bits)")
    print(f"  Hamming(O0, O7): {o0.hamming_distance(o7)} (differ in 3 bits)")
    
    # Subdivision (fractal)
    print("\n4. Fractal subdivision:")
    root = Octant(0, position=(0, 0, 0), size=2.0)
    print(f"  Root: size={root.size}, level={root.level}")
    children = root.subdivide()
    print(f"  Children: {len(children)} octants")
    print(f"  Child[0]: size={children[0].size}, level={children[0].level}")
    
    # Verify geometric property: √(Hamming) = distance for unit octants
    print("\n5. Verifying geometric invariant:")
    unit_octants = [
        Octant(i, position=tuple(s * 0.5 for s in OCTANT_SIGNS[i]), size=1.0) 
        for i in range(8)
    ]
    for i in range(8):
        for j in range(i+1, 8):
            hamming = unit_octants[i].hamming_distance(unit_octants[j])
            distance = unit_octants[i].distance_to(unit_octants[j])
            expected = math.sqrt(hamming)
            error = abs(distance - expected)
            print(f"  O{i} → O{j}: Hamming={hamming}, dist={distance:.3f}, "
                  f"√H={expected:.3f}, error={error:.6f}")
