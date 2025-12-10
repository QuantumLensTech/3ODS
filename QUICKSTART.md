# 3ODS Quickstart Guide

**Get the core implementation running in 5 minutes**

---

## Prerequisites

- Python 3.10 or higher
- pip (Python package manager)

Check your Python version:
```bash
python3 --version  # Should be 3.10+
```

---

## Setup

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

For minimal setup (no testing/dev tools):
```bash
pip install pytest  # Only needed for tests
```

### 2. Verify Installation

Run the self-test demos:

```bash
# Run Octant demonstration
python3 octant.py

# Run OctoSpace demonstration  
python3 octospace.py
```

**Expected output**: Each script prints geometric properties and verifies invariants.

---

## Running Tests

### Full Test Suite

```bash
pytest test_octant.py -v
```

**Expected result**: All tests pass (30+ tests)

### Specific Test Categories

```bash
# Test only Octant class
pytest test_octant.py::TestOctant -v

# Test only OctoSpace class
pytest test_octant.py::TestOctoSpace -v

# Test geometric invariants
pytest test_octant.py::TestGeometricInvariants -v
```

### Test Coverage

```bash
pytest test_octant.py --cov=. --cov-report=html
```

Open `htmlcov/index.html` to see coverage report.

---

## Running Examples

### Example 1: 3D Maze Navigation

```bash
python3 example_maze.py
```

**What it does**:
- Creates 3D maze using octree structure
- Finds path using A* algorithm
- Benchmarks octree vs. flat array (projected)
- Visualizes 2D slice of path
- Demonstrates geometric properties

**Expected output**:
```
3ODS Example: 3D Maze Navigation with Octree
============================================================

3ODS Maze Navigation Benchmark
============================================================

Depth 2 (resolution: 4³ = 64 octants)
------------------------------------------------------------
  Total octants: 73
  Leaf octants: 64
  Path found: 8 steps
  Octree time: 2.34 ms
  Flat array estimate: 5.27 ms
  Speedup: 2.25x

[... more benchmarks and visualization ...]
```

---

## Quick API Tour

### Creating Octants

```python
from octant import Octant

# Create octant by index (0-7)
oct = Octant(index=5)  # Octant 5 = (+, -, +)

# Create from coordinate signs
oct = Octant.from_signs((+1, -1, +1))

# Get coordinate signs
signs = oct.get_signs()  # Returns (+1, -1, +1)

# Check properties
print(f"Position: {oct.position}")
print(f"Size: {oct.size}")
print(f"Level: {oct.level}")
```

### Computing Distances

```python
from octant import Octant
import math

o0 = Octant(0)  # (-, -, -)
o1 = Octant(1)  # (+, -, -)
o7 = Octant(7)  # (+, +, +)

# Euclidean distance
d_edge = o0.distance_to(o1)       # 1.0 (edge)
d_space = o0.distance_to(o7)      # √3 ≈ 1.732 (space diagonal)

# Hamming distance
h = o0.hamming_distance(o7)       # 3 (all bits differ)

# Verify geometric invariant
assert abs(d_space - math.sqrt(h)) < 1e-9  # ✓
```

### Fractal Subdivision

```python
from octant import Octant

# Create root octant
root = Octant(0, position=(0, 0, 0), size=8.0)

# Subdivide into 8 children
children = root.subdivide()

print(f"Root size: {root.size}")       # 8.0
print(f"Child size: {children[0].size}")  # 4.0
print(f"Num children: {len(children)}")   # 8

# Recursive subdivision
for child in children:
    grandchildren = child.subdivide()
    print(f"Grandchild size: {grandchildren[0].size}")  # 2.0
```

### Spatial Queries

```python
from octospace import OctoSpace

# Create octree
space = OctoSpace(depth=3)
space.subdivide_to_depth(3)

print(f"Total octants: {space.count_octants()}")  # 585
print(f"Leaf octants: {space.count_leaves()}")    # 512

# Query bounding box
bbox = ((0, 0, 0), (2, 2, 2))
octants = space.query_spatial(bbox)
print(f"Octants in bbox: {len(octants)}")

# Find neighbors
origin = space.root
neighbors = space.neighbors(origin, distance=1.0)
print(f"Edge neighbors: {len(neighbors)}")

# Point location
point = (1.5, 1.5, 1.5)
octant = space.get_octant_at_point(point)
print(f"Point is in: {octant}")
```

---

## Verifying Geometric Properties

The core strength of 3ODS is **provable geometric invariants**. Here's how to verify them:

### Invariant 1: Distance = √(Hamming)

```python
from octant import Octant, OCTANT_SIGNS
import math

# Create octants at unit cube corners
octants = [
    Octant(i, position=OCTANT_SIGNS[i], size=2.0)
    for i in range(8)
]

# Verify for all pairs
for i in range(8):
    for j in range(i+1, 8):
        hamming = octants[i].hamming_distance(octants[j])
        distance = octants[i].distance_to(octants[j])
        expected = math.sqrt(hamming)
        
        error = abs(distance - expected)
        assert error < 1e-9, f"Invariant violated! Error: {error}"

print("✓ Geometric invariant verified for all 28 pairs")
```

### Invariant 2: Volume Conservation

```python
from octant import Octant

root = Octant(0, size=4.0)
root_volume = root.size ** 3  # 64.0

children = root.subdivide()
children_volume = sum(c.size ** 3 for c in children)  # 8 × 2³ = 64.0

assert abs(root_volume - children_volume) < 1e-9
print("✓ Volume conserved under subdivision")
```

### Invariant 3: Three Distance Types Only

```python
from octant import Octant, OCTANT_SIGNS
import math

octants = [Octant(i, position=OCTANT_SIGNS[i], size=2.0) for i in range(8)]

distances = set()
for i in range(8):
    for j in range(i+1, 8):
        d = octants[i].distance_to(octants[j])
        distances.add(round(d, 6))

# Should be exactly 3 unique distances
assert len(distances) == 3
print(f"✓ Exactly 3 distance types: {sorted(distances)}")
print(f"  Edge: 1.0, Face: {math.sqrt(2):.6f}, Space: {math.sqrt(3):.6f}")
```

---

## Common Issues

### Issue: `ModuleNotFoundError: No module named 'octant'`

**Solution**: Make sure you're in the directory containing `octant.py` and `octospace.py`:

```bash
ls octant.py octospace.py  # Should list both files
python3 octant.py          # Then try running
```

### Issue: Tests fail with `ImportError`

**Solution**: Install pytest:

```bash
pip install pytest
```

### Issue: Python version too old

**Solution**: Upgrade to Python 3.10+:

```bash
# On Ubuntu/Debian
sudo apt update
sudo apt install python3.10

# On macOS (with Homebrew)
brew install python@3.10

# Then use python3.10 instead of python3
python3.10 --version
```

---

## Next Steps

Once you've verified the core works:

1. **Explore the code**:
   - Read `octant.py` - understand the fundamental geometric unit
   - Read `octospace.py` - see how octrees provide spatial structure
   - Read `test_octant.py` - comprehensive test coverage

2. **Read the documentation**:
   - `FOUNDATIONS.md` - Mathematical foundations with proofs
   - `TOPOLOGICAL_COMPUTING.md` - Connection to quantum computing
   - `README.md` - Full project overview

3. **Experiment**:
   - Modify `example_maze.py` to create different maze configurations
   - Try deeper octrees (depth 4-5) and observe performance
   - Implement your own spatial algorithms using OctoSpace

4. **Contribute**:
   - Add new examples
   - Improve documentation
   - Implement missing features (see GitHub Issues)

---

## Questions?

- **GitHub Issues**: [https://github.com/quantumlens/3ODS/issues](https://github.com/quantumlens/3ODS/issues)
- **Email**: quantumlens.research@gmail.com
- **Documentation**: See `docs/` folder for detailed specs

---

**Welcome to 3ODS - where geometry meets computation!** 🎯

*"Eight octants, twelve phases, infinite possibilities"*
