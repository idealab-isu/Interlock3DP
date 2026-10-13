# Code Verification Report

**Date:** 2026-10-08
**Paper:** A. Chambers, A. Krishnamurthy, *"Designing Interlocking Interfaces of Complex Geometries for Multimaterial 3D Printing"*, Manufacturing Letters 49 (2026) 1215–1225 (NAMRC 54) — `90-Chambers-2026-ML.pdf`
**Code verified:** `main.py`, `Utilities.py` at commit `3271338` of this repository. The check was run on commit
`a099dfc` of the original repository (github.com/BillTheBobJr/3dModel), before the code moved here; `main.py`,
`Utilities.py` and `requirements.txt` are byte-identical in both commits (same git blob ids).

## Verdict

The code implements the paper's method. Given two overlapping meshes, it produces two watertight, non-overlapping solids whose shared region is split along a per-layer A* path and filled with interlocking stamp features.

## Code ↔ paper mapping

| Paper (Algorithm 1) | Code |
|---|---|
| Ω = vol(S₁) ∩ vol(S₂), voxelize | `RemeshIntersection` → `trimesh.boolean.intersection`, `trimesh.voxel.creation.voxelize(...).fill()` |
| Boundary ∂Vₖ and padded set Ṽₖ | `FindBoundary` + `filtered_layer_with_margin_dictionary` |
| Farthest boundary pair sₖ, tₖ | Double loop over `boundary` in `RemeshIntersection` |
| A* split path Γₖ | `CreateLayerPath` (`MinHeap`, `Element` in `Utilities.py`) |
| Flood fill partition → M₁, M₂ (1 / −1 / 0 encoding) | `SplitLayer`, `RemeshVoxelizedIntersection` |
| W_same / W_swap continuity heuristic (Eq. 3–4) | `w1` / `w2` in `RemeshVoxelizedIntersection` |
| Stamps 𝓕, 𝓕̄ (flat), 𝓒, 𝓒̄ (corner) | `MakeFlatInter`, `MakeCornerInter`, `CreateBoundaryVoxel` (4-bit encoding + 16) |
| Ŝₐ = (Sₐ \ Ω) ∪ Ω̂ₐ (Eq. 5–6) | Outputs `d1.obj`/`d2.obj` = Sₐ \ Ω and `i1.obj`/`i2.obj` = Ω̂ₐ |

## Test results

Usage: `python main.py <input1> <input2> <output_dir/> <voxel_size>`

| Test | Voxel arg | Runtime | Watertight | Final parts | S1 ∩ S2 volume | S1 + S2 vs union(inputs) |
|---|---|---|---|---|---|---|
| Cubes `[0,10]³` and `[5..15, 4..14, 2..12]` | 0.4 | 0.7 s | all yes | 1 body each | 0 | 1760.000 = 1760.000 |
| same | 0.2 | 1.0 s | all yes | 1 body each | 0 | 1760.000 = 1760.000 |
| same | 0.1 | 3.2 s | all yes | 1 body each | 0 | 1760.000 = 1760.000 |
| Narrow overlap: `[0,10]³` and `[7..17, -1..11, -1..11]` | 0.2 | 1.3 s | all yes | 1 body each | 0 | 2140.000 = 2140.000 |

The halves of the overlap (`i1 + i2`) always summed exactly to the intersection volume.

**Dense signed-distance sampling** of the overlap (10 samples per voxel edge):

| Test | A only | B only | Both | Neither | Columns with z-label change | z-transitions |
|---|---|---|---|---|---|---|
| Cubes, 0.4 | 44.7% | 55.3% | 0 | 0 | 27 / 750 | 216 |
| Cubes, 0.2 | 34.4% | 65.6% | 0 | 0 | 12 / 3000 | 192 |
| Narrow, 0.2 | 12.1% | 87.9% | 0 | 0 | 48 / 3000 | 960 |

- No gaps and no doubly-claimed points.
- Label changes along z *within* a voxel layer are the stamp teeth, so interlocking features are present. A horizontal slice of the narrow-overlap case shows a periodic tooth pattern along the interface.
- On diagonal staircase paths almost every path cell is a turn, which gets a corner stamp (a simple full-height notch). Flat (T-shaped) stamps, and with them most of the z-variation, appear mainly on straight path segments.
- Note: `trimesh.Trimesh.contains` (ray test) gave false "neither" readings at stamp boundaries. Use `trimesh.proximity.signed_distance` instead.

## Issues found

1. **Dependencies don't install cleanly.** `requirements.txt` doesn't pin numpy. `trimesh==4.1.4` fails on NumPy 2.x (`AttributeError: 'numpy.ndarray' object has no attribute 'ptp'`), and NumPy 1.x doesn't run on Python 3.13. **Working setup: Python 3.12 + `numpy<2`** (1.26.4), `trimesh==4.1.4`, `rtree`, `manifold3d`, `scipy`. The original repository's checked-in `__pycache__` (cpython-312) matches; it was not carried over here.
2. **The custom stamp arguments crash.** With 8 arguments (`main.py ... flat flatAlt corner cornerAlt`), the paths are assigned as strings and never loaded with `trimesh.load`, so the run fails with `AttributeError: 'str' object has no attribute 'copy'`. Only the built-in stamps work, but the paper says stamps are supplied at runtime.
3. **No combined output.** The final Ŝ₁ = `d1 ∪ i1` and Ŝ₂ = `d2 ∪ i2` are not written; the user has to union them (`verification/check_outputs.py` does this).
4. **Where the code departs from the paper:**
   - The CLI voxel size is the stamp feature size. The actual voxel edge is `5 × voxel_size` (`voxel_size = voxel_size_inner * 5`).
   - Endpoints are chosen by maximizing `√|dx| + √|dy|`, not the grid geodesic distance described in the paper.
   - Continuity weights use a product (`m11/√m1t · m22/√m2t`), whereas Eq. 3–4 use a sum.
   - The A* heap is capped at 500 entries (`MinHeap(500)`). On overflow it prints "No room for element in heap" and silently drops candidates, and the search also stops after 10,000 iterations.
   - The paper's lexicographic (x, then y) tie-break is not implemented.
5. **`alt_main.py` is missing.** The message of commit `030440c` in the original repository says to run it, but it
   is not in that repository or this one.
6. **The cut hugs an edge on rectangular overlaps.** The farthest boundary pair are opposite corners, and the shortest A* path follows the overlap's border, so one part can get very little of the overlap (12% in the narrow test). This follows from the method rather than being a bug, but it affects how strong the joint is.

## Changes after this report (2026-10-12)

The report above describes the code as verified. These changes were made afterwards. The method is
unchanged except for the tie-break (item 3).

1. **Fixed: the output changed from run to run.** `SplitLayer` starts its flood fill from the first
   entry of `list(set(...))`, a set of strings. Python randomizes string hashing per process, so
   that order, and with it which side of the cut went to which part, varied between runs. (Two runs
   of the cube test at 0.4 gave different `i1.obj` files.) It is now `sorted(set(...))`. The tables
   above came from single runs of the old code, so a rerun gives different shares.
2. **Fixed: A* limits (issue 4).** The heap now grows as needed instead of dropping entries
   past 500, and the 10,000-iteration cap is removed. The open and closed lists are kept as one set of
   seen cells instead of being rebuilt as string lists on every step. With the old tie-break and
   determinism fix 1, the output is byte-identical to the old code on all six test cases below.
3. **Changed: lexicographic tie-break (issue 4),** as the paper describes: equal A* costs are ordered
   by x, then y. This moves the cut on four of the six test cases.
4. **Fixed: issues 1-3.** `requirements.txt` pins the working versions. The four custom stamps are
   loaded and checked: each must exist, be watertight and lie in the unit cube. They give exactly the
   built-in result when passed the built-in stamps. `S1.stl` and `S2.stl` (the final parts) are
   written.
5. **Command line:** `argparse` usage and errors (non-zero exit), the output directory is created if
   needed, and paths are joined with `os.path.join` (no trailing separator needed).
6. **Fixed: layer order.** The layer keys are strings, and `layers.sort()` in `RemeshIntersection`
   sorted them as text, so overlaps more than 10 layers tall were processed as 0, 1, 10, 11, ..., 2, ...
   and the continuity weight compared a layer with the previous one in that order instead of the
   layer below it. It is now `layers.sort(key=int)`. Overlaps of 10 layers or fewer are unaffected
   (byte-identical output). On the two taller test cases (cubes at 0.1, 17 layers; narrow overlap)
   the files change only in triangle order: the parts are geometrically identical (symmetric
   difference 0), so every number below holds before and after this fix.

Not changed, by decision (the repository stays the paper's reference): the endpoint choice, the
product continuity weight, and the edge-following cut (issue 6).

### Results with the changes

Six cases, all with `PYTHONHASHSEED` 1 and 2 (identical output for both):

| Test | Feature size | Watertight, 1 body each | S1 ∩ S2 | S1 + S2 = union(inputs) |
|---|---|---|---|---|
| Cubes | 0.4 | yes | 0 | 1760.000 = 1760.000 |
| Cubes | 0.2 | yes | 0 | 1760.000 = 1760.000 |
| Cubes | 0.1 | yes | 0 | 1760.000 = 1760.000 |
| Narrow overlap | 0.2 | yes | 0 | 2140.000 = 2140.000 |
| Plates `[0,40]²×[0,4]` and `[3..43, 2..42, 1..5]` | 0.2 | yes | 0 | 8582.000 = 8582.000 |
| Cylinders r 12, h 6, offset (9, 4, 2) | 0.2 | yes | 0 | 4530.762 = 4530.762 |

Dense sampling, as above:

| Test | A only | B only | Both | Neither | Columns with z-label change | z-transitions |
|---|---|---|---|---|---|---|
| Cubes, 0.4 | 71.8% | 28.2% | 0 | 0 | 9 / 750 | 72 |
| Cubes, 0.2 | 64.5% | 35.5% | 0 | 0 | 36 / 3000 | 576 |
| Narrow, 0.2 | 83.3% | 16.7% | 0 | 0 | 144 / 3000 | 2880 |

## Reproducing

```bash
conda create -p ./py312 python=3.12
./py312/python -m pip install -r requirements.txt

# make test inputs
python -c "import trimesh; trimesh.creation.box(bounds=[[0,0,0],[10,10,10]]).export('a.stl'); trimesh.creation.box(bounds=[[5,4,2],[15,14,12]]).export('b.stl')"

python main.py a.stl b.stl out 0.2
python verification/check_outputs.py out/ a.stl b.stl          # watertight / overlap / volume, rewrites S1.stl, S2.stl
python verification/dense_sample.py out/ 0.2 a.stl b.stl       # gap/overlap sampling + interlock z-transitions
```
