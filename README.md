# Interlock3DP

Code to create interlocked geometries for multimaterial 3D printing.

Given two solids that overlap, Interlock3DP splits the shared region between them so that the two
printed parts lock together. The result is two watertight, non-overlapping solids, ready to be printed
in two materials.

This is the reference implementation for:

> A. Chambers, A. Krishnamurthy, *"Designing Interlocking Interfaces of Complex Geometries for
> Multimaterial 3D Printing"*, Manufacturing Letters 49 (2026) 1215–1225 (NAMRC 54).

## How it works

1. **Overlap.** The intersection Ω of the two input solids is computed and voxelized.
2. **Cut each layer.** In every voxel layer, an A* path between the two farthest boundary points
   divides the layer's overlap. A flood fill gives one side to each part, and a continuity weight keeps
   the sides consistent from one layer to the next.
3. **Stamp the interface.** Each cell on the path is replaced by an interlocking stamp: a flat
   (T-shaped) stamp on straight runs and a corner stamp at turns. Each stamp has a complementary
   half, so the two parts' teeth fit together within the layer.
4. **Assemble.** Each part is its input solid minus the overlap, plus its share of the overlap.

`VERIFICATION.md` maps each step of the paper's Algorithm 1 to the function that implements it.

## Setup

Use **Python 3.12** with **NumPy 1.x**. `trimesh` 4.1.4 does not work with NumPy 2, and NumPy 1.x
does not run on Python 3.13.

```bash
conda create -p ./py312 python=3.12
./py312/python -m pip install "numpy<2" "trimesh==4.1.4" rtree manifold3d scipy
```

## Usage

```bash
python main.py <input1> <input2> <output_dir/> <feature_size>
```

- `input1`, `input2`: the two solids, in any mesh format trimesh reads (STL, OBJ, ...). They must
  overlap.
- `output_dir/`: must already exist and must end with a path separator, because the output names are
  appended to it directly.
- `feature_size`: the stamp feature size, in model units; pick it near your nozzle diameter. One voxel
  is 5 × `feature_size`, and each stamp is a 5 × 5 × 5 pattern of features.

### Output

| File | Contents |
|---|---|
| `d1.obj`, `d2.obj` | Input 1 and input 2 with the overlap removed |
| `i1.obj`, `i2.obj` | The two interlocking halves of the overlap |

The final parts are `d1 ∪ i1` and `d2 ∪ i2`; `main.py` does not combine them. The
`verification/check_outputs.py` script does: it writes them as `S1.stl` and `S2.stl`.

### Example

```bash
python -c "import trimesh; trimesh.creation.box(bounds=[[0,0,0],[10,10,10]]).export('a.stl'); trimesh.creation.box(bounds=[[5,4,2],[15,14,12]]).export('b.stl')"
mkdir out
python main.py a.stl b.stl out/ 0.2
python verification/check_outputs.py out/ a.stl b.stl
```

This takes about a second. Each final part is one watertight body, and the two parts do not overlap.

## Verification

`VERIFICATION.md` compares the code with the paper and records the test runs. The two scripts in
`verification/` reproduce them:

| Script | Checks |
|---|---|
| `check_outputs.py <out/> <a> <b>` | Watertightness, number of bodies, overlap between the parts, volume conservation; writes `S1.stl` and `S2.stl` |
| `dense_sample.py <out/> <feature_size> <a> <b>` | Samples the overlap densely to look for gaps and doubly claimed points, and counts the stamp teeth (label changes along z); run `check_outputs.py` first |

## Known limitations

These are described in detail in `VERIFICATION.md`.

- **`requirements.txt` does not pin NumPy;** install `numpy<2` as shown above.
- **The custom stamp arguments do not work.** `main.py` accepts four stamp meshes after the required
  arguments, but they are never loaded, and the run fails. Only the built-in stamps work.
- **The final parts are not written** (see Output).
- **Departures from the paper:**
  - the farthest boundary pair maximizes √|dx| + √|dy| rather than the grid distance;
  - the continuity weights are a product rather than the sum in Eq. 3–4;
  - the A* heap holds at most 500 entries, and the search stops after 10,000 iterations;
  - the lexicographic tie-break is not implemented.
- **On rectangular overlaps the cut follows an edge,** so one part can get little of the overlap (12 %
  in the narrow-overlap test).

## C++ implementation

GPView 2.2 ([idealab-isu/GPView2.0](https://github.com/idealab-isu/GPView2.0)) has a C++ implementation
of the method, `gpview::interlock`. It has:

- an `interlock_export` command-line tool and an Interlock tab in the viewer;
- a mode that reproduces this code's cut and stamps (`--paper`);
- other interface patterns (gyroid by default) and a stepped diagonal cut.

## License

MIT; see `LICENSE`.
