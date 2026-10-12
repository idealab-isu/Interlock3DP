# Dense signed-distance sampling of the overlap region of two input meshes.
# Usage: python dense_sample.py <output_dir/> <voxel_size> <input1> <input2>
# Requires S1.stl / S2.stl in output_dir (produced by check_outputs.py).
import sys, trimesh, numpy as np

d = sys.argv[1]
h = float(sys.argv[2]) * 5          # actual voxel edge = 5x the CLI voxel size
m1, m2 = trimesh.load(sys.argv[3]), trimesh.load(sys.argv[4])
a = trimesh.load(d + 'S1.stl'); b = trimesh.load(d + 'S2.stl')

lo, hi = trimesh.boolean.intersection([m1, m2]).bounds
step = h / 10
xs, ys, zs = [np.arange(lo[k] + step / 2, hi[k], step) for k in range(3)]
G = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3)
shape = (len(xs), len(ys), len(zs))
sa = (trimesh.proximity.signed_distance(a, G) > 0).reshape(shape)
sb = (trimesh.proximity.signed_distance(b, G) > 0).reshape(shape)

print("samples", sa.size,
      "| A only", (sa & ~sb).mean().round(4), "B only", (sb & ~sa).mean().round(4),
      "both", (sa & sb).mean().round(4), "neither", (~sa & ~sb).mean().round(4))

# Label changes along z inside a layer indicate interlocking stamp teeth
chg = np.diff(sa.astype(int), axis=2) != 0
print("columns whose label changes along z:", chg.any(axis=2).sum(), "of", chg.shape[0] * chg.shape[1])
print("total z label transitions:", chg.sum())
