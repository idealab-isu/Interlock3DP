# Checks main.py outputs (i1, i2, d1, d2) for watertightness, overlap and volume conservation,
# and writes the combined final parts S1.stl / S2.stl.
# Usage: python check_outputs.py <output_dir/> <input1> <input2>
import sys, trimesh
d = sys.argv[1]
a, b = trimesh.load(sys.argv[2]), trimesh.load(sys.argv[3])
L = {k: trimesh.load(d + k + '.obj') for k in ['i1','i2','d1','d2']}
inter = trimesh.boolean.intersection([a, b])
for k, m in L.items():
    print(f"{k}: watertight={m.is_watertight} volume={m.volume:.3f} faces={len(m.faces)} bodies={len(m.split(only_watertight=False))}")
print(f"intersection vol={inter.volume:.3f}  i1+i2={L['i1'].volume+L['i2'].volume:.3f}")
ov = trimesh.boolean.intersection([L['i1'], L['i2']])
print(f"i1 & i2 overlap vol={ov.volume if len(ov.faces) else 0:.5f}")
S1 = trimesh.boolean.union([L['d1'], L['i1']]); S2 = trimesh.boolean.union([L['d2'], L['i2']])
for n, m in [('S1', S1), ('S2', S2)]:
    print(f"{n}: watertight={m.is_watertight} vol={m.volume:.3f} bodies={len(m.split(only_watertight=False))}")
    m.export(d + n + '.stl')
print(f"S1+S2 vol={S1.volume+S2.volume:.3f}  vs union(a,b) vol={trimesh.boolean.union([a,b]).volume:.3f}")
ov = trimesh.boolean.intersection([S1, S2]); print(f"S1 & S2 overlap vol={ov.volume if len(ov.faces) else 0:.5f}")
# interlock evidence: is the split surface non-planar per layer? count distinct z-levels/face normals in i1
import numpy as np
print("i1 distinct face-normal directions:", len(np.unique(np.round(L['i1'].face_normals,3),axis=0)))
