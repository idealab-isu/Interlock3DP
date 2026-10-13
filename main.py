import numpy as np
import trimesh
import re
from collections import defaultdict
from copy import deepcopy
import sys
import os
import argparse
import Utilities

flat = trimesh.Trimesh()
flatAlt = trimesh.Trimesh()

corner = trimesh.Trimesh()
cornerAlt = trimesh.Trimesh()

def MakeFlatInter():
    base = trimesh.boolean.union(
        [
         trimesh.creation.box(bounds=[[2,0,0],[3,1,5]]),
         trimesh.creation.box(bounds=[[2,4,2],[3,5,3]]),
         trimesh.creation.box(bounds=[[2,3,0],[3,4,2]]),
         trimesh.creation.box(bounds=[[2,3,3],[3,4,5]]),
         trimesh.creation.box(bounds=[[1,1,2],[3,4,3]]),
         trimesh.creation.box(bounds=[[0,0,0],[1,5,5]]),
        ]
    )

    baseAlt = trimesh.boolean.difference([trimesh.creation.box(bounds=[[0,0,0],[5,5,5]]), base])

    base.apply_transform(trimesh.transformations.scale_matrix(1/5))
    baseAlt.apply_transform(trimesh.transformations.scale_matrix(1/5))

    global flat 
    global flatAlt
    flat = base.copy()
    flatAlt = baseAlt.copy()

def MakeCornerInter():
    base = trimesh.creation.box(bounds=[[4,4,0],[5,5,5]])
    baseAlt = trimesh.boolean.difference([trimesh.creation.box(bounds=[[0,0,0],[5,5,5]]), base])

    base.apply_transform(trimesh.transformations.scale_matrix(1/5))
    baseAlt.apply_transform(trimesh.transformations.scale_matrix(1/5))

    global corner 
    global cornerAlt
    corner = base.copy()
    cornerAlt = baseAlt.copy()

def CreateBox(x, y, z, origin, voxelSize):
    box = trimesh.creation.box(bounds=[origin + [(x) * voxelSize, y* voxelSize, z * voxelSize],  origin  + [(x + 1) * voxelSize, (y + 1) * voxelSize, (z + 1) * voxelSize]])
    return box

def CreateBoundaryVoxel(x, y, z, origin, voxelSize, encoding):
    orientation = int(encoding & 3)
    voxel = trimesh.Trimesh()

    match (encoding >> 2) & 3:
        case 0:
            global flat
            voxel = flat.copy()
        case 1:
            global flatAlt
            voxel = flatAlt.copy()
        case 2:
            global corner
            voxel = corner.copy()
        case 3:
            global cornerAlt
            voxel = cornerAlt.copy()

    voxel.apply_transform(trimesh.transformations.rotation_matrix(orientation*np.pi / 2, [0, 0, 1], [.5, .5, .5]))
    voxel.apply_translation([x, y, z])
    voxel.apply_transform(trimesh.transformations.scale_matrix(voxelSize))
    voxel.apply_translation(origin)

    return voxel

def FindValidNewElements(element, end, layer):
    x, y = layer.shape
    try:
        return list(filter(lambda s: layer[s.x][s.y], element.createSuccessors(x, y, end) ))
    except:
        exit()

def CreateLayerPath(voxelized_layer, coords):
    # A* from start to end. The heap has no size limit and the search runs until the end is
    # reached or the layer is exhausted. A cell is added once: cells already open or closed
    # are not added again.
    end = Utilities.Element(coords[1][0], coords[1][1])
    start = Utilities.Element(coords[0][0], coords[0][1], end)
    heap = Utilities.MinHeap()
    heap.insert(start)
    seen = {(start.x, start.y)}
    current = start
    while(heap.size > 0):
        current = heap.remove()
        if(current == end):
            break
        for element in FindValidNewElements(current, end, voxelized_layer):
            if (element.x, element.y) not in seen:
                seen.add((element.x, element.y))
                heap.insert(element)
    return current

def SplitLayer(layer, layer_indices_with_margin):
    def FlatToAct(flat_ele):
        m = re.search(r"\((-?\d+), (-?\d+), (\d+)\)" , flat_ele)
        return np.array([int(m.group(1)),int(m.group(2)),int(m.group(3))])

    size_x = layer.shape[0]
    size_y = layer.shape[1]
    to_clear = [layer_indices_with_margin[0]]
    cleared = []

    while(len(to_clear) > 0):
        curr = to_clear.pop()

        if(not(curr in layer_indices_with_margin) or (curr in cleared)):
            continue
        layer_indices_with_margin.remove(curr)
        cleared.append(curr)
        act = FlatToAct(curr)

        for x in [-1,0,1]:
            for y in [-1,0,1]:
                to_clear.append(Utilities.Flatten3DElement(act + [x,y,0]))

    final = list(filter(lambda ele: ele[0] != -1 and ele[1] != -1 and ele[0] < size_x and ele[1] < size_y,list(map(FlatToAct, cleared))))

    for ele in final:
        try:
            layer[ele[0]][ele[1]] = 0
        except:
            continue
        
    return

def RemeshVoxelizedIntersection(path, voxelized_mesh_layer, layer_indices, layer_indices_with_margin, z, previous_layer):
    mesh_half_layer_1 = 1*deepcopy(voxelized_mesh_layer)
    currentElement = path
    pathElements = [[currentElement.x, currentElement.y]]
    mesh_half_layer_1[currentElement.x][currentElement.y] = -1
    
    for x in [-1,0,1]:
        for y in [-1,0,1]:
            ele = Utilities.Flatten3DElement([currentElement.x + x, currentElement.y + y, z])
            if(x == 0 and y == 0 or not(ele in layer_indices)):
                layer_indices_with_margin.remove(ele)

    while(currentElement.parent is not None):
        currentElement = currentElement.parent
        pathElements.append([currentElement.x, currentElement.y])
        mesh_half_layer_1[currentElement.x][currentElement.y] = -1
        layer_indices_with_margin.remove(Utilities.Flatten3DElement([currentElement.x, currentElement.y, z]))

    for x in [-1,0,1]:
        for y in [-1,0,1]:
            ele = Utilities.Flatten3DElement([currentElement.x + x, currentElement.y + y, z])
            if(not(ele in layer_indices) and ele in layer_indices_with_margin):
                layer_indices_with_margin.remove(ele)

    mesh_half_layer_2 = deepcopy(mesh_half_layer_1)

    SplitLayer(mesh_half_layer_1, layer_indices_with_margin)

    def MeshChangeDefinition(element):
        return -1*(element == 1)
    
    for x in range(mesh_half_layer_1.shape[0]):
        mesh_half_layer_2[x] = np.add(mesh_half_layer_2[x], list(map(MeshChangeDefinition, mesh_half_layer_1[x])))
    

    changeDict = defaultdict(list)
    
    m11, m12, m1t, m21, m22, m2t, w1, w2 = 0, 0, 0, 0, 0, 0, 0, 0

    if(not(previous_layer is None)):
        for x in range(mesh_half_layer_1.shape[0]):
            def MapFunction(element):
                return 1*(element != 0)
            
            m1 = list(map(MapFunction, mesh_half_layer_1[x]))
            m2 = list(map(MapFunction, mesh_half_layer_2[x]))
            p1 = list(map(MapFunction, previous_layer[0][x]))
            p2 = list(map(MapFunction, previous_layer[1][x]))

            for y in range(len(m1)):
                m11 += m1[y] * p1[y]
                m12 += m1[y] * p2[y]
                m21 += m2[y] * p1[y]
                m22 += m2[y] * p2[y]
                m1t += m1[y]
                m2t += m2[y]
            
        w1 = m11/m1t**.5 * m22/m2t**.5
        w2 = m12/m1t**.5 * m21/m2t**.5


    if((m11 != 0 and m22 != 0 and w1 > w2)):
        mesh_half_layer_1, mesh_half_layer_2 = mesh_half_layer_1, mesh_half_layer_2
    elif((w2 > w1 and m12 != 0 and m21 != 0)):
        mesh_half_layer_1, mesh_half_layer_2 = mesh_half_layer_2, mesh_half_layer_1


    def getMeshValue(layer, x, y):
        val = 0
        if(x > -1 and x < len(layer) and y > -1 and y < len(layer[x])):
            val = layer[x][y]
        return val

    for ele in pathElements:
        mesh1a = getMeshValue(mesh_half_layer_1, ele[0], ele[1] + 1)
        mesh1r = getMeshValue(mesh_half_layer_1, ele[0] + 1, ele[1])
        mesh1b = getMeshValue(mesh_half_layer_1, ele[0], ele[1] - 1)
        mesh1l = getMeshValue(mesh_half_layer_1, ele[0] - 1, ele[1])
        mesh1ar = getMeshValue(mesh_half_layer_1, ele[0] + 1, ele[1] + 1)
        mesh1al = getMeshValue(mesh_half_layer_1, ele[0] - 1, ele[1] + 1)
        mesh1br = getMeshValue(mesh_half_layer_1, ele[0] + 1, ele[1] - 1)
        mesh1bl = getMeshValue(mesh_half_layer_1, ele[0] - 1, ele[1] - 1)
        mesh2a = getMeshValue(mesh_half_layer_2, ele[0], ele[1] + 1)
        mesh2r = getMeshValue(mesh_half_layer_2, ele[0] + 1, ele[1])
        mesh2b = getMeshValue(mesh_half_layer_2, ele[0], ele[1] - 1)
        mesh2l = getMeshValue(mesh_half_layer_2, ele[0] - 1, ele[1])
        mesh2ar = getMeshValue(mesh_half_layer_2, ele[0] + 1, ele[1] + 1)
        mesh2al = getMeshValue(mesh_half_layer_2, ele[0] - 1, ele[1] + 1)
        mesh2br = getMeshValue(mesh_half_layer_2, ele[0] + 1, ele[1] - 1)
        mesh2bl = getMeshValue(mesh_half_layer_2, ele[0] - 1, ele[1] - 1)
        mesh1_cardinal = [mesh1a, mesh1r, mesh1b, mesh1l]
        mesh1_cardinal.sort()
        cardianlSet = set(mesh1_cardinal)
        if(len(cardianlSet) == 1):
            changeDict[0b0000].append(ele)
        if((mesh1a == -1 and mesh1b == -1) or (mesh1r == -1 and mesh1l == -1) or ((0 in mesh1_cardinal and mesh1_cardinal.index(0) == 1) or (1 in mesh1_cardinal and mesh1_cardinal.index(1) == 1))):
            if(mesh1a == 1 or mesh2b == 1):
                changeDict[0b0011].append(ele)
            elif(mesh1r == 1 or mesh2l == 1):
                changeDict[0b0010].append(ele)
            elif(mesh1b == 1 or mesh2a == 1):
                changeDict[0b0001].append(ele)
            elif(mesh1l == 1 or mesh2r == 1):
                changeDict[0b0000].append(ele)
            elif(mesh1r == -1 or mesh1l == -1):
                changeDict[0b0011].append(ele)
            else:
                changeDict[0b0010].append(ele)
        else:
            if(mesh1a == -1 and mesh1r == -1):
                if(mesh1ar == 1 or mesh2al == 1 or mesh2l == 1 or mesh2bl == 1 or mesh2b == 1 or mesh2br == 1):
                    changeDict[0b1000].append(ele)
                else:
                    changeDict[0b1100].append(ele)
            elif(mesh1r == -1 and mesh1b == -1):
                if(mesh1br == 1 or mesh2bl == 1 or mesh2l == 1 or mesh2al == 1 or mesh2a == 1 or mesh2ar == 1):
                    changeDict[0b1011].append(ele)
                else:
                    changeDict[0b1111].append(ele)
            elif(mesh1b == -1 and mesh1l == -1):
                if(mesh1bl == 1 or mesh2al == 1 or mesh2a == 1 or mesh2ar == 1 or mesh2r == 1 or mesh2br == 1):
                    changeDict[0b1010].append(ele)
                else:
                    changeDict[0b1110].append(ele)
            elif(mesh1l == -1 and mesh1a == -1):
                if(mesh1al == 1 or mesh2ar == 1 or mesh2r == 1 or mesh2br == 1 or mesh2b == 1 or mesh2bl == 1):
                    changeDict[0b1001].append(ele)
                else:
                    changeDict[0b1101].append(ele)
    
    for key in changeDict.keys():
        for ele in changeDict[key]:
            mesh_half_layer_1[ele[0]][ele[1]] = key + 16
            mesh_half_layer_2[ele[0]][ele[1]] = (key ^ 0b0100) + 16
    
    return [mesh_half_layer_1, mesh_half_layer_2]


def CreateMeshLayerAddition(remesh, origin, voxelSize, z):
    X, Y = np.array(remesh).shape
    layer = trimesh.Trimesh()
    
    for x in range(X):
        row = trimesh.Trimesh()
        for y in range(Y):
            if(remesh[x][y] == 1):
                if(len(row.vertices) == 0):
                    row = CreateBox(x, y, z, origin, voxelSize)
                else:
                    row = trimesh.boolean.union([row, CreateBox(x, y, z, origin, voxelSize)])
            if(remesh[x][y] >= 16):
                if(len(row.vertices) == 0):
                    row = CreateBoundaryVoxel(x, y, z, origin, voxelSize, remesh[x][y])
                else:
                    row = trimesh.boolean.union([row, CreateBoundaryVoxel(x, y, z, origin, voxelSize, remesh[x][y])])

        if(len(layer.vertices) == 0):
            layer = row
        elif(not len(row.vertices) == 0):
            try:
                layer = trimesh.boolean.union([layer, row])
            except:
                layer.export("l.obj")
                row.export("r.obj")
                exit()
    return layer

def FindBoundary(voxelized_intersection, shape):
    flat_voxelized_intersection = list(map(Utilities.Flatten3DElement, voxelized_intersection))
    boundary = []
    for ele in voxelized_intersection:
        if(ele[0] == 0 or ele[1] == 0 or ele[0] == shape[0] - 1 or ele[1] == shape[1] - 1):
            boundary.append(ele)
        else:
            neighborhood = []
            for x in [-1, 0, 1]:
                for y in [-1, 0, 1]:
                    if x == 0 and y == 0:
                        continue
                    else:
                        neighborhood.append(ele + [x, y, 0])
            flag = True
            flat_neighborhood = list(map(Utilities.Flatten3DElement, neighborhood))
            for flat_ele in flat_neighborhood:
                flag = flat_ele in flat_voxelized_intersection
                if(not flag):
                    break
            if(not flag):
                boundary.append(ele)

    return boundary

def RemeshIntersection(mesh1, mesh2, voxel_size_inner, output_directory):
    voxel_size = voxel_size_inner * 5
    intersection_mesh = trimesh.boolean.intersection([mesh1, mesh2])
    voxelized_intersection = trimesh.voxel.creation.voxelize(intersection_mesh, voxel_size)
    
    voxelized_intersection = voxelized_intersection.fill()

    origin = voxelized_intersection.bounds[0]
    
    boundary_indices = FindBoundary(voxelized_intersection.sparse_indices, voxelized_intersection.shape)
    
    
    filtered_boundary_dictionary = defaultdict(list)
    filtered_layer_dictionary = defaultdict(list)
    
    for index in boundary_indices: 
        filtered_boundary_dictionary[f'{index[2]}'].append(index)
    
    for index in voxelized_intersection.sparse_indices: 
        filtered_layer_dictionary[f'{index[2]}'].append(Utilities.Flatten3DElement(index))

    filtered_layer_with_margin_dictionary = deepcopy(filtered_layer_dictionary)

    for key in filtered_boundary_dictionary.keys():
        list_margin = []
        for ele in filtered_boundary_dictionary[key]:
            for x in [-1,0,1]:
                for y in [-1,0,1]:
                    list_margin.append(ele + [x,y,0])
        filtered_layer_with_margin_dictionary[key].extend(list(map(Utilities.Flatten3DElement, list_margin)))
        filtered_layer_with_margin_dictionary[key] = sorted(set(filtered_layer_with_margin_dictionary[key]))

    final_mesh_dictionary = {}

    previous_z_key = None
    
    layers = list(filtered_boundary_dictionary.keys())
    layers.sort(key=int)  # the keys are strings; sort them as numbers so each layer follows the one below it


    for z in layers:
        boundary = filtered_boundary_dictionary[z]

        start, end, dist = 0, 0, 0
        for i1 in range(len(boundary)):
            for i2 in range(i1 + 1, len(boundary)):
                new_dist = abs(boundary[i1][0] - boundary[i2][0])**.5 + abs(boundary[i1][1] - boundary[i2][1])**.5
                
                if(new_dist > dist):
                    start, end = boundary[i1], boundary[i2]
                    dist = new_dist

        splitting_path = CreateLayerPath(voxelized_intersection.matrix[:,:,int(z)],[start, end])

        previous_layer = None if previous_z_key is None else final_mesh_dictionary[previous_z_key]
        final_mesh_dictionary[z] = RemeshVoxelizedIntersection(splitting_path, voxelized_intersection.matrix[:,:,int(z)], filtered_layer_dictionary[z], filtered_layer_with_margin_dictionary[z], int(z), previous_layer)
        previous_z_key = z
    
    mesh1AdditionList = []
    mesh2AdditionList = []

    diff1 = mesh1.difference(mesh2)
    diff2 = mesh2.difference(mesh1)


    for zString in final_mesh_dictionary.keys():
        if(final_mesh_dictionary[zString][1] is not None):
            mesh1AdditionList.append(CreateMeshLayerAddition(final_mesh_dictionary[zString][0], origin, voxel_size, int(zString)))  
        if(final_mesh_dictionary[zString][0] is not None):
            mesh2AdditionList.append(CreateMeshLayerAddition(final_mesh_dictionary[zString][1], origin, voxel_size, int(zString)))

    while None in mesh1AdditionList:
        mesh1AdditionList.remove(None)

    while None in mesh2AdditionList:
        mesh2AdditionList.remove(None)

    inter1 = trimesh.boolean.union(mesh1AdditionList)
    inter1 = trimesh.boolean.intersection([inter1, intersection_mesh])
    inter1.export(os.path.join(output_directory, "i1.obj"))
    diff1.export(os.path.join(output_directory, "d1.obj"))


    inter2 = trimesh.boolean.union(mesh2AdditionList)
    inter2 = trimesh.boolean.intersection([inter2, intersection_mesh])
    inter2.export(os.path.join(output_directory, "i2.obj"))
    diff2.export(os.path.join(output_directory, "d2.obj"))

    # The final parts: each input minus the overlap, plus its share of the overlap.
    for name, diff, inter in [("S1.stl", diff1, inter1), ("S2.stl", diff2, inter2)]:
        trimesh.boolean.union([diff, inter]).export(os.path.join(output_directory, name))



def LoadStamp(path):
    if not os.path.isfile(path):
        sys.exit(f"error: stamp {path} not found")
    stamp = trimesh.load(path, force='mesh')
    if not stamp.is_watertight:
        sys.exit(f"error: stamp {path} is not watertight")
    if stamp.bounds[0].min() < -1e-6 or stamp.bounds[1].max() > 1 + 1e-6:
        sys.exit(f"error: stamp {path} must lie in the unit cube [0, 1]^3")
    return stamp


parser = argparse.ArgumentParser(
    description="Split the overlap of two solids into two interlocking halves.")
parser.add_argument("input1", help="first solid (any mesh format trimesh reads)")
parser.add_argument("input2", help="second solid; must overlap the first")
parser.add_argument("output_directory", help="created if it does not exist")
parser.add_argument("feature_size", type=float,
                    help="stamp feature size; one voxel is 5 x feature_size")
parser.add_argument("stamps", nargs="*", metavar="stamp",
                    help="optional custom stamps, all four or none: flat flatAlt corner cornerAlt "
                         "(watertight meshes in the unit cube [0, 1]^3)")
args = parser.parse_args()
if args.feature_size <= 0:
    parser.error("feature_size must be positive")
if len(args.stamps) not in (0, 4):
    parser.error("give all four stamps (flat flatAlt corner cornerAlt) or none")

if args.stamps:
    flat, flatAlt, corner, cornerAlt = map(LoadStamp, args.stamps)
else:
    MakeFlatInter()
    MakeCornerInter()

os.makedirs(args.output_directory, exist_ok=True)
for path in (args.input1, args.input2):
    if not os.path.isfile(path):
        sys.exit(f"error: input {path} not found")
mesh1 = trimesh.load(args.input1, force='mesh')
mesh2 = trimesh.load(args.input2, force='mesh')
RemeshIntersection(mesh1, mesh2, args.feature_size, args.output_directory)
