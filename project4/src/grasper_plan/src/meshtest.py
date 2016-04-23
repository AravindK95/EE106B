import obj_file
import mesh
import sys

mesh = obj_file.ObjFile(sys.argv[1]).read()
mesh.compute_normals()
# mesh.tri_normals()

def print_lens(mesh=mesh):
    print("num vertices: "+str(len(mesh.vertices)))
    print("num triangles: "+str(len(mesh.triangles)))
    print("num normals: "+str(len(mesh.normals)))
