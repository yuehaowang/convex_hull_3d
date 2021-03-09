import open3d as o3d
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize mesh and its convex hull.')
    parser.add_argument('--mesh', type=str, required=True, help='The mesh model file.')
    parser.add_argument('--convhull', type=str, required=True, help='The convex hull file.')
    args = parser.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.mesh)
    mesh.compute_vertex_normals()

    convhull = o3d.io.read_triangle_mesh(args.convhull)
    convhull_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(convhull)
    
    o3d.visualization.draw_geometries([mesh, convhull_wireframe])
