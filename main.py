import argparse
import numpy as np
import open3d as o3d
from convhull import ConvexHull3D


if __name__ == '__main__':
    # Arguments
    parser = argparse.ArgumentParser(description='Compute the convex hull of a 3D object.')
    parser.add_argument('--file', type=str, required=True, help='The target model file.')
    parser.add_argument('--vis', action='store_true', help='Whether visualize the result?')
    parser.add_argument('--save_path', type=str, default='', help='The saving path of the result.')
    args = parser.parse_args()

    # Load the mesh
    mesh = o3d.io.read_triangle_mesh(args.file)
    print('\nLoaded mesh file %s' % args.file)
    print('#vertices:', np.asarray(mesh.vertices).shape[0])
    print('#faces:', np.asarray(mesh.triangles).shape[0])
    print('Is manifold:', mesh.is_edge_manifold())
    print('Is self-intersecting:', mesh.is_self_intersecting())
    print('Is watertight:', mesh.is_watertight())

    # Compute the 3D convex hull
    print('\nComputing convex hull...')
    convhull = ConvexHull3D(np.asarray(mesh.vertices), show_progress=True)

    # Save the result
    if args.save_path:
        print('\nSaving result...')
        convhull.save(args.save_path)

    # Visualize the mesh
    if args.vis:
        print('\nVisualizing...')

        # Compute normals of the input mesh
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()

        # Get the mesh of the convex hull
        mesh_convhull = convhull.to_o3d_mesh()
        # Get the wireframe of the convex hull mesh
        wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh_convhull)

        # Draw input mesh and the convex hull mesh
        o3d.visualization.draw_geometries([mesh, wireframe])
