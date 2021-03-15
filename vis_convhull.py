import open3d as o3d
import numpy as np
import argparse


def vis(obj, convhull):
    convhull_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(convhull)
    convhull_wireframe.paint_uniform_color(np.array([0, 0, 1.0]))

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name='Convex Hull Visualizer')
    visualizer.add_geometry(obj)
    visualizer.add_geometry(convhull_wireframe)
    visualizer.run()
    visualizer.destroy_window()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize a mesh and its convex hull.')
    parser.add_argument('-m', type=str, required=True, help='The mesh model file.')
    parser.add_argument('-ch', type=str, required=True, help='The convex hull file.')
    args = parser.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.m)
    mesh.compute_vertex_normals()

    convhull = o3d.io.read_triangle_mesh(args.ch)
    vis(mesh, convhull)
