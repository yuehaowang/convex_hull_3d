import open3d as o3d
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='A mesh simplifier.')
    parser.add_argument('--file', type=str, required=True, help='The target model file.')
    parser.add_argument('--fineness', type=int, default=16, help='The fineness of simplification.')
    args = parser.parse_args()

    input_filename = args.file
    output_filename = input_filename.split('.')[-2] + ('_sim%s.' % args.fineness) + input_filename.split('.')[-1]
    print('Input file:', input_filename)
    print('Output file:', output_filename)

    mesh = o3d.io.read_triangle_mesh(input_filename)

    voxel_size = max(mesh.get_max_bound() - mesh.get_min_bound()) / args.fineness
    print(f'voxel_size = {voxel_size:e}')

    mesh = mesh.simplify_vertex_clustering(
        voxel_size=voxel_size,
        contraction=o3d.geometry.SimplificationContraction.Average)

    o3d.io.write_triangle_mesh(output_filename, mesh)
