import open3d as o3d
import argparse
import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Normalize the input mesh\'s bounding box to [-1, 1] x [-1, 1] x [-1, 1].')
    parser.add_argument('--file', type=str, required=True, help='The target model file.')
    args = parser.parse_args()

    input_filename = args.file
    output_filename = '.'.join(input_filename.split('.')[:-1]) + '_n.' + input_filename.split('.')[-1]
    print('Input file:', input_filename)
    print('Output file:', output_filename)

    mesh = o3d.io.read_triangle_mesh(input_filename)
    bbox = mesh.get_max_bound() - mesh.get_min_bound()
    scale = 2 / bbox.min()
    mesh.scale(scale, np.array([0, 0, 0]))

    o3d.io.write_triangle_mesh(output_filename, mesh)
