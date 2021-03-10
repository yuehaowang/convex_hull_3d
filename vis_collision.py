import open3d as o3d
import numpy as np
import argparse
import time
from collision_detection import RigidBody, SAT3D


class CollisionDetectionVisualizer():
    MOVE_SPEED = 0.1

    def __init__(self, obj1, obj2):
        # Create two bodies
        self.bodyA = RigidBody(*obj1)
        self.bodyB = RigidBody(*obj2)
        self._initialize_pos()

        # Create a coordinate frame
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(2)

        # Create a visualizer
        self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self.visualizer.create_window()

        self.visualizer.add_geometry(self.coord_frame)

        # Add the two bodies to the visualizer
        self.bodyA.addTo(self.visualizer)
        self.bodyB.addTo(self.visualizer)

        # Create a SAT
        self.sat_detector = SAT3D(self.bodyA, self.bodyB)

        ## Add keyboard input callback
        # Left
        self.visualizer.register_key_action_callback(65, lambda vis, a, m: self._translate(a, np.array([-1, 0, 0])))
        # Right
        self.visualizer.register_key_action_callback(68, lambda vis, a, m: self._translate(a, np.array([1, 0, 0])))
        # Down
        self.visualizer.register_key_action_callback(83, lambda vis, a, m: self._translate(a, np.array([0, -1, 0])))
        # Up
        self.visualizer.register_key_action_callback(87, lambda vis, a, m: self._translate(a, np.array([0, 1, 0])))
        # Backward
        self.visualizer.register_key_action_callback(81, lambda vis, a, m: self._translate(a, np.array([0, 0, -1])))
        # Forward
        self.visualizer.register_key_action_callback(69, lambda vis, a, m: self._translate(a, np.array([0, 0, 1])))

    def show(self):
        self.visualizer.run()
        self.visualizer.destroy_window()

    def _initialize_pos(self):
        offset_x = (self.bodyA.get_max_bound() - self.bodyA.get_min_bound())[0] + 2
        self.bodyA.translate(np.array([-2, 0, 0]))
        self.bodyB.translate(np.array([offset_x, 0, 0]))


    def _translate(self, action, dir):
        dir = dir.astype(np.float64)
        self.bodyA.translate(dir * self.MOVE_SPEED)

        if action == 0:
            t0 = time.time()
            is_hit = self.sat_detector.hitTest()
            print('Hit result: %s, time consumed: %s' % (is_hit, time.time() - t0))
            if is_hit:
                self.bodyA.paintConvexHull([1.0, 0, 0])
            else:
                self.bodyA.paintConvexHull([0, 0, 1.0])

        return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize collision detection between convex hulls.')
    parser.add_argument('-m1', type=str, required=True, help='The first mesh  file.')
    parser.add_argument('-ch1', type=str, required=True, help='The convex hull file of the first mesh.')
    parser.add_argument('-m2', type=str, required=True, help='The first mesh  file.')
    parser.add_argument('-ch2', type=str, required=True, help='The convex hull file of the first mesh.')
    args = parser.parse_args()

    mesh1 = o3d.io.read_triangle_mesh(args.m1)
    convhull1 = o3d.io.read_triangle_mesh(args.ch1)

    mesh2 = o3d.io.read_triangle_mesh(args.m2)
    convhull2 = o3d.io.read_triangle_mesh(args.ch2)

    visualizer = CollisionDetectionVisualizer((mesh1, convhull1), (mesh2, convhull2))
    visualizer.show()
