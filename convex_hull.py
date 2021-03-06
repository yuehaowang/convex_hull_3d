import argparse
import numpy as np
from tqdm import tqdm
import open3d as o3d
from utils import endpoint_key, Edge, Face
import vis_convhull
import time


def _signed_vol(a, b, c, p):
    '''
    Compute signed volume of a tetrahedron
    '''
    return np.linalg.det(np.concatenate([np.array([a, b, c, p]), np.array([[1.0, 1.0, 1.0, 1.0]]).T], axis=1))


def _trash_bin(o, trash_flags):
   for k in trash_flags:
        if trash_flags[k]:
            o.pop(k)


class ConvexHull3D():
    '''
    Incremental convex hull for 3D objects
    @param vtxs
        input vertices with 3D coordinates
        type: np.array
        shape: |V| x 3
    '''

    def __init__(self, vtxs, show_progress=False):
        # Faces, edges of the convex hull
        self.faces = {}
        self.edges = {}
        # Input vertices
        self._in_vtxs = list(vtxs)

        self._show_progress = show_progress

        self._initialize_hull()
        self._incremental()

    def _add_face(self, p1, p2, p3, p4=None):
        vtxs = self._in_vtxs
        
        # Order vertices counterclockwise
        f = Face([p1, p2, p3])
        if p4:
            vol = _signed_vol(vtxs[p1], vtxs[p2], vtxs[p3], vtxs[p4])
            if vol < 0:
                f = Face([p1, p3, p2])
            elif vol == 0:
                print('WARNING: coplanar tetrahedron (%s-%s-%s-%s)' % (p1, p2, p3, p4))

        self.faces[endpoint_key(p1, p2, p3)] = f

        # Set edges of the new face
        for i in range(len(f.vertices)):
            ep1 = f.vertices[i]
            ep2 = f.vertices[(i + 1) % len(f.vertices)]

            if not endpoint_key(ep1, ep2) in self.edges:
                edge = Edge(ep1, ep2)
                self.edges[endpoint_key(ep1, ep2)] = edge
                
            self.edges[endpoint_key(ep1, ep2)].add_adjacent_face(endpoint_key(p1, p2, p3))
            
    def _initialize_hull(self):
        self._add_face(2, 3, 4, 1)
        self._add_face(1, 3, 4, 2)
        self._add_face(1, 2, 4, 3)
        self._add_face(1, 2, 3, 4)

    def _incremental(self):
        vtxs = self._in_vtxs

        iter_obj = range(5, len(vtxs))
        if self._show_progress:
            iter_obj = tqdm(iter_obj)
        
        for pi in iter_obj:

            # Find visible faces according to pi, which will be removed then.
            visible_ls = {}
            has_vis_face = False

            for f_key in self.faces:
                f = self.faces[f_key]

                # Compute the signed volume of (f, pi) to detect whether f is visible, i.e. signed vol. < 0.
                vol = _signed_vol(vtxs[f.vertices[0]], vtxs[f.vertices[1]], vtxs[f.vertices[2]], vtxs[pi])
                if vol == 0:
                    # Coplanar tetrahedron => skip pi since pi should NOT be on the convex hull
                    has_vis_face = False
                    break

                is_visible = vol < 0
                visible_ls[f_key] = is_visible
                has_vis_face = has_vis_face or is_visible

            if not has_vis_face:
                continue


            # Find border edges and create new faces
            edge_trash_flags = {}
            new_faces = []

            for e_key in self.edges:
                edge = self.edges[e_key]

                # Case 0: border edge
                if len(edge.adj_faces) < 2:
                    edge_trash_flags[e_key] = False
                # Case 1: both two adjacent faces are visible
                elif visible_ls[edge.adj_faces[0]] and visible_ls[edge.adj_faces[1]]:
                    edge_trash_flags[e_key] = True
                # Case 2: one face is visible and another is invisible
                elif visible_ls[edge.adj_faces[0]] or visible_ls[edge.adj_faces[1]]:
                    edge_trash_flags[e_key] = False

                    # Determine which face is invisible
                    visible_f_key = edge.adj_faces[0]
                    invisible_f_key = edge.adj_faces[1]
                    if not visible_ls[visible_f_key]:
                        visible_f_key = edge.adj_faces[1]
                        invisible_f_key = edge.adj_faces[0]

                    # Remove the visible face from the edge's adjacent face list
                    edge.remove_adjacent_face(visible_f_key)
                    
                    # Find the point inner the visible face
                    for vface_p in self.faces[visible_f_key].vertices:
                        if not vface_p in self.faces[invisible_f_key].vertices:
                            break

                    # Create a new face
                    new_faces.append([edge.p1, edge.p2, pi, vface_p])


            # Trash abandoned edges and faces
            _trash_bin(self.edges, edge_trash_flags)
            _trash_bin(self.faces, visible_ls)

            # Add new faces
            for new_face in new_faces:
                self._add_face(*new_face)

    def to_o3d_mesh(self):
        face_idx_ls = []
        for f_key in self.faces:
            f = self.faces[f_key]
            face_idx_ls.append([f.vertices[0], f.vertices[1], f.vertices[2]])
        triangles = o3d.utility.Vector3iVector(face_idx_ls)

        return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(self._in_vtxs), triangles)

    def save(self, path):
        return o3d.io.write_triangle_mesh(path, self.to_o3d_mesh())



if __name__ == '__main__':
    # Arguments
    parser = argparse.ArgumentParser(description='Compute the convex hull of a 3D object.')
    parser.add_argument('--file', type=str, help='The target model file.')
    parser.add_argument('--save_path', type=str, default='', help='The saving path of the result. (not supported in perf-test mode)')
    parser.add_argument('--perf_test', action='store_true', help='Enable perf-test mode')
    parser.add_argument('--num_pts', type=int, default=1000, help='The number of generated points (perf-test mode only).')
    parser.add_argument('--num_trials', type=int, default=10, help='The number of trials (perf-test mode only).')
    parser.add_argument('--vis', action='store_true', help='Whether visualize the result? (only visualize the last trial in perf-test mode)')
    args = parser.parse_args()

    target_obj = None

    if args.file:
        # Load the mesh
        mesh = o3d.io.read_triangle_mesh(args.file)
        # Compute normals of the input mesh
        mesh.compute_vertex_normals()

        print('\nLoaded mesh file %s' % args.file)
        print('#vertices:', np.asarray(mesh.vertices).shape[0])
        print('#faces:', np.asarray(mesh.triangles).shape[0])
        print('Is edge manifold:', mesh.is_edge_manifold())
        print('Is self-intersecting:', mesh.is_self_intersecting())
        print('Is watertight:', mesh.is_watertight())

        # Compute the 3D convex hull
        print('\nComputing convex hull...')
        convhull = ConvexHull3D(np.asarray(mesh.vertices), show_progress=True)
    
        # Save the result
        if args.save_path:
            print('\nSaving result...')
            convhull.save(args.save_path)

        target_obj = mesh

    elif args.perf_test:
        time_ls = []
        for i in range(args.num_trials):
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.random.randn(args.num_pts, 3)))

            t0 = time.time()
            convhull = ConvexHull3D(np.asarray(pc.points), show_progress=False)
            time_ls.append(time.time() - t0)

            print('Time consumed: %ss' % time_ls[-1])

        print('Avg time consumed: %ss' % (sum(time_ls) / 10))

        target_obj = pc

    # Visualize the mesh
    if args.vis and target_obj:
        print('\nVisualizing...')
        # Get the mesh of the convex hull
        mesh_convhull = convhull.to_o3d_mesh()
        vis_convhull.vis(target_obj, mesh_convhull)
