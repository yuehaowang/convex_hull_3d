import numpy as np
import open3d as o3d
import math
from utils import endpoint_key, Edge, Face


class RigidBody():
    def __init__(self, mesh, conv_mesh):
        self.mesh = mesh
        self.mesh.compute_vertex_normals()
        self.convhull = conv_mesh
        self.convhull.compute_triangle_normals()
        self.convhull.remove_duplicated_vertices()
        self.convhull.remove_duplicated_triangles()

        # Faces, vertices, normals of the convex hull
        self.faces = np.asarray(self.convhull.triangles)
        self.vertices = np.asarray(self.convhull.vertices)
        self.face_normals = np.asarray(self.convhull.triangle_normals)
        self.edges, self.edges_gauss_map = self._build_edges()

    def get_max_bound(self):
        return self.convhull.get_max_bound()
    
    def get_min_bound(self):
        return self.convhull.get_min_bound()

    def translate(self, t):
        self.mesh.translate(t)
        self.convhull.translate(t)

    def _build_edges(self):
        # Search for all edges and record their adjacent faces' normals
        edges = {}
        for j, f in enumerate(self.faces):
            for i in range(len(f)):
                ep1 = f[i]
                ep2 = f[(i + 1) % 3]

                e_key = endpoint_key(ep1, ep2)
                if not e_key in edges:
                    edges[e_key] = Edge(ep1, ep2)

                # Only record normals of two adjacent faces
                edges[e_key].add_adjacent_face(self.face_normals[j])

        # Extract edges and Gauss map to np array
        edges_list = []
        gauss_map = []
        for k in edges:
            e = edges[k]
            
            # Skip non-manifold edges
            if len(e.adj_faces) != 2:
                continue

            edges_list.append([e.p1, e.p2])
            gauss_map.append([e.adj_faces[0], e.adj_faces[1]])

        return np.array(edges_list), np.array(gauss_map)


class SAT3D():
    '''
    Implementation of Separating Axis Theorem for 3D convex hulls
    @param obj1
        type: RigidBody
        The first collider
    @param obj2
        type: RigidBody
        The second collider
    @param num_chunks
        type: int
        Chunk number of separating axes
    '''

    def __init__(self, obj1, obj2, num_chunks=80):
        self.bodyA = obj1
        self.bodyB = obj2
        
        self.axes = self._build_proj_axes()
        self.chunks = self._build_axes_chunks(num_chunks)

    def _build_proj_axes(self):
        # Face normal axes
        A_normals = self.bodyA.face_normals
        B_normals = self.bodyB.face_normals
        axes = np.concatenate([A_normals, B_normals], axis=0)

        A_edges = self._build_edge_vec(self.bodyA)
        gauss_a, gauss_b = self.bodyA.edges_gauss_map[:, 0, :], self.bodyA.edges_gauss_map[:, 1, :]
        B_edges = self._build_edge_vec(self.bodyB)
        gauss_c, gauss_d = self.bodyB.edges_gauss_map[:, 0, :], self.bodyB.edges_gauss_map[:, 1, :]

        # Edge to edge axes
        edge_edge_axes = np.reshape(np.cross(A_edges[:, np.newaxis, :], B_edges), (-1, 3))
        # Check intersection on Gauss map
        bxa = np.cross(gauss_b, gauss_a)
        dxc = np.cross(gauss_d, gauss_c)
        cba = np.reshape(gauss_c @ bxa.T, (-1,))
        dba = np.reshape(gauss_d @ bxa.T, (-1,))
        adc = np.reshape(gauss_a @ dxc.T, (-1,))
        bdc = np.reshape(gauss_b @ dxc.T, (-1,))
        is_minkowski_face = (cba * dba < 0) & (adc * bdc < 0) & (cba * bdc > 0)
        # Edge pruning according to Gauss map
        edge_edge_axes = edge_edge_axes[is_minkowski_face]

        axes = np.concatenate([axes, edge_edge_axes], axis=0)

        return axes

    def _build_edge_vec(self, body):
        return body.vertices[body.edges[:, 0]] - body.vertices[body.edges[:, 1]]

    def _build_axes_chunks(self, num_chunks):
        size = self.axes.shape[0]
        chunk_size = math.ceil(size / num_chunks)
        chunks = list(range(0, size, chunk_size))
        chunks.append(size)
        return chunks

    def hit_test(self):
        for i in range(len(self.chunks[:-1])):
            axes_chunk = self.axes[self.chunks[i]:self.chunks[i + 1], :]
            A_projs = axes_chunk @ self.bodyA.vertices.T
            B_projs = axes_chunk @ self.bodyB.vertices.T
            A_proj_maxs = np.max(A_projs, axis=1)
            A_proj_mins = np.min(A_projs, axis=1)
            B_proj_maxs = np.max(B_projs, axis=1)
            B_proj_mins = np.min(B_projs, axis=1)
            maxs = np.max(np.stack([A_proj_maxs, B_proj_maxs], axis=1), axis=1)
            mins = np.min(np.stack([A_proj_mins, B_proj_mins], axis=1), axis=1)

            not_overlay = ((A_proj_maxs - A_proj_mins) + (B_proj_maxs - B_proj_mins) < (maxs - mins)).any()
            if not_overlay:
                return False
        
        return True

