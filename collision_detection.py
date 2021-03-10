import numpy as np
import open3d as o3d


class RigidBody():
    def __init__(self, mesh, conv_mesh):
        self.mesh = mesh
        self.mesh.compute_vertex_normals()
        self.convhull = conv_mesh
        self.convhull.compute_vertex_normals()
        self.convhull_wf = o3d.geometry.LineSet.create_from_triangle_mesh(self.convhull)
        self.paintConvexHull([0, 0, 1.0])

        self.vertices = np.asarray(self.convhull.vertices)
        self.vertex_normals = np.asarray(self.convhull.vertex_normals)
        self.edges = np.asarray(self.convhull_wf.lines)

    def addTo(self, vis):
        vis.add_geometry(self.mesh)
        vis.add_geometry(self.convhull_wf)

    def paintConvexHull(self, color):
        self.convhull_wf.paint_uniform_color(np.array(color).astype(np.float64))

    def get_max_bound(self):
        return self.convhull.get_max_bound()
    
    def get_min_bound(self):
        return self.convhull.get_min_bound()

    def translate(self, t):
        self.mesh.translate(t)
        self.convhull.translate(t)
        self.convhull_wf.translate(t)


class SAT3D():
    '''
    Implementation of Separating Axis Theorem for 3D convex hulls
    @param obj1
        type: RigidBody
        The first collider
    @param obj2
        type: RigidBody
        The second collider
    '''

    def __init__(self, obj1, obj2):
        self.bodyA = obj1
        self.bodyB = obj2
        
        self.axes = self._build_proj_axes()

    def _build_proj_axes(self):
        A_normals = self.bodyA.vertex_normals
        B_normals = self.bodyB.vertex_normals
        axes = np.concatenate([A_normals, B_normals], axis=0)

        A_edges = self._build_edge_vec(self.bodyA)
        B_edges = self._build_edge_vec(self.bodyB)
        A_edges = A_edges[:, np.newaxis, :]
        axes = np.concatenate([axes, np.reshape(np.cross(A_edges, B_edges), (-1, 3))], axis=0)

        return axes

    def _build_edge_vec(self, body):
        return body.vertices[body.edges[:, 0]] - body.vertices[body.edges[:, 1]]

    def hitTest(self):
        chunk_size = 10000
        chunks = list(range(0, self.axes.shape[0], chunk_size))
        if not self.axes.shape[0] in chunks:
            chunks.append(self.axes.shape[0])

        for i in range(len(chunks[:-1])):
            axes_chunk = self.axes[chunks[i]:chunks[i + 1], :]
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

