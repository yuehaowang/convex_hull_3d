def endpoint_key(*arg):
    t = [str(a) for a in sorted(arg)]
    return '-'.join(t)


class Edge():
    def __init__(self, p1, p2):
        # Indices of endpoints
        self.p1 = p1
        self.p2 = p2
        # Adjacent faces (suppose manifold)
        self.adj_faces = []

    def add_adjacent_face(self, f):
        self.adj_faces.append(f)

        if len(self.adj_faces) > 2:
            print('WARNING: edge(%s): non-manifold edge' % endpoint_key(self.p1, self.p2))

    def remove_adjacent_face(self, f):
        if len(self.adj_faces) == 0:
            print('WARNING: edge(%s): no adjacent face to remove' % endpoint_key(self.p1, self.p2))
            return

        self.adj_faces.remove(f)


class Face():
    def __init__(self, vtx):
        # Indices of vertices
        self.vertices = vtx

        if len(self.vertices) > 3:
            print('WARNING: face(%s): non-triangular face' % endpoint_key(*self.vertices))
