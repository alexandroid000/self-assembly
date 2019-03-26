import numpy as np

# Environment Utilities
# -----------------

def mk_spiky_circle(n, r):
    d = 2*np.pi/n
    theta = 0
    pts = []
    for i in range(n):
        pt1 = [r*np.cos(theta), r*np.sin(theta)]
        r2 = 1.5*r
        pt2 = [r2*np.cos(theta), r2*np.sin(theta)]
        theta += d
        pts.extend([pt1, pt2])
    return pts

def mk_spiky_obstacle(n, r):
    d = 2*np.pi/n
    theta = 0.0
    pts = []
    for i in range(n):
        pt1 = [r*np.cos(theta), r*np.sin(theta)]
        r2 = 0.6*r
        th2 = theta + 0.3*d
        pt2 = [r2*np.cos(th2), r2*np.sin(th2)]
        theta += d
        pts.extend([pt1, pt2])
    return pts[::-1]

def mk_regpoly(n, r, offset=0.0):
    d = 2*np.pi/n
    theta = offset
    pts = []
    for i in range(n):
        pt = [r*np.cos(theta), r*np.sin(theta)]
        theta += d
        pts.append(pt)
    return pts

def mk_obstacle(vertices):
    return vertices[::-1]

# Geometric Operations
# --------------------


# http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
def closest_edge(pt, poly):
    [x0,y0] = pt
    vs = poly.vertex_list_per_poly
    n = poly.size
    components = len(vs)
    min_d = 100000000000
    closest_component = -1
    closest_edge = -1
    # find closest edge over external boundary and holes
    for (i, component) in enumerate(vs):
        m = len(component)
        for j in range(m):
            [x1, y1], [x2,y2] = component[j][1], component[(j+1) % m][1]
            d = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / np.sqrt((x2-x1)**2 + (y2-y1)**2)
            if d < min_d:
                min_d = d
                closest_component = i
                closest_edge = j
    return min_d, closest_component, closest_edge

def dist_dir_closest_edge(pt, poly):
    d, c, j = closest_edge(pt, poly)
    vs = poly.vertex_list_per_poly
    csize = len(vs[c])
    edge_vect = vs[c][(j + 1) % csize][1] - vs[c][j][1]
    return d, edge_vect

def normalize(vector):
    norm = np.linalg.norm(vector)
    return vector/norm

def rotate_vector(v, theta):
    vx, vy = v[0], v[1]
    return np.array( [np.cos(theta)*vx - np.sin(theta)*vy,
                     np.sin(theta)*vx + np.cos(theta)*vy])

# Magnetic flow field generation
# ------------------------------

class Wire():
    def __init__(self, xy, dir):
        self.xy = xy
        self.dir = dir

    def force_at(self, xy_measured):
        normal = xy_measured - self.xy
        # current flowing through wire creates mag field that drops off as 1/r
        field_strength = 1.0/np.linalg.norm(normal)
        if self.dir == "CW":
            return field_strength*normalize(rotate_vector(normal, 3.*np.pi/2.))
        else:
            return field_strength*normalize(rotate_vector(normal, np.pi/2.))

# magnetic fields follow superposition principle
def force_from_wires(wires, xy):
    force = np.array([0.0, 0.0])
    for w in wires:
        force += w.force_at(xy)
    return force
