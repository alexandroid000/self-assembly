import numpy as np
from simple_polygon import Simple_Polygon
from random import uniform
from helper.shoot_ray_helper import IsInPoly, ClosestPtAlongRay

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

def mk_bounding_box(poly):
    vs = poly.vertex_list_per_poly[0]
    xs = np.sort([v[0] for i, v in vs])
    ys = np.sort([v[1] for i, v in vs])
    min_x = xs[0]
    max_x = xs[-1]
    min_y = ys[0]
    max_y = ys[-1]
    bb_verts = np.array([(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)])
    bb = Simple_Polygon("bb"+poly.name, bb_verts)
    return min_x, max_x, min_y, max_y, bb

def uniform_sample_from_poly(poly, n):
    min_x, max_x, min_y, max_y, bb = mk_bounding_box(poly)
    samples = [[0,0]]*n
    for i in range(n):
        sample = [uniform(min_x, max_x), uniform(min_y, max_y)]
        while not IsInPoly(sample, poly):
            sample = uniform(min_x, max_x), uniform(min_y, max_y)
        samples[i] = sample
    return samples


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

def midpoint(pt1, pt2):
    return (pt1+pt2)/2

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
        if self.dir == "CCW":
            return field_strength*normalize(rotate_vector(normal, np.pi/2.))
        else:
            return np.array([0., 0.])

# magnetic fields follow superposition principle
def force_from_wires(wires, xy):
    force = np.array([0.0, 0.0])
    for w in wires:
        force += w.force_at(xy)
    return force

# running policies
# ----------------

# hardcoded: four wires, each wire is either CW, CCW, or X
# let wires be arranged as:
    # 0  1
    # 2  3
# let CW := 0
#     CCW := 1
#     X := 2
#
# encode with base-3 number system

wire_to_state = {"CW":0, "CCW":1, "X":2}
state_to_wire = {0:"CW", 1:"CCW", 2:"X"}

def encode_policy(wires):
    [w0, w1, w2, w3] = wires # TODO use pycontracts
    policy = 0
    for i,w in enumerate(wires):
        policy += wire_to_state[w]*(3**i)
    return policy

def decode_policy(policy):
    states = ["", "", "", ""]
    for i in range(4):
        mod_policy = policy % 3
        states[i] = mod_policy
        policy = policy // 3
    str_states = [state_to_wire[s] for s in states]
    return str_states
