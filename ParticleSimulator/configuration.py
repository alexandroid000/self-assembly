from simple_polygon import Simple_Polygon
from maps import *
from utilities import *
import numpy as np

# System Configuration
# --------------------

# define simulation parameters here

L = 3.0
N = 100
T = 31
R = 0.02
border_region = R
allow_attachment = False

#import yaml

# System Configuration
# --------------------

# define simulation parameters here

#with open("configuration.yaml", 'r') as f:
#    data = yaml.load(f)



#square = Simple_Polygon("square",np.array([[0.0,0.0], [L, 0.0],[L,L],[0.0,L]]))
#square_hole = Simple_Polygon("sqh",simple_holes[0], simple_holes[1])
#spikes = Simple_Polygon("spikes",np.array(mk_spiky_circle(8, 0.5*L)))

spike_annulus = Simple_Polygon("spk_ring",
                               np.array(mk_spiky_circle(8, 0.5*L)),
                              [np.array(mk_obstacle(mk_regpoly(4, 0.4*L)))])

oct_verts = np.array(mk_regpoly(8, 0.8*L, offset=np.pi/8.))
wire_verts = np.array(mk_regpoly(4, 0.4*L, offset=np.pi/4))


r1 = [wire_verts[0], midpoint(oct_verts[0], oct_verts[1]), oct_verts[1], oct_verts[2],
midpoint(oct_verts[2], oct_verts[3]), wire_verts[1]]
r2 = [wire_verts[1], midpoint(oct_verts[2], oct_verts[3]), oct_verts[3], oct_verts[4],
midpoint(oct_verts[4], oct_verts[5]), wire_verts[2]]
r3 = [wire_verts[2], midpoint(oct_verts[4], oct_verts[5]), oct_verts[5], oct_verts[6],
midpoint(oct_verts[6], oct_verts[7]), wire_verts[3]]
r4 = [wire_verts[3], midpoint(oct_verts[6], oct_verts[7]), oct_verts[7], oct_verts[0],
midpoint(oct_verts[0], oct_verts[1]), wire_verts[0]]

rs = [wire_verts, r1, r2, r3, r4]
rs_as_obs = [mk_obstacle(r) for r in rs]
regions = [Simple_Polygon("r"+str(i), np.array(vs)) for i,vs in enumerate(rs)]

octagon = Simple_Polygon("octagon", oct_verts)
env = octagon

# type A particles:
    # faster
    # smaller rotational drift
    # escape from walls more quickly
A_properties = {'vel':1.0, 'wall_prob': 0.05, 'beta': 0.2}

# type B particles:
    # slower
    # more rotational drift
    # get stuck on walls more
B_properties = {'vel':0.3, 'wall_prob': 0.2, 'beta': 0.5}

properties = { 'A-free':A_properties
             ,'B-free':B_properties
             ,'A-wall':A_properties
             ,'B-wall':B_properties
             }
