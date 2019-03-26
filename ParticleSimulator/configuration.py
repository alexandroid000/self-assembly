from simple_polygon import Simple_Polygon
from maps import *
from utilities import *
import numpy as np

# System Configuration
# --------------------

# define simulation parameters here

L = 3.0
N = 20
T = 200
R = 0.02
border_region = R
allow_attachment = False



#square = Simple_Polygon("square",np.array([[0.0,0.0], [L, 0.0],[L,L],[0.0,L]]))
#square_hole = Simple_Polygon("sqh",simple_holes[0], simple_holes[1])
#spikes = Simple_Polygon("spikes",np.array(mk_spiky_circle(8, 0.5*L)))

spike_annulus = Simple_Polygon("spk_ring",
                               np.array(mk_spiky_circle(8, 0.5*L)),
                              [np.array(mk_obstacle(mk_regpoly(4, 0.4*L)))])

oct_verts = np.array(mk_regpoly(8, 0.5*L, offset=np.pi/8.))
wire_verts = np.array(mk_regpoly(4, 0.25*L, offset=np.pi/4))
orientations = ["CW", "CCW", "CW", "CCW"]
wires = [Wire(v, o) for v, o in zip(wire_verts, orientations)]
octagon = Simple_Polygon("octagon", oct_verts)


env = octagon
simname = env.name+'_'+str(L)+"_N"+str(N)+"_T"+str(T)

# type A particles:
    # faster
    # smaller rotational drift
    # escape from walls more quickly
A_properties = {'vel':0.3, 'wall_prob': 0.05, 'beta': 0.2}

# type B particles:
    # slower
    # more rotational drift
    # get stuck on walls more
B_properties = {'vel':0.05, 'wall_prob': 0.2, 'beta': 0.5}

properties = { 'A-free':A_properties
             ,'B-free':B_properties
             ,'A-wall':A_properties
             ,'B-wall':B_properties
             }
