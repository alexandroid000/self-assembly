from simple_polygon import Simple_Polygon
import numpy as np

# System Configuration
# --------------------

# define simulation parameters here
# todo: abstract out into config file

L = 2.0
N = 25
T = 500
R = 0.02
simname = "box_L"+str(L)+"_N"+str(N)+"_T"+str(T)
# Define square cell
#cell = Cell(side=[L,L])
cell = Simple_Polygon("square",np.array([[0.0,0.0], [L, 0.0],[L,L],[0.0,L]]))
border_region = R # TODO: should be tied to radius of agents, not env size

A_properties = {'vel':0.5, 'wall_prob': 0.05, 'beta': 1.0}
B_properties = {'vel':0.05, 'wall_prob': 0.2, 'beta': 0.5}
properties = { 'A-free':A_properties
             ,'B-free':B_properties
             ,'A-wall':A_properties
             ,'B-wall':B_properties
             }
