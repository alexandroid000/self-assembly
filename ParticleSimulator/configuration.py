from simple_polygon import Simple_Polygon
import numpy as np

# System Configuration
# --------------------

# define simulation parameters here
# todo: abstract out into config file


def mk_spiky_circle(n, r):
    d = 2*np.pi/n
    theta = 0
    pts = []
    for i in range(n):
        pt1 = [r*np.cos(theta), r*np.sin(theta)]
        th2 = theta + 0.05*d
        r2 = 1.5*r
        pt2 = [r2*np.cos(th2), r2*np.sin(th2)]
        theta += d
        pts.extend([pt1, pt2])
    return pts



L = 2.0
N = 50
T = 100
R = 0.02
border_region = R
# Define square cell
#cell = Cell(side=[L,L])
cell = Simple_Polygon("square",np.array([[0.0,0.0], [L, 0.0],[L,L],[0.0,L]]))
#cell = Simple_Polygon("spikes",np.array(mk_spiky_circle(8, 0.5*L)))
simname = cell.name+'_'+str(L)+"_N"+str(N)+"_T"+str(T)

A_properties = {'vel':0.5, 'wall_prob': 0.05, 'beta': 1.0}
B_properties = {'vel':0.05, 'wall_prob': 0.2, 'beta': 0.5}
properties = { 'A-free':A_properties
             ,'B-free':B_properties
             ,'A-wall':A_properties
             ,'B-wall':B_properties
             }
