from simple_polygon import Simple_Polygon
from maps import *
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

def mk_regpoly_obstacle(n, r):
    d = 2*np.pi/n
    theta = 0.0
    pts = []
    for i in range(n):
        pt = [r*np.cos(theta), r*np.sin(theta)]
        theta += d
        pts.append(pt)
    return pts[::-1]


#cell = Cell(side=[L,L])
#square = Simple_Polygon("square",np.array([[0.0,0.0], [L, 0.0],[L,L],[0.0,L]]))
#square_hole = Simple_Polygon("sqh",simple_holes[0], simple_holes[1])
#spikes = Simple_Polygon("spikes",np.array(mk_spiky_circle(8, 0.5*L)))

spike_annulus = Simple_Polygon("spk_ring",
                               np.array(mk_spiky_circle(8, 0.5*L)),
                    [np.array(mk_regpoly_obstacle(4, 0.4*L))])

cell = spike_annulus

# Define square cell
simname = cell.name+'_'+str(L)+"_N"+str(N)+"_T"+str(T)

A_properties = {'vel':0.5, 'wall_prob': 0.05, 'beta': 1.0}
B_properties = {'vel':0.05, 'wall_prob': 0.2, 'beta': 0.5}
properties = { 'A-free':A_properties
             ,'B-free':B_properties
             ,'A-wall':A_properties
             ,'B-wall':B_properties
             }
