import numpy as np
from copy import copy
from random import random


# using bounce-viz as a submodule for geometric utilities
import sys
sys.path.insert(0, "./bounce-viz/src/")
from helper.shoot_ray_helper import IsInPoly, ClosestPtAlongRay
from helper.geometry_helper import AngleBetween 
from configuration import *

# Utility Functions
# -----------------

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

# Simulation Backend
# ------------------

class WBallBackend(object):

    def __init__(self, system, delta=0.1, env = cell, br = border_region, sticky = True):
        self.system = system
        self.delta = delta
        self.env = env
        self.vs = self.env.vertex_list_per_poly
        self.n = len(self.vs[0]) # outer boundary
        self.br = br
        self.sticky = sticky

    def neighbors(self, particle):
        [x,y] = particle.position
        neighbors = []
        bounding_box = Simple_Polygon("bb",np.array([[x-R,y-R]
                                                   , [x+R,y-R]
                                                   , [x+R,y+R]
                                                   , [x-R,y+R]]))
        for p in self.system.particle:
            if IsInPoly(p.position, bounding_box) and (p is not particle):
                neighbors.append(p)
        return neighbors

    def checkAttach(self, particle):
        ns = self.neighbors(particle)
        if ns == []:
            return False, []
        else:
            return True, ns

    # TODO: replace this with polygon offset calculator to make more robust for
    # nonconvex polygons
    # look into pyclipper library
    def project_to_border_region(self, pt):
        d, [ex,ey] = dist_dir_closest_edge(pt, self.env)
        normal_dir = normalize(np.array([-ey, ex])) # pointing into polygon
        return pt + (self.br - d) * normal_dir

    def take_step_boundary(self, particle):
        d, edge_dir = dist_dir_closest_edge(particle.position, self.env)

        if random() > properties[particle.species]['wall_prob']:
            particle.species = particle.species[0]+'-free'
            [ex,ey] = edge_dir
            [nx, ny] = normalize(np.array([-ey, ex])) # pointing into polygon
            [vx, vy] = particle.velocity
            # rotate particle's velocity uniformly out from wall
            th_out = np.pi*random() - np.pi/2
            particle.velocity = normalize([np.cos(th_out)*nx - np.sin(th_out)*ny,
                                           np.sin(th_out)*nx + np.cos(th_out)*ny])
        else:
            particle.position += self.delta*d*normalize(edge_dir)


    def take_step(self, particle):
        # stochastic update to heading theta
        # right now, uniform - TODO: change to Gaussian
        xi_x = np.random.normal(scale = L/10.) # mean zero, standard deviation L/10
        xi_y = np.random.normal(scale = L/10.)
        theta = np.arctan2(particle.velocity[1], particle.velocity[0])
        xi_theta = np.random.normal(loc=theta) # mean at current heading, sd 1
        # velocity
        v = properties[particle.species]['vel']
        xdot = v*particle.velocity[0] + xi_x
        ydot = v*particle.velocity[1] + xi_y

        beta = properties[particle.species]['beta']
        particle.velocity[0] += beta*np.cos(xi_theta)
        particle.velocity[1] += beta*np.sin(xi_theta)

        particle.velocity = normalize(particle.velocity)
        dr = self.delta*np.array([xdot, ydot])
        if IsInPoly(particle.position + dr, self.env):
            particle.position += dr
        else:
            particle.position = self.project_to_border_region(particle.position)
            particle.species = particle.species[0]+'-wall'

    def run(self, steps):
        for i in range(steps):
            for p in self.system.particle:
                if self.sticky:
                    val, ns = self.checkAttach(p)
                    if val:
                        print("detected collision between",p.position,"and",[n.position for n in ns])
                        p.radius = R*len(ns)
                        mode = p.species[2:]
                        p.species = 'B-'+mode
                        for n in ns:
                            self.system.particle.remove(n)
                        print("there are now",len(self.system.particle),"particles")

                if p.species[2:] == "wall":
                    self.take_step_boundary(p)
                else:
                    self.take_step(p)

# to log data while simulation is running, we create a callback function which
# copies state to a dictionary
pos_db = [[]]*T
def cbk(sim, db):
    xys = [(copy(p.species), copy(p.position)) for p in sim.system.particle]
    pos_db[sim.current_step] = xys

