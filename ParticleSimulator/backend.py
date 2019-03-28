import numpy as np
from copy import copy, deepcopy
from random import random


# using bounce-viz as a submodule for geometric utilities
import sys
sys.path.insert(0, "./bounce-viz/src/")
from helper.shoot_ray_helper import IsInPoly, ClosestPtAlongRay
from helper.geometry_helper import AngleBetween
from utilities import *
from configuration import *


# Simulation Backend
# ------------------

class ParticlePhysics(object):

    def __init__(self, system, env, delta=0.05,
                       br = 0.01, sticky = True,
                       wires = []):
        self.system = system
        self.env = env
        self.delta = delta
        self.vs = self.env.vertex_list_per_poly
        self.n = len(self.vs[0]) # outer boundary
        self.br = br
        self.sticky = sticky
        self.wires = wires

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

    # move obstacle #c in the direction of dir
    # currently only internal obstacles can move, boundary is fixed
    def move_obstacle(self, c, dir):
        old_poly = [[v for (i,v) in c] for c in deepcopy(self.env.vertex_list_per_poly)]
        new_poly = [(v + 0.1*dir) for v in old_poly[c]]
        new_obstacles = old_poly[:c]+[new_poly]+old_poly[(c+1):]
        print("moved obstacle",c,"along vector",dir)
        print(len(new_obstacles),"new obstacles:",new_obstacles)
        new_env = Simple_Polygon(self.env.name, new_obstacles[0], new_obstacles[1:])
        self.env = new_env

    def obstacle_interaction(self, particle, edge_dir, d):
        d, c, j = closest_edge(particle.position, self.env)
        if c != 0:
            dr = self.next_dr(particle)

            particle.position += self.delta*d*normalize(edge_dir)
            [ex,ey] = edge_dir
            push_dir = normalize(np.array([ey, -ex])) # pointing into obstacle
            self.move_obstacle(c, push_dir)

    def scatter(self, particle, edge_dir):
        particle.species = particle.species[0]+'-free'
        [ex,ey] = edge_dir
        normal = normalize(np.array([-ey, ex])) # pointing into polygon

        # rotate particle's velocity uniformly out from wall
        th_out = np.pi*random() - np.pi/2
        particle.velocity = normalize(rotate_vector(normal, th_out))

    def take_step_boundary(self, particle):
        d, edge_dir = dist_dir_closest_edge(particle.position, self.env)

        # escape from wall
        if random() > properties[particle.species]['wall_prob']:
            self.scatter(particle, edge_dir)

        # stay on wall, impart force
        else:
            self.obstacle_interaction(particle, edge_dir, d)

    def next_dr(self, particle):

        # Brownian motion, random step on unit circle
        [xi_x, xi_y] = normalize([random()-0.5, random()-0.5])

        # compute direction of next step
        v = properties[particle.species]['vel']
        xdot = v*particle.velocity[0] + xi_x
        ydot = v*particle.velocity[1] + xi_y

        # random update to velocity heading
        # Gaussian, mean at current heading, standard deviation 1 radian
        theta = np.arctan2(particle.velocity[1], particle.velocity[0])
        xi_theta = np.random.normal(loc=theta)

        # update velocity for next step
        beta = properties[particle.species]['beta']
        if self.wires != []: 
            particle.velocity = force_from_wires(self.wires, particle.position)
        particle.velocity[0] += beta*np.cos(xi_theta)
        particle.velocity[1] += beta*np.sin(xi_theta)
        particle.velocity = normalize(particle.velocity)

        # take step, scaled by delta
        dr = self.delta*np.array([xdot, ydot])
        return dr

    def take_step(self, particle):
        dr = self.next_dr(particle)
        if IsInPoly(particle.position + dr, self.env):
            particle.position += dr
        else:
            particle.position = self.project_to_border_region(particle.position)
            particle.species = particle.species[0]+'-wall'

class ParticleSim(ParticlePhysics):

    def __init__(self, system, database, env, delta=0.05,
                       br = 0.01, sticky = True, wires = [],
                       regions = [], policy = []):

        ParticlePhysics.__init__(self, system, env, delta, br, sticky, wires)

        self.system = system
        self.db = database
        self.delta = delta
        self.env = env
        self.vs = self.env.vertex_list_per_poly
        self.n = len(self.vs[0]) # outer boundary
        self.br = br
        self.sticky = sticky
        self.wires = wires
        self.regions = regions
        self.policy = policy

    def run(self, steps):

        # initialize region counts
        region_counts = [0]*len(self.regions)
        for p in self.system.particle:
            for i,r in enumerate(self.regions):
                if IsInPoly(p.position, r):
                    region_counts[i] += 1

        # run sim for T steps
        for i in range(steps):

            # log regions; only works at beginning of loop for some reason
            self.log_data(i, region_counts)
            region_counts = [0]*len(self.regions)
            for p in self.system.particle:
                for i,r in enumerate(self.regions):
                    if IsInPoly(p.position, r):
                        region_counts[i] += 1

                # detect particle-particle collisions
                if self.sticky:
                    val, ns = self.checkAttach(p)
                    if val:
                        p.radius = R*len(ns)
                        mode = p.species[2:]
                        p.species = 'B-'+mode
                        for n in ns:
                            self.system.particle.remove(n)

                # boundary mode
                if p.species[2:] == "wall":
                    self.take_step_boundary(p)
                # free space mode
                else:
                    self.take_step(p)


    def log_data(self, step, r_counts):
        xys = [(copy(p.species), copy(p.position)) for p in self.system.particle]
        envs = [[v for (i,v) in c] for c in deepcopy(self.env.vertex_list_per_poly)]
        rs = copy(r_counts)
        self.db["pos"][step] = xys
        self.db["env"][step] = envs
        self.db["counts"][step] = rs

