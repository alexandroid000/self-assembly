import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from copy import copy
from random import random

from atooms.system.particle import Particle
from atooms.system.cell import Cell
from atooms.system import System
from atooms.simulation import Simulation
from atooms.trajectory import TrajectoryXYZ

# using bounce-viz as a submodule for geometric utilities
import sys
sys.path.insert(0, "./bounce-viz/src/")
from helper.shoot_ray_helper import IsInPoly, ClosestPtAlongRay
from helper.geometry_helper import AngleBetween 
from simple_polygon import Simple_Polygon

# System Configuration
# --------------------

# define simulation parameters here
# todo: abstract out into config file

L = 2.0
N = 10
T = 100
R = 0.01
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

# Utility Functions
# -----------------

# http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
def closest_edge(pt, poly):
    [x0,y0] = pt
    vs = poly.complete_vertex_list
    n = len(vs)
    min_d = 100000000000
    closest_edge = -1
    for j in range(n):
        [x1, y1], [x2,y2] = vs[j], vs[(j+1) % n]
        d = abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / np.sqrt((x2-x1)**2 + (y2-y1)**2)
        if d < min_d:
            min_d = d
            closest_edge = j
    return min_d, closest_edge

def normalize(vector):
    norm = np.linalg.norm(vector)
    return vector/norm

# Simulation Backend
# ------------------

class WBallBackend(object):

    def __init__(self, system, delta=0.1, env = cell, br = border_region):
        self.system = system
        self.delta = delta
        self.env = env
        self.vs = self.env.complete_vertex_list
        self.n = len(self.vs)
        self.br = br

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
    def project_to_border_region(self, pt, dr):
        d, j = closest_edge(pt, self.env)
        [ex,ey] = self.vs[(j + 1) % self.n] - self.vs[j]
        normal_dir = normalize(np.array([-ey, ex])) # pointing into polygon
        return pt + (self.br - d) * normal_dir

    def take_step_boundary(self, particle):
        d, j = closest_edge(particle.position, self.env)
        p1, p2 = self.vs[j], self.vs[(j+1) % self.n]
        # wall following
        dir = normalize(p2-p1)
        particle.position += self.delta*d*dir

        if random() > properties[particle.species]['wall_prob']:
            particle.species = particle.species[0]+'-free'
            [vx, vy] = particle.velocity
            # rotate particle's velocity 45 degrees from wall
            particle.velocity = normalize([0.7*vx - 0.7*vy, 0.7*vx + 0.7*vy])

    def take_step(self, particle):
        # stochastic update to heading theta
        # right now, uniform - TODO: change to Gaussian
        xi_x = np.random.normal() # mean zero, standard deviation 1
        xi_y = np.random.normal()
        theta = np.arctan2(particle.velocity[1], particle.velocity[0])
        xi_theta = np.random.normal(loc=theta)
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
            particle.position = self.project_to_border_region(particle.position, dr)
            particle.species = particle.species[0]+'-wall'

    def run(self, steps):
        for i in range(steps):
            for p in self.system.particle:
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
    xys = [copy(p.position) for p in sim.system.particle]
    pos_db[sim.current_step] = xys

# initialize simulation
system = System()
backend = WBallBackend(system)
simulation = Simulation(backend)
# We will execute the callback every step
simulation.add(cbk, 1, db=pos_db)

# create N particles at random locations in the box
for i in range(N):
    x,y = L*random(), L*random()
    vel = np.array([random()-0.5, random()-0.5])
    norm = np.linalg.norm(vel)
    vel /= norm
    system.particle.append(Particle(position=[x,y], velocity=list(vel), radius = None, species= 'A-free'))

# run simulation for T steps
simulation.run(T-1)

print("ran sim for ",T,"steps")

# write data to file
with open(simname+'.xyz','w') as th:
    for i in range(T):
        xys = pos_db[i]
        for [x,y] in xys:
            th.write(str(x)+" "+str(y)+" ")
        th.write("\n")

print("wrote data to",simname+".xyz")
print("writing video to",simname+".mp4")

class Data:

    def __init__(self, db, start=0):
        self.db = db
        self.num = start

    def __iter__(self):
        return self

    def clean_system(self, xys):
        return np.transpose(np.array(xys))

    def __next__(self):
        xys = self.db[self.num]
        self.num += 1
        return self.clean_system(xys)

d = Data(pos_db)


fig = plt.figure()
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-0.2, L+0.2), ylim=(-0.2, L+0.2))

# particles holds the locations of the particles
particles, = ax.plot([], [], 'bo', ms=6)

# rect is the box edge
rect = plt.Rectangle([0,0],
                     L,
                     L,
                     ec='none', lw=2, fc='none')
ax.add_patch(rect)

def init():
    """initialize animation"""
    global rect
    particles.set_data([], [])
    rect.set_edgecolor('none')
    return particles, rect


def animate(i):
    """perform animation step"""
    global d, rect, dt, ax, fig
    xys = next(d)

    ms = int(fig.dpi * 0.02 * fig.get_figwidth()
             / np.diff(ax.get_xbound())[0])
    
    # update pieces of the animation
    rect.set_edgecolor('k')
    particles.set_data(xys[0], xys[1])
    particles.set_markersize(ms)
    return particles, rect

ani = animation.FuncAnimation(fig, animate, frames=T,
                              interval=10, blit=True, init_func=init)


# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
ani.save(simname+'.mp4', fps=15, extra_args=['-vcodec', 'libx264'])
