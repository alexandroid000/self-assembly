from atooms.system.particle import Particle
from atooms.system.cell import Cell
from atooms.system import System
from atooms.simulation import Simulation
from atooms.trajectory import TrajectoryXYZ

from copy import copy

import numpy as np
from random import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

L = 2.0
N = 1000
T = 100
simname = "box_L"+str(L)+"_N"+str(N)+"_T"+str(T)

system = System()
# Define square cell
cell = Cell(side=[L,L])

# create N particles at random locations in the box
for i in range(N):
    x,y = L*random(), L*random()
    system.particle.append(Particle(position=[x,y], radius = None, species= 'A'))

# weaselball simulation backend
class WBallBackend(object):

    def __init__(self, system, delta=1.0):
        self.system = system
        self.delta = delta

    def run(self, steps):
        for i in range(steps):
            for p in self.system.particle:
                dr = np.array([random()-0.5, random()-0.5])
                dr *= self.delta
                p.position += dr


# to log data while simulation is running, we create a callback function which
# copies state to a dictionary
pos_db = [[]]*T
def cbk(sim, db):
    xys = [copy(p.position) for p in sim.system.particle]
    pos_db[sim.current_step] = xys

# initialize simulation
backend = WBallBackend(system)
simulation = Simulation(backend)
# We will execute the callback every step
simulation.add(cbk, 1, db=pos_db)

# run simulation for T steps
simulation.run(T-1)

# write data to file
with open(simname+'.xyz','w') as th:
    for i in range(T):
        xys = pos_db[i]
        for [x,y] in xys:
            th.write(str(x)+" "+str(y)+" ")
        th.write("\n")

# 
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
                     xlim=(-3.2, 3.2), ylim=(-2.4, 2.4))

# particles holds the locations of the particles
particles, = ax.plot([], [], 'bo', ms=6)

# rect is the box edge
rect = plt.Rectangle([-2,-2],
                     4,
                     4,
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

    ms = int(fig.dpi * 2 * 0.04 * fig.get_figwidth()
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
ani.save(simname+'.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
