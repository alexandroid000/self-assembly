from atooms.system.particle import Particle
from atooms.system.cell import Cell
from atooms.system import System
from atooms.simulation import Simulation

import csv

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
import matplotlib.animation as animation

from backend import *
from configuration import *
from utilities import normalize, Wire
from random import random

class Data:

    def __init__(self, db, start=0):
        self.xy = db["pos"]
        self.env = db["env"]
        self.num = start

    def __iter__(self):
        return self

    def clean_system(self, dat):
        types = [t for (t,xy) in dat]
        xys = [xy for (t,xy) in dat]
        return types, np.array(xys)

    def __next__(self):
        dat = self.xy[self.num]
        polys = [np.array(poly) for poly in self.env[self.num]]
        self.num += 1
        return self.clean_system(dat), polys


def write_data(database):
        
    # write position data to file
    with open(simname+'.xyz','w') as th:
        for i in range(T):
            xys = database["pos"][i]
            for (_, [x,y]) in xys:
                th.write(str(x)+" "+str(y)+" ")
            th.write("\n")

    print("wrote data to",simname+".xyz")

    # write region count data to file
    with open(simname+'_regions.csv','w') as th:
        wr = csv.writer(th, quoting=csv.QUOTE_ALL)
        rs = database["counts"][-1]
        wr.writerow(rs)

    print("wrote data to",simname+"_regions.csv")

color_map = { 'A-wall': (0, 0, 1)
            , 'A-free': (0, 0, 1)
            , 'B-wall': (0, 1, 0)
            , 'B-free': (0, 1, 0)
            }
size_map =  { 'A-wall': 10
            , 'A-free': 10
            , 'B-wall': 50
            , 'B-free': 50
            }




def init():
    """initialize animation"""
    global scat, patches, w_patches
    patches = []
    w_patches = []
    for poly in initenv:
        p = Polygon(poly, ec='k', lw=2, fc='none')
        patches.append(ax.add_patch(p))

    for w in wire_verts:
        c = Circle(w, 0.1, facecolor='r', hatch='x')
        w_patches.append(ax.add_patch(c))
    return w_patches+patches+[scat]

def animate(i):
    """perform animation step"""
    global scat, patches, w_patches
    xy, polys = next(d)

    for j in range(1,len(patches)):
        patches[j].set_xy(polys[j])
        patches[j].set_edgecolor('k')

    # update pieces of the animation
    scat.set_facecolors([color_map[t] for t in xy[0]])
    scat.set_sizes([size_map[t] for t in xy[0]])
    scat.set_offsets(xy[1])
    return w_patches+patches+[scat]

def mkAnimation():
    ani = animation.FuncAnimation(fig, animate, frames=T-2, interval=10,
                              blit=True, init_func=init)
    # save the animation as an mp4.  This requires ffmpeg or mencoder to be
    # installed.  The extra_args ensure that the x264 codec is used, so that
    # the video can be embedded in html5.  You may need to adjust this for
    # your system: for more information, see
    # http://matplotlib.sourceforge.net/api/animation_api.html
    ani.save(simname+'.mp4', fps=15, extra_args=['-vcodec', 'libx264'])


if __name__ == '__main__':

    args = sys.argv[1:]
    starting_poly = int(args[0])
    action = int(args[1])
    orientations = decode_policy(action)
    wires = [Wire(v, o) for v, o in zip(wire_verts, orientations)]

    # initialize simulation
    system = System()
    data = {"pos":[[]]*T, "env":[[]]*T, "counts":[[]]*(T-1)}
    be = ParticleSim(system, data, env, br = border_region,
                      sticky=allow_attachment, wires=wires,
                      regions=regions)
    simulation = Simulation(be)
    simname = env.name+"_N"+str(N)+"_T"+str(T)+"_R"+str(starting_poly)+"_A"+str(action)

    # create N particles at random locations in the polygon
    starting_poly = regions[starting_poly]
    start_pts = uniform_sample_from_poly(starting_poly, N)
    for i in range(N):
        vel = normalize(np.array([random()-0.5, random()-0.5]))
        system.particle.append(Particle(position=start_pts[i], velocity=list(vel), radius = None, species= 'A-free'))

    # run simulation for T steps
    simulation.run(T-1)
    print("ran sim for ",T,"steps")

    write_data(be.db)

    # make iterator to make animation easier
    d = Data(be.db)
    initxy, initenv = copy(next(d))

    colors = [color_map[t] for t in initxy[0]]
    sizes = [size_map[t] for t in initxy[0]]

    fig = plt.figure()
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                         xlim=(-L-0.2, L+0.2), ylim=(-L-0.2, L+0.2))

    scat = ax.scatter(initxy[1][:,0]
                    , initxy[1][:,1]
                    , facecolors = colors
                    , s = sizes
                    )


    print("writing video to",simname+".mp4")
    mkAnimation()


