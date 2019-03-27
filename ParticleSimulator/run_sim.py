from atooms.system.particle import Particle
from atooms.system.cell import Cell
from atooms.system import System
from atooms.simulation import Simulation
from atooms.trajectory import TrajectoryXYZ

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation

from backend import *
from configuration import *

# initialize simulation
system = System()
data = {"pos":[[]]*T, "env":[[]]*T}
be = WBallBackend(system, data, env, br = border_region,
                  sticky=allow_attachment, wires=wires)
simulation = Simulation(be)

# create N particles at random locations in the box
for i in range(N):
#    [x,y] = normalize([1.0, 1.0])
    [x,y] = 0.5*L*normalize([random()-0.5,random()-0.5])
#    [x,y] = 0.5*L*normalize([np.cos(3*np.pi/8),np.sin(3*np.pi/8)])
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
        xys = be.db["pos"][i]
        for (_, [x,y]) in xys:
            th.write(str(x)+" "+str(y)+" ")
        th.write("\n")

print("wrote data to",simname+".xyz")
print("writing video to",simname+".mp4")

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

d = Data(be.db)

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

particles = np.zeros(N, dtype=[('position', float, 2),
                               ('size',     float, 1),
                               ('color',    float, 3)])

initxy, initenv = copy(next(d))

particles['position'] = initxy[1]
particles['color'] = [color_map[t] for t in initxy[0]]
particles['size'] = [size_map[t] for t in initxy[0]]

fig = plt.figure()
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-L-0.2, L+0.2), ylim=(-L-0.2, L+0.2))

scat = ax.scatter(particles['position'][:,0]
                , particles['position'][:,1]
                , facecolors=particles['color']
                , s=particles['size']
                )

patches = []
for poly in initenv:
    p = Polygon(poly, ec='k', lw=2, fc='none')
    patches.append(ax.add_patch(p))

def init():
    """initialize animation"""
    global scat, patches
    return patches+[scat]

def animate(i):
    """perform animation step"""
    global scat, patches
    xy, polys = next(d)

    for j in range(1,len(patches)):
        patches[j].set_xy(polys[j])
        patches[j].set_edgecolor('k')

    # update pieces of the animation
    scat.set_facecolors([color_map[t] for t in xy[0]])
    scat.set_sizes([size_map[t] for t in xy[0]])
    scat.set_offsets(xy[1])
    return patches+[scat]

ani = animation.FuncAnimation(fig, animate, frames=T-2, interval=10,
                              blit=True, init_func=init)


# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
ani.save(simname+'.mp4', fps=15, extra_args=['-vcodec', 'libx264'])
