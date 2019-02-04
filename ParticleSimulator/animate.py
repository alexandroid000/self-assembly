import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from atooms.trajectory import TrajectoryXYZ

partXYs = [[],[]]

class Data:

    def __init__(self, start=0, fname='test.xyz'):
        self.th = open(fname, 'r')
        self.num = start

    def __iter__(self):
        return self

    def clean_system(self, sys):
        pos = np.array([p.position for p in sys.particle])
        return np.transpose(pos)

    def __next__(self):
        sys = self.th[self.num]
        self.num += 1
        return self.clean_system(sys)

d = Data()

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

ani = animation.FuncAnimation(fig, animate, frames=50,
                              interval=10, blit=True, init_func=init)


# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
ani.save('particle_box.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
