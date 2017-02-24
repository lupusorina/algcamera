import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)


def generate_plot():
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_ylim3d(-24, 24)
    ax.set_zlim3d(-12.5, 24)
    ax.set_xlim3d(-24, 24)
    return ax


def add_to_plot(ax, a,b,c):
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)