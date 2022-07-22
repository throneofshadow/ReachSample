""" Functions intended to help replicate robot commands, create new robot commands for different experimental paradigms,
    and to visualize robot commands within the robot workspace. For use with the ReachSample software. Written B Nelson
    7/19/22, UC Berkeley"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
# Must have ffmpeg installed



# Public functions


def read_command_file(fname):
    positions = pd.read_csv(fname)
    x_3, y_3, z_3 = [], [], []
    for index, row in positions.iterrows():
        x, y, z = xform_coords_euclidean(row['r'], row['thetay'], row['thetaz'])
        x_3.append(x)
        y_3.append(y)
        z_3.append(z)
    fpositions = np.vstack((x_3, y_3, z_3))
    return fpositions


def xform_coords_euclidean(r, theta, phi):
    x = r * np.cos(phi * (np.pi / 180.)) * np.cos(theta * (np.pi / 180.))
    y = r * np.sin(theta * (np.pi / 180.)) * np.cos(phi * (np.pi / 180.))
    z = r * np.sin(phi * (np.pi / 180.))
    x = .1 * x  # cm
    y = .1 * y  # cm
    z = .1 * z  # cm
    return x, y, z


def xform_coords_spherical(x, y, z):
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)  # path length
    theta = np.arccos(z / r)  # degrees
    phi = np.arctan(y / x)  # degrees
    return r, theta, phi


def euclidean_distance_from_reaching_start(x, y, z):
    x0, y0, z0 = 0, 0, 0
    distance = np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
    return distance


def circle_variablerad_xdim(y, r):
    """ Function to create hyperbola line, focus at 10cm in reaching space. Given a set of x-coordinates, generate y
        for a given radius r. Phi (elevation) is given as 90 degrees or pi/4, allowing points to be level in the plane. """
    x = np.sqrt(r ** 2 - y ** 2)
    return x


def initialize_commands_pilot():
    """ This function loads in initial pilot data command positions, returning a vector size
        Experiment Type, n trials, 3 (n dimensions). """
    zero_mm = read_command_file('data/0mm.txt')[:, 0:9]
    ten_mm = read_command_file('data/10mm.txt')[:, 0:9]
    twenty_mm = read_command_file('data/20mm.txt')[:, 0:9]
    thirty_mm = read_command_file('data/30mm.txt')[:, 0:9]
    forty_mm = read_command_file('data/40mm.txt')[:, 0:9]
    pilot_3d_positions = read_command_file('data/9pt_pidiv3_cone.txt')
    return np.vstack((zero_mm, ten_mm, twenty_mm, thirty_mm, forty_mm, pilot_3d_positions))


def make_plot_pilot(fig, ax, pilot_command_positions):
    """ Plotting function, typeset for pilot_command_positions. """
    ax.scatter(pilot_command_positions[5, :, 0], pilot_command_positions[5, :, 1], pilot_command_positions[5, :, 2],
               color='r', label='3D Cone')
    ax.scatter(pilot_command_positions[1, :, 0], pilot_command_positions[1, :, 1], pilot_command_positions[1, :, 2],
               color='g', label='10mm Out')
    ax.scatter(pilot_command_positions[4, :, 0], pilot_command_positions[4, :, 1], pilot_command_positions[4, :, 2],
               color='g', label='40mm Out')
    ax.scatter(pilot_command_positions[0, :, 0], pilot_command_positions[0, :, 1], pilot_command_positions[0, :, 2],
               color='b', label='Initial Reaching Target')
    ax.scatter(pilot_command_positions[2, :, 0], pilot_command_positions[2, :, 1], pilot_command_positions[2, :, 2],
               color='y', s=55, label='Origin')
    ax.set_zlabel('Z (cm)')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    plt.legend()
    return fig,


def animate(ax, fig, i):
    """ Function that adjusts animation's elevation and azimuth. """
    ax.view_init(elev=15., azim=i)
    return fig,


def create_pilot_visualizations(pilot_command_positions, make_gif_animation=True):
    """ This function visualizes the 3-D workspace of ReachMaster's pilot experiments. """
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1, projection='3d', label='Reaching Volume Projection')
    ax.view_init(10, 80)
    if make_gif_animation:
        anim = animation.FuncAnimation(fig, animate, init_func=make_plot_pilot(fig,ax, pilot_command_positions),
                                       frames=360, interval=20, blit=True)
        anim.save('visualizations/pilot_animations.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    return
