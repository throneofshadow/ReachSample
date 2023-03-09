""" Functions intended to help replicate robot commands, create new robot commands for different experimental paradigms,
    and to visualize robot commands within the robot workspace. For use with the ReachSample software. Written B Nelson
    7/19/22, UC Berkeley"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import pdb

# Public functions


def read_command_file(filename):
    """ Function to read robot command files using pandas.
    """
    positions = pd.read_csv(filename)
    x_3, y_3, z_3 = [], [], []
    for index, row in positions.iterrows():
        x, y, z = xform_coords_euclidean(row['r'], row['thetay'], row['thetaz'])
        x_3.append(x)
        y_3.append(y)
        z_3.append(z)
    _positions = np.vstack((x_3, y_3, z_3))
    return _positions


def xform_coords_euclidean(r, theta, phi):
    """ Transforms spherical-based robot commands (r, theta, phi) into euclidean-based coordinates. Used for
        plotting workspaces in reach_sample.
    """
    x = r * np.cos(phi * (np.pi / 180.)) * np.cos(theta * (np.pi / 180.))
    y = r * np.sin(theta * (np.pi / 180.)) * np.cos(phi * (np.pi / 180.))
    z = r * np.sin(phi * (np.pi / 180.))
    x = .1 * x  # cm
    y = .1 * y  # cm
    z = .1 * z  # cm
    return x, y, z


def xform_coords_spherical(x, y, z):
    """ Transforms euclidean-based 3D coordinates to spherical coordinates (r, theta, phi). """
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)  # path length
    theta = np.radians(np.arccos(z / r))  # degrees
    phi = np.radians(np.arctan(y / x))  # degrees
    return r, theta, phi


def euclidean_distance_from_reaching_start(x, y, z):
    """ Determines the euclidean distance from the tentative center of reaching area.
    """
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
    return np.vstack((zero_mm, ten_mm, twenty_mm, thirty_mm, forty_mm, pilot_3d_positions)).reshape((6, 3, 9))


def get_2d_commands(z_length, y_length, radius, n_positions, n_trials, sample=False, extrema=True):
    """ Function that obtains trial-on-trial ReachMaster command positions for the 2-D spatial dimension of task.
        This function either a) structures points in a symmetric (about x) manner or b) uses randomization to subsample
        points within a given 2-D space. Sub-sampling may either include consistent extrema or none at all. """
    command_positions_2d = np.zeros((n_trials, n_positions, 3))
    if sample:
        for i in range(0, n_trials):  #
            if extrema:
                phi_command = obtain_single_phi_command(z_length, radius, n_positions, sample=True)
                theta_command = obtain_single_theta_command(y_length, radius, n_positions, sample=True)
            else:
                phi_command = obtain_single_phi_command(z_length, radius, n_positions, sample=True, extrema=False)
                theta_command = obtain_single_theta_command(y_length, radius, n_positions, sample=True, extrema=False)
            s = np.random.binomial(1, 0.5, 1)
            if s < 1:  # odd split theta, even split phi
                command_positions_2d[i, ::1, :] = theta_command[::1, :]
                command_positions_2d[i, ::2, :] = phi_command[::2, :]
            else:  # even split theta, odd split phi
                command_positions_2d[i, ::1, :] = phi_command[::1, :]
                command_positions_2d[i, ::2, :] = theta_command[::2, :]
    else:
        phi_command = obtain_single_phi_command(z_length, radius, n_positions)
        theta_command = obtain_single_phi_command(y_length, radius, n_positions)
        for i in range(0, n_trials):
            s = np.random.binomial(1, 0.5, 1)  # draw from binomial distribution
            if s < 1:  # odd split theta, even split phi
                command_positions_2d[i, ::1, :] = theta_command[::1, :]
                command_positions_2d[i, ::2, :] = phi_command[::2, :]
            else:  # even split theta, odd split phi
                command_positions_2d[i, ::1, :] = phi_command[::1, :]
                command_positions_2d[i, ::2, :] = theta_command[::2, :]
    return command_positions_2d


def sample_3d_structure(stride, y_length, z_length,  radius, n_positions, n_trials, sample=False, extrema=True):
    """ Function to sample 2-D positional commands in a stride-based symmetric manner. This function requires
        pre-defined x,y,z lengths (stride is the x-length), radius of circle around reaching position, the
        number of positions and number of trials to sample commands over. """
    commands_3d = np.zeros((n_trials, n_positions, 3))
    commands_2d = get_2d_commands(z_length, y_length, radius, n_positions, n_trials, sample=sample, extrema=extrema)
    for l in range(0, n_trials):
        for ir in range(0, n_positions):
            choose_axis = np.random.multinomial(1, [1.0 / 3, 1.0 / 3, 1.0 / 3], size=1)
            #pdb.set_trace()
            if choose_axis[0, 0] == 1:  # Take selected component stride in +x direction
                commands_3d[l, ir, 1] = commands_2d[l, ir, 1]
                commands_3d[l, ir, 2] = commands_2d[l, ir, 2]
                commands_3d[l, ir, 0] = commands_2d[l, ir, 0] + stride
            if choose_axis[0, 1] == 1:
                commands_3d[l, ir, 1] = commands_2d[l, ir, 1]
                commands_3d[l, ir, 2] = commands_2d[l, ir, 2]
                commands_3d[l, ir, :] = commands_2d[l, ir, :]
            if choose_axis[0, 2] == 1:
                commands_3d[l, ir, 1] = commands_2d[l, ir, 1]
                commands_3d[l, ir, 2] = commands_2d[l, ir, 2]
                commands_3d[l, ir, 0] = commands_2d[l, ir, 0] - stride
    return commands_3d


def rand_sample_circle(y_array, radius, n_positions=9):
    """ Randomly sample the x position for 1-D commands. """
    radius_array = np.repeat(radius, n_positions)
    x_position = circle_variablerad_xdim(y_array, radius_array)
    return x_position


def obtain_single_theta_command(length, radius, n_positions, sample=False, extrema=True):
    """ Obtain a command in the single theta dimension that either randomly selects from a set of points or randomly samples
        between defined ranges."""
    z_positions = np.zeros(n_positions)
    y_positions = np.zeros(n_positions)
    if sample:
        if extrema:
            y_positions[0] = length
            y_positions[n_positions - 1] = -1 * length
            y_positions[1:int((n_positions - 1) / 2)] = np.random.uniform(length - (length / 16), 0 + (length / 16), 3)
            y_positions[int((n_positions - 1) / 2) + 1:n_positions - 1] = np.random.uniform(0 - (length / 16),
                                                                                            -1 * length + (length / 16),
                                                                                            3)
        else:
            y_positions[0:int((n_positions - 1) / 2)] = np.random.uniform(length - (length / 16 * 2),
                                                                          0 + (length / 16 * 2), 4)
            y_positions[int((n_positions - 1) / 2) + 1:n_positions] = np.random.uniform(0 - (length / 16 * 2),
                                                                                        -1 * length + (length / 16 * 2),
                                                                                        4)
        x_positions = rand_sample_circle(y_positions, radius, n_positions)
    else:
        y_positions = np.linspace(length, -1 * length, n_positions)
        x_positions = rand_sample_circle(y_positions, radius,
                                         n_positions) + 0.2  # the origin offset in the x-direction.
    return np.vstack((x_positions, y_positions, z_positions)).T


def obtain_single_phi_command(length, radius, n_positions, sample=False, extrema=False):
    """ Function that inverts commands in the theta plane to rotate about the x-axis (z-plane). These commands
        allow a user to functionally interrogate the theta-z or phi plane in spherical coordinates with the robot
        handle position."""
    y_positions = np.zeros(n_positions)
    z_positions = np.zeros(n_positions)  # the origin offset in the x-direction.
    if sample:
        z_positions[0] = length
        z_positions[n_positions - 1] = -1 * length
        z_positions[1:int((n_positions - 1) / 2)] = np.random.uniform(length - (length / 16), 0, 3)
        z_positions[int((n_positions - 1) / 2) + 1:n_positions - 1] = np.random.uniform(-1 * length + (length / 16), 0, 3)
        x_positions = rand_sample_circle(z_positions, radius, n_positions)
        z_positions[int((n_positions - 1) / 2) + 1:n_positions - 1] = \
            z_positions[int((n_positions - 1) / 2) + 1:n_positions - 1]
        x_positions = x_positions
    else:
        x_positions = np.linspace(length, -1 * length, n_positions)   # the origin offset in the x-direction.
        z_positions = rand_sample_circle(x_positions, radius,
                                         n_positions)
    return np.vstack((x_positions, y_positions, z_positions)).T


def sample_theta_commands(length, radius, n_trials, n_positions, extrema=True):
    """ Get randomized position commands for theta (1-D) over n trials. Can leave the ends (extrema=True). """
    theta_commands = np.zeros((n_trials, n_positions, 3))
    for i in range(0, n_trials):
        theta_commands[i, :, :] = obtain_single_theta_command(length, radius, n_positions, sample=extrema)
    return theta_commands


def sample_phi_commands(length, radius, n_positions, n_trials, extrema=True):
    """ Get randomized position commands for phi (1-D) over n trials. Can leave the ends (extrema=True). """
    phi_commands = np.zeros((n_trials, n_positions, 3))
    for i in range(0, n_trials):
        phi_commands[i, :, :] = obtain_single_phi_command(length, radius, n_positions, sample=extrema)
    return phi_commands


def make_plot_pilot(fig, ax, pilot_command_positions):
    """ Plotting function, typeset for pilot_command_positions. """
    ax.scatter(pilot_command_positions[5, 0, :], pilot_command_positions[5, 1, :], pilot_command_positions[5, 2, :],
               color='r', label='3D Cone')
    ax.scatter(pilot_command_positions[1, 0, :], pilot_command_positions[1, 1, :], pilot_command_positions[1, 2, :],
               color='g', label='10mm Out')
    ax.scatter(pilot_command_positions[4, 0, :], pilot_command_positions[4, 1, :], pilot_command_positions[4, 2, :],
               color='g', label='40mm Out')
    ax.scatter(pilot_command_positions[0, 0, :], pilot_command_positions[0, 1, :], pilot_command_positions[0, 2, :],
               color='b', label='Initial Reaching Target')
    ax.scatter(pilot_command_positions[2, 0, :], pilot_command_positions[2, 1, :], pilot_command_positions[2, 2, :],
               color='y', s=55, label='Origin')
    plt.plot(np.zeros(30), np.linspace(-0.4, 0.4, 30), np.zeros(30), color='k', label='Enclosure Entrance')
    ax.set_zlabel('Z (cm)')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    plt.legend()
    return fig,


def animate_a(ax, fig, i):
    """ Function that adjusts animation's elevation and azimuth. """
    ax.view_init(elev=15., azim=i)
    return fig,


def create_pilot_visualizations(pilot_command_positions, make_gif_animation=True):
    """ This function visualizes the 3-D workspace of ReachMaster's pilot experiments. """
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1, projection='3d', label='Reaching Volume Projection')
    ax.view_init(10, 80)
    make_plot_pilot(fig, ax, pilot_command_positions)
    print('Visualization of pilot experiment workspace.')
    plt.show()
    if make_gif_animation:
        print('Creating animation for pilot experiment workspace. ')
        anim = animation.FuncAnimation(fig, animate_a, init_func=make_plot_pilot(fig, ax, pilot_command_positions),
                                       frames=360, interval=20, blit=True)
        anim.save('visualizations/pilot_animations.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    return


def func_viz(fig, ax, positions):
    """ Function to visualize command positions using the scatter command. Function scatters entire command vector."""
    ax.scatter(positions[:, :, 0], positions[:, :, 1], positions[:, :, 2], c='r', label='Positions')
    ax.scatter(0.2, 0, 0, color='y', s=55, label='Origin')
    plt.plot(np.zeros(30), np.linspace(-0.4, 0.4, 30), np.zeros(30), color='k', label='Enclosure Entrance')
    plt.legend()
    ax.set_zlabel('Z (cm)')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    return fig,


def func_viz_sample(fig, ax, positions):
    """ Function to visualize randomly sampled command positions using the scatter command. Function scatters entire
        command. """
    for ir in range(1, positions.shape[0] - 1):
        ax.scatter(positions[ir, :, 0], positions[ir, :, 1], positions[ir, :, 2])
    ax.scatter(positions[0, :, 0], positions[0, :, 1], positions[0, :, 2], label='Positions')
    ax.scatter(2, 0, 0, color='y', s=55, label='Origin')
    plt.plot(np.zeros(30), np.linspace(-0.4, 0.4, 30), np.zeros(30), color='k', label='Enclosure Entrance')
    plt.legend()
    ax.set_zlabel('Z (cm)')
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    return fig,


def visualize_commands(commands, sample=False, animate=False, animate_filename=False):
    """ Function to visualize incoming vector-based x,y, z commands. Function takes in vector size n_trials, n_positions, 3.
        Function outputs matlab-based visualization of positions within ReachMaster's 3-D workspace. """
    fig1 = plt.figure(figsize=(10, 10))
    ax1 = fig1.add_subplot(1, 1, 1, projection='3d', label='Reaching Volume Projection: Created Experiment')
    if sample:
        func_viz_sample(fig1, ax1, commands)
    else:
        func_viz(fig1, ax1, commands)
    plt.show()
    if animate:
        anim = animation.FuncAnimation(fig1, animate_a, init_func=func_viz(fig1, ax1, commands), frames=360,
                                       interval=20,
                                       blit=True)
        if animate_filename:
            anim.save(animate_filename, fps=30, extra_args=['-vcodec', 'libx264'])
        else:
            anim.save('visualizations/default_animations.mp4', fps=30, extra_args=['-vcodec', 'libx264'])


def histogram_command_files(commands, bin_num=25, density=False, comtype=None, save_file = None):
    """ Function to create multi-class histogram to examine x, y, and z positions for a given set of commands.
        These commands may be in any dimension. """
    x_commands = commands[:, :, 0].reshape(commands.shape[0]*commands.shape[1]) - 2  # x_offset.
    y_commands = commands[:, :, 1].reshape(commands.shape[0]*commands.shape[1])
    z_commands = commands[:, :, 2].reshape(commands.shape[0]*commands.shape[1])
    plt.hist(x_commands, bins=bin_num, density=density, color='r', histtype='barstacked', label='X Positions')
    plt.hist(y_commands, bins=bin_num, density=density, color='g', histtype='barstacked', label='Y Positions')
    plt.hist(z_commands, bins=bin_num, density=density, color='b', histtype='barstacked', label='Z Positions')
    plt.xlabel('Positions relative to origin (cm)')
    plt.ylabel('Counts')
    plt.legend()
    if comtype:
        plt.title('Command Positions: ' + str(comtype))
    else:
        plt.title('Command Positions. ')

    if save_file:
        plt.savefig(save_file, dpi=400)
    plt.show()



