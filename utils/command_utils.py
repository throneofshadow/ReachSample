""" Functions intended to help replicate robot commands, create new robot commands for different experimental paradigms,
    and to visualize robot commands within the robot workspace. For use with the ReachSample software. Written B Nelson
    7/19/22, UC Berkeley"""
import pandas as pd
import numpy as np


# Public functions


def read_command_file(fname):
    positions = pd.read_csv(fname)
    x_3, y_3, z_3 = [], [], []
    for index, row in positions.iterrows():
        x, y, z = xform_coords_euclidean(row['r'], row['thetay'], row['thetay.1'])
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
    x = np.sqrt(r**2 - y**2)
    return x

def initialize_commands_pilot():
    zero_mm = 'data/0mm.txt'
    ten_mm = 'data/10mm.txt'
    twenty_mm = 'data/20mm.txt'
    thirty_mm = 'data/30mm.txt'
    forty_mm = 'data/40mm.txt'
    pilot_3d_positions = 'data/9pt_pidiv3_cone.txt'
    return np.vstack((zero_mm, ten_mm, twenty_mm, thirty_mm, forty_mm, pilot_3d_positions))