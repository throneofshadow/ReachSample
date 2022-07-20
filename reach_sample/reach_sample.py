""" Program intended to easily collect, visualize, and describe robot command structures using statistics. Robot commands
    are meant to be used within the robot workspace created by the ReachMaster software. """
import numpy as np


class ReachSample:
    def __init__(self, n_positions, n_trials, x_length, y_length, z_length, command_type='1D'):
        self.sampled_robot_commands = np.empty((3, n_positions, n_trials))

    def return_commands(self):
        return self.sampled_robot_commands
