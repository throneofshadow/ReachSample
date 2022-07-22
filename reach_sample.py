""" Program intended to easily collect, visualize, and describe robot command structures using statistics. Robot commands
    are meant to be used within the robot workspace created by the ReachMaster software. """
import numpy as np
import utils.command_utils as c_d
import pandas as pd


class ReachSample:
    def __init__(self):
        self.sampled_robot_commands, self.theta_commands, self.phi_commands = 0, [], []
        self.initial_commands = c_d.initialize_commands_pilot()

    def create_new_commands(self, n_positions, n_trials, x_length, y_length, z_length, command_type='1D'):
        self.sampled_robot_commands = np.empty((3, n_positions, n_trials))

    def return_commands(self):
        return self.sampled_robot_commands

    def visualize_pilot_workspace(self, create_gif=False):
        """ Visualization function for pilot commands in the ReachMaster system. """
        c_d.create_pilot_visualizations(self.initial_commands, make_gif_animation=create_gif)

    def create_theta_workspace(self, y_limit, radius, n_trials, n_positions, sample=False, visualize=False, export=False):
        """ Pass pilot positions object into the visualize commands function to plot pilot data along-side. """
        self.theta_commands = c_d.sample_theta_commands(y_limit, radius, n_trials, n_positions, extrema=sample)
        if visualize:
            c_d.visualize_commands(self.theta_commands)
        if export:
            for i in n_trials:
                self.theta_commands = c_d.xform_coords_spherical(self.theta_commands[i, :, 0], self.theta_commands[i, :, 1],
                                                                 self.theta_commands[i, :, 2])
            self.create_robot_command(self.theta_commands)

    def create_robot_command(self, id):
        """ Function to in-take the calculated x,y,z command positions within the reaching workspace and
            transform them into commands in the spherical robot workspace. The output of this function may be saved as a
            pandas DataFrame using the savefile option."""
        csv_ob = pd.DataFrame(id, )

