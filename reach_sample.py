""" Program intended to easily collect, visualize, and describe robot command structures using statistics. Robot commands
    are meant to be used within the robot workspace created by the ReachMaster software. """
import numpy as np
import utils.command_utils as c_d
import pandas as pd


class ReachSample:
    def __init__(self):
        self.sampled_robot_commands, self.theta_commands, self.phi_commands, self.commands_2d = 0, [], [], []
        self.commands_3d = []
        self.initial_commands = c_d.initialize_commands_pilot()

    def create_new_commands(self, n_positions, n_trials, x_length, y_length, z_length, command_type='1D'):
        self.sampled_robot_commands = np.empty((3, n_positions, n_trials))

    def return_commands(self):
        return self.sampled_robot_commands

    def visualize_pilot_workspace(self, create_gif=False):
        """ Visualization function for pilot commands in the ReachMaster system. """
        c_d.create_pilot_visualizations(self.initial_commands, make_gif_animation=create_gif)

    def create_theta_workspace(self, y_limit, radius, n_trials, n_positions, sample=False, visualize=False,
                               export=False,
                               animate=False, animate_filename=False, export_filename=False):
        """  """
        self.theta_commands = c_d.sample_theta_commands(y_limit, radius, n_trials, n_positions, extrema=sample)
        if visualize:
            c_d.visualize_commands(self.theta_commands, sample=sample, animate=animate)
        if export:
            for i in n_trials:
                self.theta_commands = c_d.xform_coords_spherical(self.theta_commands[i, :, 0],
                                                                 self.theta_commands[i, :, 1],
                                                                 self.theta_commands[i, :, 2])
            self.create_robot_command(self.theta_commands, fname=export_filename)

    def create_phi_workspace(self, x_limit, radius, n_trials, n_positions, sample=False, visualize=False, export=False,
                             animate=False, animate_filename=False, export_filename=False):
        """  """
        self.phi_commands = c_d.sample_phi_commands(x_limit, radius, n_trials, n_positions, extrema=sample)
        if visualize:
            c_d.visualize_commands(self.phi_commands, sample=sample)
        if export:
            for i in n_trials:
                self.phi_commands = c_d.xform_coords_spherical(self.phi_commands[i, :, 0], self.phi_commands[i, :, 1],
                                                               self.phi_commands[i, :, 2])
            self.create_robot_command(self.phi_commands, fname=export_filename)

    def create_2d_workspace(self, z_length, y_length, radius, n_trials, n_positions, extrema=True, sample=False,
                            visualize=False, export=False, animate=False, animate_filename=False, export_filename=False):
        self.commands_2d = c_d.get_2d_commands(z_length, y_length, radius, n_positions, n_trials, sample=sample,
                                               extrema=extrema)
        if visualize:
            c_d.visualize_commands(self.commands_2d, sample=sample)
        if export:
            for i in n_trials:
                self.commands_2d = c_d.xform_coords_spherical(self.commands_2d[i, :, 0], self.commands_2d[i, :, 1],
                                                              self.commands_2d[i, :, 2])
            self.create_robot_command(self.commands_2d, fname=export_filename)

    def create_robot_command(self, idi, fname=False):
        """ Function to in-take the calculated x,y,z command positions within the reaching workspace and
            transform them into commands in the spherical robot workspace. The output of this function may be saved as a
            pandas DataFrame using the savefile option."""

        csv_ob = pd.DataFrame(idi, columns=['r', 'thetay', 'thetaz'])
        if fname:
            csv_ob.to_csv(fname)
        else:
            csv_ob.to_csv('new_export_commands.csv')
