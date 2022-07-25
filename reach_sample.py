""" Program intended to easily collect, visualize, and describe robot command structures using statistics. Robot commands
    are meant to be used within the robot workspace created by the ReachMaster software. """
import numpy as np
import utils.command_utils as c_d
import pandas as pd


def create_robot_command(idi, file=None):
    """ Function to in-take the calculated x,y,z command positions within the reaching workspace and
        transform them into commands in the spherical robot workspace. The output of this function may be saved as a
        pandas DataFrame using the savefile option."""

    csv_ob = pd.DataFrame(idi, columns=['r', 'thetay', 'thetaz'])
    if file:
        csv_ob.to_csv(file)
    else:
        csv_ob.to_csv('new_export_commands.csv')


class ReachSample:
    def __init__(self):
        self.sampled_robot_commands, self.theta_commands, self.phi_commands, self.commands_2d = 0, [], [], []
        self.commands_3d = []
        self.initial_commands = c_d.initialize_commands_pilot()

    def create_new_commands(self, n_positions, n_trials, x_length, y_length, z_length, command_type='1D'):
        """ Method to create and export new commands from supported command types.
           :param n_positions: Number of positions in a single trial generation, float
           :param n_trials: Number of total trials to visualize and export, float
           :param x_length: Length sample in x_direction, float
           :param y_length: Length sample in y_direction, float
           :param z_length: Length sample in z_direction, float
           :param command_type: Type of command, string
           :return: Returns vector containing new commands.
        """
        self.sampled_robot_commands = np.empty((3, n_positions, n_trials))
        return self.sampled_robot_commands

    def visualize_pilot_workspace(self, create_gif=False):
        """ Visualization function for pilot commands in the ReachMaster system. """
        c_d.create_pilot_visualizations(self.initial_commands, make_gif_animation=create_gif)

    def create_theta_workspace(self, y_limit, radius, n_trials, n_positions, sample=False, visualize=False,
                               export=False,
                               animate=False, animate_filename=False, export_filename=False):
        """ Method to create a 1-D theta (y-plane) task workspace. This method relies on functions from
            utils directory to create, visualize, generalize with statistics, and export command files for
            a theta robot command position workspace. """
        self.theta_commands = c_d.sample_theta_commands(y_limit, radius, n_trials, n_positions, extrema=sample)
        if visualize:
            c_d.visualize_commands(self.theta_commands, sample=sample, animate=animate)
        if export:
            for i in n_trials:
                self.theta_commands = c_d.xform_coords_spherical(self.theta_commands[i, :, 0],
                                                                 self.theta_commands[i, :, 1],
                                                                 self.theta_commands[i, :, 2])
            create_robot_command(self.theta_commands, file=export_filename)

    def create_phi_workspace(self, x_limit, radius, n_trials, n_positions, sample=False, visualize=False, export=False,
                             animate=False, animate_filename=False, export_filename=False):
        """ Method to create a 1-D phi (z-plane) task workspace. This method relies on functions from
            utils directory to create, visualize, generalize with statistics, and export command files for
            a phi robot command position workspace. """
        self.phi_commands = c_d.sample_phi_commands(x_limit, radius, n_trials, n_positions, extrema=sample)
        if visualize:
            c_d.visualize_commands(self.phi_commands, sample=sample)
        if export:
            for i in n_trials:
                self.phi_commands = c_d.xform_coords_spherical(self.phi_commands[i, :, 0], self.phi_commands[i, :, 1],
                                                               self.phi_commands[i, :, 2])
            create_robot_command(self.phi_commands, file=export_filename)

    def create_2d_workspace(self, z_length, y_length, radius, n_trials, n_positions, extrema=True, sample=False,
                            visualize=False, export=False, animate=False, animate_filename=False,
                            export_filename=False):
        """ Method to create a 2-D theta-phi (y-z plane) task workspace. This method relies on functions from
            utils directory to create, visualize, generalize with statistics, and export command files for
            a theta-phi robot command position workspace. """
        self.commands_2d = c_d.get_2d_commands(z_length, y_length, radius, n_positions, n_trials, sample=sample,
                                               extrema=extrema)
        if visualize:
            c_d.visualize_commands(self.commands_2d, sample=sample, animate=animate, animate_filename=animate_filename)
        if export:
            for i in n_trials:
                self.commands_2d = c_d.xform_coords_spherical(self.commands_2d[i, :, 0], self.commands_2d[i, :, 1],
                                                              self.commands_2d[i, :, 2])
            create_robot_command(self.commands_2d, file=export_filename)

    def create_3d_workspace(self, z_length, y_length, x_length, radius, n_trials, n_positions, extrema=True, sample=False,
                            visualize=False, export=False, animate=False, animate_filename=False,
                            export_filename=False):
        """ Method to create a 3-D theta-phi (y-z plane) task workspace. This method relies on functions from
            utils directory to create, visualize, generalize with statistics, and export command files for
            a theta-phi robot command position workspace. This workspace is then randomly sampled from either
            +x_length, -x_length, or kept at the originating 2-D x_length, allowing a researcher to randomly
            sample from the 3-D workspace while keeping as much resembling structure as possible. """
        self.commands_3d = c_d.sample_3d_structure(x_length, y_length, z_length, radius, n_positions, n_trials,
                                                   sample=sample, extrema=extrema)
        if visualize:
            c_d.visualize_commands(self.commands_3d, sample=sample, animate=animate, animate_filename = animate_filename)
        if export:
            for i in n_trials:
                self.commands_3d = c_d.xform_coords_spherical(self.commands_3d[i, :, 0], self.commands_3d[i, :, 1],
                                                              self.commands_3d[i, :, 2])
            create_robot_command(self.commands_3d, file=export_filename)
