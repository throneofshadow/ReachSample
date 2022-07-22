""" Program intended to easily collect, visualize, and describe robot command structures using statistics. Robot commands
    are meant to be used within the robot workspace created by the ReachMaster software. """
import numpy as np
import utils.command_utils as c_d

class ReachSample:
    def __init__(self):
    # Define class variables
        self.sampled_robot_commands = 0
        self.initial_commands = c_d.initialize_commands_pilot()



    def create_new_commands(self, n_positions, n_trials, x_length, y_length, z_length, command_type = '1D'):
        self.sampled_robot_commands = np.empty((3, n_positions, n_trials))

    def return_commands(self):
        return self.sampled_robot_commands

    def visualize_pilot_workspace(self, create_gif=False):
        c_d.create_pilot_visualizations(make_gif_animation=create_gif)

