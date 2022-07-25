import unittest
import numpy as np
import os
os.chdir('../')
from reach_sample import ReachSample as RS
RS_ = RS()


class TestReachSample(unittest.TestCase):

    def test_reach_sample_init(self):
        assert RS_.visualize_pilot_workspace(create_gif=False)

    def test_reach_sample_1d_workspaces(self):
        RS_.create_theta_workspace(0.4, 2, 50, 9, visualize=True)
        self.assertEquals(RS_.theta_commands.shape, np.asarray([50, 9, 3]), 'Robot theta command shape. ')

    def test_reach_sample_phi_workspaces(self):
        RS_.create_phi_workspace(0.4, 2, 50, 9, visualize=True)
        self.assertEquals(RS_.phi_commands.shape, np.asarray([50, 9, 3]), 'Robot phi command shape. ')

    def test_reach_sample_2d_workspace(self):
        RS_.create_2d_workspace(1, 1, 2, 150, 9, sample=True, visualize=True)
        self.assertEquals(RS_.commands_2d.shape, np.asarray([150, 9, 3]), 'Robot 2-D command shape. ')

    def test_reach_sample_3d_workspace(self):
        RS_.create_3d_workspace(0.5, 0.4, 1, 2, 150, 9, sample=True, visualize=True)
        self.assertEquals(RS_.commands_3d.shape, np.asarray([150, 9, 3]), 'Robot 3-D command shape. ')


if __name__ == "__main__":
    unittest.main()
