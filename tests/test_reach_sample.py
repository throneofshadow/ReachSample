import unittest
import os
os.chdir('../')
from reach_sample import ReachSample as RS
RS_ = RS()


class TestReachSample(unittest.TestCase):

    def test_reach_sample_init(self):
        try:
            RS_.visualize_pilot_workspace(create_gif=False)
        except:
            self.fail("Encountered an unexpected exception.")

    def test_reach_sample_1d_workspaces(self):
        try:
            RS_.create_theta_workspace(0.4, 2, 50, 9, visualize=True)
        except:
            self.fail("Encountered an unexpected exception.")

    def test_reach_sample_phi_workspaces(self):
        try:
            RS_.create_phi_workspace(0.4, 2, 50, 9, visualize=True)
        except:
            self.fail("Encountered an unexpected exception.")

    def test_reach_sample_2d_workspace(self):
        try:
            RS_.create_2d_workspace(1, 1, 2, 150, 9, sample=True, visualize=True)
        except:
            self.fail("Encountered an unexpected exception.")

    def test_reach_sample_3d_workspace(self):
        try:
            RS_.create_3d_workspace(0.5, 0.4, 1, 2, 150, 9, sample=True, visualize=True)
        except:
            self.fail("Encountered an unexpected exception.")


if __name__ == "__main__":
    unittest.main()
