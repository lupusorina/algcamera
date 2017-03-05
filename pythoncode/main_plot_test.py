import unittest
import main_plot
import numpy as np


class matrix_utilsTests(unittest.TestCase):
    def test_build(self):
        self.assertEqual(1, 1)

    def test_Plane(self):
        p1 = np.array([1,1,1,1])
        p2 = np.array([-1,1,0,1])
        p3 = np.array([2,0,3,1])
        n_result, d_result = main_plot.Plane(p1, p2, p3)
        n_correct = np.array([-1,3,2])
        d_correct = -4

        self.assertEqual(d_result, d_correct)
        np.testing.assert_array_equal(n_result, n_correct)