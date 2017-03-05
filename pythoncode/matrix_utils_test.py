import unittest
import matrix_utils
import numpy as np


class matrix_utilsTests(unittest.TestCase):
    def test_build(self):
        self.assertEqual(1, 1)

    def test_matrix_vector_multipl_4x4_4x1(self):
        a = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [1, 2, 3, 4]])
        b = np.array([1, 2, 3, 4])
        result = np.array([1+4+9+16, 5+12+21+32, 9+20+33+48,  1+4+9+16])
        result2 = matrix_utils.matrix_vector_multipl(a, b)
        np.testing.assert_array_equal(result2, result)

    def test_translation_matrix(self):
        translation_matrix = np.array([[1, 0, 0, 11], [0, 1, 0, 12],
                                       [0, 0, 1, 13], [0, 0, 0, 1]])
        np.testing.assert_array_equal(translation_matrix, matrix_utils.generate_translation_mx(11, 12, 13))

    def test_cross_product(self):
        test_vec1 = [1, 2, 3]
        test_vec2 = [4, 5, 6]
        test_result = matrix_utils.cross_product(test_vec1, test_vec2)
        print(test_result)
        self.assertEqual(test_result[0], -3)
        self.assertEqual(test_result[1], -6)
        self.assertEqual(test_result[2], -3)

    def test_matrix_vector_multipl_4x4_4x4(self):
        a = np.array([[1, 2, 3, 4], [5, 6, 7, 8],
                     [9, 10, 11, 12], [1, 2, 3, 4]])
        b = np.array([[1, 2, 3, 4], [5, 6, 7, 8],
                     [9, 10, 11, 12], [1, 2, 3, 4]])

        mult = np.array([[42, 52, 62, 72], [106, 132, 158, 184],
                        [170, 212, 254, 296], [42, 52, 62, 72]])

        np.testing.assert_array_equal(mult, matrix_utils.matrix_matrix_multipl(a, b))

    def test_inverse_matrix(self):
        a = np.array([[1, 2, 1, 1], [1, 0, 1, 1],
                      [1, 0, 0, 0], [4, 0, 1, 2]])
        result = np.array([[0, 0, 1, 0], [0.5, -0.5, 0, 0],
                      [0, 2, 2, -1], [0, -1, -3, 1]])
        result_computed = []
        result_computed = matrix_utils.inverse_matrix(a)

        np.testing.assert_array_equal(result, result_computed)
