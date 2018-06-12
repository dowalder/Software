import unittest

import numpy as np

import check_robustness.transforms


class Translation(unittest.TestCase):

    def setUp(self):
        self.img = np.array([
            [1, 2, 3, 4, 5],
            [6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15],
            [16, 17, 18, 19, 20],
            [21, 22, 23, 24, 25]
        ])

    def test_horizontal_translation(self):
        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=0.2,
                                                         vertical_percentage=0.0)
        expected = np.array([
            [0, 1, 2, 3, 4],
            [0, 6, 7, 8, 9],
            [0, 11, 12, 13, 14],
            [0, 16, 17, 18, 19],
            [0, 21, 22, 23, 24]
        ])

        self.assertTrue(np.array_equal(expected, output))

        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=0.4,
                                                         vertical_percentage=0.0)
        expected = np.array([
            [0, 0, 1, 2, 3],
            [0, 0, 6, 7, 8],
            [0, 0, 11, 12, 13],
            [0, 0, 16, 17, 18],
            [0, 0, 21, 22, 23]
        ])

        self.assertTrue(np.array_equal(expected, output))

    def test_vertical_translation(self):
        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=0.0,
                                                         vertical_percentage=0.2)
        expected = np.array([
            [0, 0, 0, 0, 0],
            [1, 2, 3, 4, 5],
            [6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15],
            [16, 17, 18, 19, 20]
        ])

        self.assertTrue(np.array_equal(expected, output))

        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=0.0,
                                                         vertical_percentage=0.4)
        expected = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [1, 2, 3, 4, 5],
            [6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15]
        ])

        self.assertTrue(np.array_equal(expected, output))

    def test_both_translations(self):
        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=0.2,
                                                         vertical_percentage=0.2)
        expected = np.array([
            [0, 0, 0, 0, 0],
            [0, 1, 2, 3, 4],
            [0, 6, 7, 8, 9],
            [0, 11, 12, 13, 14],
            [0, 16, 17, 18, 19]
        ])

        self.assertTrue(np.array_equal(expected, output))

    def test_negative_translation(self):
        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=-0.2,
                                                         vertical_percentage=-0.2)
        expected = np.array([
            [7, 8, 9, 10, 0],
            [12, 13, 14, 15, 0],
            [17, 18, 19, 20, 0],
            [22, 23, 24, 25, 0],
            [0, 0, 0, 0, 0]
        ])

        self.assertTrue(np.array_equal(expected, output))

        output = check_robustness.transforms.translation(img=self.img, horizontal_percentage=-0.2,
                                                         vertical_percentage=0.2)
        expected = np.array([
            [0, 0, 0, 0, 0],
            [2, 3, 4, 5, 0],
            [7, 8, 9, 10, 0],
            [12, 13, 14, 15, 0],
            [17, 18, 19, 20, 0],
        ])

        self.assertTrue(np.array_equal(expected, output))


class Rotation(unittest.TestCase):

    def setUp(self):
        self.img = np.array([
            [1, 2, 3, 4, 5],
            [6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15],
            [16, 17, 18, 19, 20],
            [21, 22, 23, 24, 25]
        ])

    def test_rotation(self):
        output = check_robustness.transforms.rotation(self.img, 45)
        expected = np.array([
            [0, 4, 9, 10, 0],
            [2, 8, 9, 14, 20],
            [7, 7, 13, 19, 19],
            [6, 12, 17, 18, 24],
            [0, 16, 17, 22, 0]
        ])
        self.assertTrue(np.array_equal(expected, output))

        output = check_robustness.transforms.rotation(self.img, 90)
        expected = np.array([
            [5, 10, 15, 20, 25],
            [4, 9, 14, 19, 24],
            [3, 8, 13, 18, 23],
            [2, 7, 12, 17, 22],
            [1, 6, 11, 16, 21]
        ])
        self.assertTrue(np.array_equal(expected, output))


class TranslationObjects(unittest.TestCase):

    def setUp(self):
        self.img = np.array([
            [1, 2, 3, 4, 5],
            [6, 7, 8, 9, 10],
            [11, 12, 13, 14, 15],
            [16, 17, 18, 19, 20],
            [21, 22, 23, 24, 25]
        ])

    def test_fixed(self):
        fixed_translation = check_robustness.transforms.FixedTranslation(horizontal_percentage=0.2,
                                                                         vertical_percentage=0.2)

        output = fixed_translation(img=self.img)
        expected = np.array([
            [0, 0, 0, 0, 0],
            [0, 1, 2, 3, 4],
            [0, 6, 7, 8, 9],
            [0, 11, 12, 13, 14],
            [0, 16, 17, 18, 19]
        ])

        self.assertTrue(np.array_equal(expected, output))

    def test_discrete(self):
        discrete_translation = check_robustness.transforms.DiscreteTranslation(param_sets=[(0.2, 0.2), (-0.2, -0.2)])

        expected1 = np.array([
            [7, 8, 9, 10, 0],
            [12, 13, 14, 15, 0],
            [17, 18, 19, 20, 0],
            [22, 23, 24, 25, 0],
            [0, 0, 0, 0, 0]
        ])

        expected2 = np.array([
            [0, 0, 0, 0, 0],
            [0, 1, 2, 3, 4],
            [0, 6, 7, 8, 9],
            [0, 11, 12, 13, 14],
            [0, 16, 17, 18, 19]
        ])

        for _ in range(10):
            output = discrete_translation(img=self.img)
            self.assertTrue(np.array_equal(expected1, output) or np.array_equal(expected2, output))



