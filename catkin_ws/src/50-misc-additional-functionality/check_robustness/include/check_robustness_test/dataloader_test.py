import check_robustness

import unittest
import os

import cv2


class TestDataLoader(unittest.TestCase):

    def test_no_gt(self):
        i = 0
        for _ in check_robustness.DataLoader(path=os.path.join(os.path.dirname(__file__), "data"),
                                             has_ground_truth=False):
            i += 1

        self.assertEqual(i, 3)

    def test_with_gt(self):
        for img, label in check_robustness.DataLoader(path=os.path.join(os.path.dirname(__file__), "data"),
                                                      has_ground_truth=True):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_numbers = map(float, img.flatten())
            self.assertEqual(img_numbers, label)
