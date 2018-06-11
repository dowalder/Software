import unittest

import analyze_logs

# from comptests.registrar import comptest


class TestFunctions(unittest.TestCase):
    def test_assign_nearest_points(self):
        references = [1.0, 2.0, 3.0]
        points = [0.0, 0.0, 2.1, 2.6, 3.1, 4.5]
        expected_output = [0, 0, 1, 2, 2, 2]

        output = analyze_logs.assign_nearest_points(references, points)
        self.assertEqual(expected_output, output)
        # assert expected_output == output

        references = [1.0, 2.0, 3.0]
        points = [2.1, 2.6, 3.1, 4.5]
        expected_output = [1, 2, 2, 2]

        output = analyze_logs.assign_nearest_points(references, points)
        self.assertEqual(expected_output, output)
        # assert expected_output == output

        references = [1.0, 2.0, 3.0]
        points = [0.9, 1.0, 1.1, 2.0, 2.0, 2.1, 3.0]
        expected_output = [0, 0, 0, 1, 1, 1, 2]

        output = analyze_logs.assign_nearest_points(references, points)
        self.assertEqual(expected_output, output)
        # assert expected_output == output

    def test_average_correspondences(self):
        references = [1.0, 2.0, 3.0]
        points = [0.5, 1.2, 2.0, 2.1, 4.0, 5.0]
        correspondences = [0, 0, 1, 1, 2, 2]
        expected_output = [1.7 / 2.0, 4.1 / 2.0, 9.0 / 2.0]

        output = analyze_logs.average_correspondences(references=references, points=points, correspondences=correspondences)
        self.assertAlmostEqual(expected_output, output)
        # assert expected_output == output
