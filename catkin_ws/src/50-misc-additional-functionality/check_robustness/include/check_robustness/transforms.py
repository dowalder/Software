import random

import cv2
import numpy as np


def translation(img, horizontal_percentage=0.1, vertical_percentage=0.0, template=None):
    """
    Translate an image.

    :param img: np.ndarray -> original image
    :param horizontal_percentage: float -> range (-1.0,1.0), describes the amount of translation in horizontal direction
    :param vertical_percentage: float -> range (-1.0,1.0), describes the amount of translation in vertical direction
    :param template: np.ndarray -> The translated image is going to be laid on top of this template. Use if you do not
                                    want a black background.
    :return: translated image
    """
    assert img.ndim >= 2, "The image needs at least 2 dimensions, but got {}".format(img.ndim)
    assert abs(horizontal_percentage) < 1, "You are trying to get rid of the whole image."
    assert abs(vertical_percentage) < 1, "You are trying to get rid of the whole image."
    assert template.shape == img.shape if template is not None else True

    out = np.zeros(img.shape, img.dtype) if template is None else template.copy()
    height, width = img.shape[:2]

    hor_start = int(round(width * horizontal_percentage))
    ver_start = int(round(height * vertical_percentage))
    hor_end = - hor_start if hor_start != 0 else width
    ver_end = - ver_start if ver_start != 0 else height

    if hor_start < 0:
        if ver_start < 0:
            out[:ver_start, :hor_start] = img[ver_end:, hor_end:]
        else:
            out[ver_start:, :hor_start] = img[:ver_end, hor_end:]
    else:
        if ver_start < 0:
            out[:ver_start, hor_start:] = img[ver_end:, :hor_end]
        else:
            out[ver_start:, hor_start:] = img[:ver_end, :hor_end]

    return out


class FixedTranslation:
    """
    A callable object that performs the translation transformation.
    """

    def __init__(self, horizontal_percentage, vertical_percentage, template=None):
        self.hor_per = horizontal_percentage
        self.ver_per = vertical_percentage
        self.template = template

    def __call__(self, img):
        return translation(img,
                           horizontal_percentage=self.hor_per,
                           vertical_percentage=self.ver_per,
                           template=self.template)


class RandomTranslation:

    def __init__(self, horizontal_bound, vertical_bound, template=None):
        assert isinstance(horizontal_bound, tuple)
        assert isinstance(vertical_bound, tuple)
        assert len(horizontal_bound) == 2
        assert len(vertical_bound) == 2
        assert 0 <= horizontal_bound[0] <= horizontal_bound[1] < 1
        assert 0 <= vertical_bound[0] <= vertical_bound[1] < 1

        self.hor_bound = horizontal_bound
        self.ver_bound = vertical_bound
        self.template = template

        random.seed()

    def __call__(self, img):
        hor_percentage = random.uniform(self.hor_bound[0], self.hor_bound[1]) * random.choice([-1, 1])
        ver_percentage = random.uniform(self.ver_bound[0], self.ver_bound[1]) * random.choice([-1, 1])

        return translation(img,
                           vertical_percentage=ver_percentage,
                           horizontal_percentage=hor_percentage,
                           template=self.template)


class DiscreteTranslation:

    def __init__(self, param_sets, template=None):
        for param_set in param_sets:
            assert len(param_set) == 2
            assert -1 < param_set[0] < 1
            assert -1 < param_set[1] < 1

        self.param_sets = param_sets
        self.template = template

    def __call__(self, img):
        hor_percentage, ver_percentage = random.choice(self.param_sets)

        return translation(img,
                           horizontal_percentage=hor_percentage,
                           vertical_percentage=ver_percentage,
                           template=self.template)


def rotation(img, angle):
    """
    Rotates the image with the given angle around its center.

    :param img: np.ndarray -> image to be rotated
    :param angle: float -> angle in degree

    :return: rotated image
    """
    assert img.ndim >= 2, "The image needs at least 2 dimensions, but got {}".format(img.ndim)

    height, width = img.shape[:2]
    image_center = (width / 2, height / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    out = cv2.warpAffine(img, rot_mat, (width, height), flags=cv2.INTER_NEAREST)
    return out


class FixedRotation:
    """
    A callable object that performs the rotation transformation.
    """

    def __init__(self, angle):
        self.angle = angle

    def __call__(self, img):
        return rotation(img, angle=self.angle)


class RandomRotation:

    def __init__(self, lower, upper):
        assert isinstance(upper, (float, int))
        assert isinstance(lower, (float, int))
        assert -180 <= lower <= upper <= 180

        self.upper = upper
        self.lower = lower

    def __call__(self, img):
        angle = random.uniform(self.lower, self.upper)
        return rotation(img, angle)


class DiscreteRotation:

    def __init__(self, angle_set):
        for angle in angle_set:
            assert -180 <= angle <= 180

        self.angles = angle_set

    def __call__(self, img):
        angle = random.choice(self.angles)
        return rotation(img, angle)
