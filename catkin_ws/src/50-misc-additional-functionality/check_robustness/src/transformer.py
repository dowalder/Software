#!/usr/bin/env python
import rospy
import cv_bridge
import sensor_msgs.msg
import enum

import check_robustness.transforms


class Transformation(enum.Enum):
    FIXED_TRANSLATION = 1
    RANDOM_TRANSLATION = 2
    DISCRETE_TRANSLATION = 3
    FIXED_ROTATION = 4
    RANDOM_ROTATION = 5
    DISCRETE_ROTATION = 6


def transformation_factory(transformation):
    assert isinstance(transformation, Transformation)

    if transformation == Transformation.FIXED_TRANSLATION:
        hor_ratio = rospy.get_param("~fixed_translation_hor_ratio")
        ver_ratio = rospy.get_param("~fixed_translatuion_ver_ratio")
        return check_robustness.transforms.FixedTranslation(horizontal_percentage=hor_ratio,
                                                            vertical_percentage=ver_ratio)
    elif transformation == Transformation.RANDOM_TRANSLATION:
        hor_bound = rospy.get_param("~fixed_translation_hor_bound")
        ver_bound = rospy.get_param("~fixed_translation_ver_bound")
        return check_robustness.transforms.RandomTranslation(horizontal_bound=hor_bound, vertical_bound=ver_bound)

    elif transformation == Transformation.DISCRETE_TRANSLATION:
        param_sets = rospy.get_param("~discrete_translation_param_sets")
        return check_robustness.transforms.DiscreteTranslation(param_sets=param_sets)

    elif transformation == Transformation.FIXED_ROTATION:
        angle = rospy.get_param("~fixed_rotation_angle")
        return check_robustness.transforms.FixedRotation(angle)

    elif transformation == Transformation.RANDOM_ROTATION:
        lower = rospy.get_param("~random_rotation_lower")
        upper = rospy.get_param("~random_rotation_upper")
        return check_robustness.transforms.RandomRotation(lower=lower, upper=upper)

    elif transformation == Transformation.DISCRETE_ROTATION:
        angle_sets = rospy.get_param("~discrete_rotation_angles")
        return check_robustness.transforms.DiscreteRotation(angle_set=angle_sets)

    else:
        raise RuntimeError("Unknown transformation: {}".format(transformation))



class Transformer:

    def __init__(self, transforms):
        if not isinstance(transforms, list):
            self.transforms = [transforms]
        else:
            self.transforms = transforms

        self.cvbridge = cv_bridge.CvBridge()

        self.pub_compressed = rospy.Publisher("~out_compressed", sensor_msgs.msg.CompressedImage)
        self.pub_raw = rospy.Publisher("~out_raw", sensor_msgs.msg.Image)

        rospy.Subscriber("~in_compressed", sensor_msgs.msg.CompressedImage, self.receive_img_compressed)
        rospy.Subscriber("~in_raw", sensor_msgs.msg.Image, self.receive_img_raw)

    def receive_img_compressed(self, img_msg):
        img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)
        out = self._receive_img(img)
        self.pub_compressed.publish(self.cvbridge.cv2_to_imgmsg(out))

    def receive_img_raw(self, img_msg):
        img = self.cvbridge.imgmsg_to_cv2(img_msg)
        out = self._receive_img(img)
        self.pub_compressed.publish(self.cvbridge.cv2_to_compressed_imgmsg(out))

    def _receive_img(self, img):
        rospy.loginfo("received img")

        for transform in self.transforms:
            img = transform(img)

        return img


def main():
    rospy.init_node("transformer")

    transformer = Transformer(transforms=[check_robustness.transforms.Translation(0.2, 0.2)])

    rospy.spin()


if __name__ == "__main__":
    main()

