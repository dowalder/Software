#!/usr/bin/env python

from __future__ import division

import argparse
import time

import rospy
import sensor_msgs.msg
import std_msgs.msg
import cv_bridge

import check_robustness


class Talker:

    def __init__(self, data_dir, ground_truth=False):
        self.data_loader = check_robustness.DataLoader(path=data_dir, has_ground_truth=ground_truth)
        self.data_iter = iter(self.data_loader)
        self.gt = ground_truth
        self.pub = rospy.Publisher("check_robustness/image_raw", sensor_msgs.msg.Image)
        self.sub = rospy.Subscriber("check_robustness/control_out",
                                    std_msgs.msg.Float64MultiArray,
                                    callback=self.receive,
                                    callback_args=self)

        self.label = None
        self.img = None
        self.received = False
        self.cvbridge = cv_bridge.CvBridge()

        rospy.init_node("check_robustness", anonymous=False)

    def load_next(self):
        if self.gt:
            self.img, self.label = next(self.data_iter)
        else:
            self.img = next(self.data_iter)

    def run(self):
        self.load_next()

        max_interval = 10.0

        start = time.time()
        while not rospy.is_shutdown():
            self.pub.publish(self.cvbridge.cv2_to_imgmsg(self.img))

            while not rospy.is_shutdown():
                if self.received:
                    self.load_next()
                    self.received = False
                    start = time.time()
                    break
                if time.time() - start > max_interval:
                    start = time.time()
                    break

                time.sleep(0.1)

    def receive(self, msg):
        if self.gt:
            rospy.loginfo("label: %s, gt: %s" % (msg, self.label))
        else:
            rospy.loginfo("label: %s" % msg)

        self.received = True


def main():
    """
    Main
    :return:
    """
    rospy.init_node("check_robustness_basic")
    parser = argparse.ArgumentParser()

    parser.add_argument("--data_dir", "-d", required=True, help="Directory containing the images.")
    parser.add_argument("--gt", action="store_true", help="If the --data_dir contains ground truth labels.")

    args, _ = parser.parse_known_args()

    rospy.loginfo("Hi")

    talker = Talker(args.data_dir, args.gt)

    talker.run()

    rospy.loginfo("Goodbye")


if __name__ == '__main__':

    main()
