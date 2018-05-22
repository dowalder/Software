#!/usr/bin/env python

from __future__ import division

import argparse
import os

import rospy
import cv2


def main():
    """
    Main
    :return:
    """
    rospy.init_node("check_robustness_basic")
    parser = argparse.ArgumentParser()

    args, _ = parser.parse_known_args()

    rospy.loginfo("Hi")

    # Send image

    # wait for response

    # process response

    rospy.loginfo("Goodbye")


if __name__ == '__main__':

    main()
