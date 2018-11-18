#!/usr/bin/env python

"""
Creates a set of images or a video with a drawn steering arrow on top of them from a rosbag.
"""

from __future__ import print_function, division

import argparse
import os

import analyze_logs

import rospy
import cv2


def main():
    """
    Main
    :return:
    """
    rospy.init_node("draw_steering_arrow")
    parser = argparse.ArgumentParser()

    parser.add_argument("--src_file", "-s", required=True,
                        help="The .bag file where the images should be created from.")
    parser.add_argument("--tgt_dir", "-t", required=True, help="Where the generated images should be stored.")
    parser.add_argument("--control_topic", required=True, help="Which topic contains the control inputs.")
    parser.add_argument("--img_topic", required=True, help="Which topic contains the images.")
    parser.add_argument("--fps", default=10, help="Fps of the video, if a video is outputted")
    parser.add_argument("--video", action="store_true", help="Output a video instead of single frames.")

    args, _ = parser.parse_known_args()

    rospy.loginfo("Hi")

    img_gen = analyze_logs.img_generator(bag_file=args.src_file,
                                         img_topic=args.img_topic,
                                         control_topic=args.control_topic)

    if args.video:
        path = os.path.join(args.tgt_dir, "%s.avi" % os.path.basename(args.src_file))
        rospy.loginfo("Writing the video file to %s" % path)
        img_size = (640, 480)
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), int(args.fps), img_size)

        for img in img_gen:
            img = cv2.resize(src=img, dsize=img_size)
            if img.ndim < 3 or img.shape[2] == 1:
                img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)
            out.write(img)
        out.release()
    else:
        rospy.loginfo("Writing images to %s" % args.tgt_dir)
        for i, img in enumerate(img_gen):
            path = os.path.join(args.tgt_dir, "%05d.jpg" % i)
            if not cv2.imwrite(path, img):
                rospy.logerr("Could not write the image %s" % path)

    rospy.loginfo("Goodbye")


if __name__ == '__main__':

    main()
    # unittest.main()
