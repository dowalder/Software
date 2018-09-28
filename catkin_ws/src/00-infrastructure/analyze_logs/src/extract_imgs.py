#!/usr/bin/env python

from __future__ import print_function, division

import argparse
import os

import analyze_logs

import rospy
import cv2


def extract_msgs(src_file, tgt_dir, img_topic, video=False, fps=30):
    msgs = analyze_logs.extract_messages(src_file, [img_topic])

    imgs = analyze_logs.img_gen(msgs[img_topic])
    if video:
        path = os.path.join(tgt_dir, "%s.avi" % os.path.basename(src_file))
        rospy.loginfo("Writing the video file to %s" % path)
        img_size = (640, 480)
        out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), int(fps), img_size)

        for img in imgs:
            img = cv2.resize(src=img, dsize=img_size)
            if img.ndim < 3 or img.shape[2] == 1:
                img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)
            out.write(img)
        out.release()
    else:
        rospy.loginfo("Writing images to %s" % tgt_dir)
        for i, img in enumerate(imgs):
            path = os.path.join(tgt_dir, "img%05d.jpg" % i)
            if not cv2.imwrite(path, img):
                rospy.logerr("Could not write the image %s" % path)


def main():
    """
    Main
    :return:
    """
    rospy.init_node("extract_runs")
    parser = argparse.ArgumentParser()

    parser.add_argument("--src_file", "-s", required=True,
                        help="The .bag file where the images should be created from.")
    parser.add_argument("--tgt_dir", "-t", required=True, help="Where the generated images should be stored.")
    parser.add_argument("--img_topic", required=True, help="the topic to get the images from")
    parser.add_argument("--fps", default=30, help="Fps of the video, if a video is outputted")
    parser.add_argument("--video", action="store_true", help="Output a video instead of single frames.")

    args, _ = parser.parse_known_args()

    try:
        os.makedirs(args.tgt_dir)
    except OSError:
        pass

    extract_msgs(args.src_file, args.tgt_dir, args.img_topic, args.video, args.fps)


if __name__ == '__main__':
    main()
