#!/usr/bin/env python

"""
Extract runs from rosbag log from duckiebot. This script only extracts images during time intervals where the duckiebot
was in autonomous mode. Additionally, it stores the timestamps of the images.
"""

from __future__ import print_function, division

import argparse
import os

import analyze_logs

import cv_bridge
import rospy
import cv2

IMG_ORIG_TOPIC = "/megabot06/camera_node/image/compressed"
IMG_CORRECTED_TOPIC = "/megabot06/neural_style/corrected"
MODE_TOPIC = "/megabot06/fsm_node/mode"
CMD_TOPIC = "/megabot06/car_cmd_switch_node/cmd"


def extract_run(src_file, tgt_dir, img_topic, video=False, fps=30, store_cmd=False):
    msgs = analyze_logs.extract_messages(src_file, [img_topic, MODE_TOPIC])
    runs = analyze_logs.create_runs(msgs, img_topic, MODE_TOPIC)

    bridge = cv_bridge.CvBridge()
    for idx, run in enumerate(runs):

        if video:
            path = os.path.join(tgt_dir, "%s_%d.avi" % (os.path.basename(src_file), idx))
            rospy.loginfo("Writing the video file to %s" % path)
            img_size = (640, 480)
            out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), int(fps), img_size)

            for img in run:
                img = cv2.resize(src=bridge.compressed_imgmsg_to_cv2(img.message), dsize=img_size)
                if img.ndim < 3 or img.shape[2] == 1:
                    img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)
                out.write(img)
            out.release()
        else:
            rospy.loginfo("Writing images to %s" % tgt_dir)
            for i, img in enumerate(run):
                path = os.path.join(tgt_dir, "run%05d_img%05d.jpg" % (idx, i))
                path_stamp = os.path.join(tgt_dir, "run%05d_img%05d.txt" % (idx, i))
                if not cv2.imwrite(path, bridge.compressed_imgmsg_to_cv2(img.message)):
                    rospy.logerr("Could not write the image %s" % path)
                else:
                    with open(path_stamp, "w") as fid:
                        fid.write(str(img.timestamp))


def process_bag(src_file, tgt_dir, video=False, fps=30):
    orig_dir = os.path.join(tgt_dir, "original")
    corrected_dir = os.path.join(tgt_dir, "corrected")
    try:
        os.mkdir(corrected_dir)
    except OSError:
        pass
    try:
        os.mkdir(orig_dir)
    except OSError:
        pass

    try:
        extract_run(src_file, orig_dir, IMG_ORIG_TOPIC, video, fps)
    except ValueError:
        rospy.loginfo("No original images found in {}".format(src_file))
    try:
        extract_run(src_file, corrected_dir, IMG_CORRECTED_TOPIC, video, fps)
    except ValueError:
        rospy.loginfo("No corrected images found in {}".format(src_file))


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
    parser.add_argument("--fps", default=30, help="Fps of the video, if a video is outputted")
    parser.add_argument("--video", action="store_true", help="Output a video instead of single frames.")
    parser.add_argument("--recursive", "-r", action="store_true",
                        help="searches directories recursively for bags and keeps the folder structure in tgt_dir")

    args, _ = parser.parse_known_args()

    if args.recursive:
        for root, _, files in os.walk(args.src_file):
            for f in files:
                if not f.endswith(".bag"):
                    continue
                src_file = os.path.join(root, f)
                tgt_dir = os.path.relpath(src_file, args.src_file)
                tgt_dir = os.path.join(args.tgt_dir, tgt_dir)
                tgt_dir = tgt_dir[:-4]  # remove the .bag
                try:
                    os.makedirs(tgt_dir)
                except OSError:
                    pass
                process_bag(src_file, tgt_dir, video=args.video, fps=args.fps)
    else:
        process_bag(args.src_file, args.tgt_dir, video=args.video, fps=args.fps)


if __name__ == '__main__':

    main()
