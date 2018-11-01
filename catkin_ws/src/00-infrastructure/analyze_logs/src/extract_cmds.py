#!/usr/bin/env python

import os

import analyze_logs
import cv_bridge
import cv2

def find_corresponding_timestamp(msg, msgs):
    stamp = msg.header.stamp
    for idx, omsg in enumerate(msgs):
        if omsg.message.header.stamp == stamp:
            return idx
    return -1

topics = [
    "/megabot06/camera_node/image/compressed",
    "/megabot06/neural_style/corrected",
    "/megabot06/image_transformer_node/corrected_image/compressed",
    "/megabot06/car_cmd_switch_node/cmd",
    "/megabot06/lane_controller_node/car_cmd",
    "/megabot06/lane_filter_node/lane_pose"
]

path = "/home/dominik/dataspace/duckie_bags/experiments_neural_style/big_trial/20_sib_cropped/l_w-curvy.bag"

msgs = analyze_logs.extract_messages(path, topics)

original = msgs["/megabot06/camera_node/image/compressed"]
neural = msgs["/megabot06/neural_style/corrected"]
anti_insta = msgs["/megabot06/image_transformer_node/corrected_image/compressed"]
cmds = msgs["/megabot06/lane_controller_node/car_cmd"]
poses = msgs["/megabot06/lane_filter_node/lane_pose"]

img_idx = 0
pairs = []

for cmd in cmds.messages:
    for idx in range(img_idx, len(original.messages)):
        if original.messages[idx].message.header.stamp == cmd.message.header.stamp:
            pairs.append((cmd.message, original.messages[idx].message))
            img_idx = idx

bridge = cv_bridge.CvBridge()

path = "/home/dominik/dataspace/images/real/duckietown2018/20_sib_cropped/l_w-curvy"
idx = 0
for cmd, img in pairs:
    cmd_path = os.path.join(path, "cmd_%d.txt" % idx)
    img_path = os.path.join(path, "img_%d.jpg" % idx)
    with open(cmd_path, "w") as fid:
        fid.write(str(cmd.omega))
    cv2.imwrite(img_path, bridge.compressed_imgmsg_to_cv2(img))
    idx += 1
