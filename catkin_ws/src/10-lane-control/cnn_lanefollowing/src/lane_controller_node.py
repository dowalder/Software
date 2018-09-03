#!/usr/bin/env python
import imp
import time

import torch
import numpy as np

import rospy
import zmq
import cv_bridge
import cv2
import sensor_msgs.msg

import duckietown_msgs.msg

import cnn_lanefollowing.networks


# use_caffe = False
# if use_caffe:
#     MODEL_DEF = '/home/dominik/workspace/duckietown_imitation_learning/deploy.prototxt'
#     MODEL_WEIGHT = '/home/dominik/workspace/duckietown_imitation_learning/duckie_model_iter_10000_original.caffemodel'
#     import caffe
#
#     class CNNController:
#
#         def __init__(self, path):
#             self.cnn = caffe.Net(MODEL_DEF, MODEL_WEIGHT, caffe.TEST)
#
#             self.cvbridge = cv_bridge.CvBridge()
#
#             self.pub = rospy.Publisher("~car_cmd", duckietown_msgs.msg.Twist2DStamped)
#
#             rospy.Subscriber("~compressed", sensor_msgs.msg.CompressedImage, self.receive_img, queue_size=1)
#
#         def receive_img(self, img_msg):
#             rospy.loginfo("received img")
#
#             img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)
#
#             img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_NEAREST)  # img.shape = [120 x 160 x 3]
#             img = img[40:, :, :]  # img.shape = [80 x 160 x 3]
#             img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # img.shape = [80 x 160]
#             img = cv2.normalize(img.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
#             img = img[None, None, :, :]
#
#             self.cnn.blobs['data'].data[...] = img
#             out = self.cnn.forward(end='out')
#
#             car_control_msg = duckietown_msgs.msg.Twist2DStamped()
#             car_control_msg.header = img_msg.header
#             car_control_msg.v = 0.386400014162
#             car_control_msg.omega = out["out"][0][0]
#
#             rospy.loginfo("publishing cmd1: %f" % out["out"][0][0])
#
#             self.pub.publish(car_control_msg)
#
# elif True:
#     class CNNController:
#
#         def __init__(self, path, device="cpu"):
#             controllers = imp.load_source("src.controllers", "/home/dominik/workspace/gym-duckietown")
#             params_mod = imp.load_source("params", "/home/dominik/workspace/gym-duckietown/src/params.py")
#             params = params_mod.TestParams("/home/dominik/workspace/gym-duckietown/conf_files/basic_lanefollower.yaml", "basic_lanefollower")
#             self.cnn = controllers.OmegaController(params)
#
#             self.cvbridge = cv_bridge.CvBridge()
#
#             self.pub = rospy.Publisher("~car_cmd", duckietown_msgs.msg.Twist2DStamped)
#
#             rospy.Subscriber("~compressed", sensor_msgs.msg.CompressedImage, self.receive_img, queue_size=1)
#
#         def receive_img(self, img_msg):
#             rospy.loginfo("received img")
#
#             img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)
#
#             img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_NEAREST)  # img.shape = [120 x 160 x 3]
#             img = img[40:, :, :]  # img.shape = [80 x 160 x 3]
#             img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # img.shape = [80 x 160]
#             img = cv2.normalize(img.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
#             img = img[None, None, :, :]
#             img = torch.Tensor(img)
#             img = img.to(self.cnn.params.device)
#
#             out = self.cnn.step(img).action
#
#             car_control_msg = duckietown_msgs.msg.Twist2DStamped()
#             car_control_msg.header = img_msg.header
#             car_control_msg.v = 0.2
#             car_control_msg.omega = out[0]
#
#             rospy.loginfo("publishing cmd: %f" % out[0])
#             print (img_msg.header.stamp - rospy.Time.now())
#
#             self.pub.publish(car_control_msg)
# else:


class Python3Connector(object):
    def __init__(self):
        pub_topic = "ipc:///home/dominik/tmp/image.zeromq"
        sub_topic = "ipc:///home/dominik/tmp/command.zeromq"
        rospy.Subscriber("~compressed", sensor_msgs.msg.CompressedImage, self.receive_img, queue_size=1)
        self.pub_ros = rospy.Publisher("~car_cmd", duckietown_msgs.msg.Twist2DStamped)

        self.context = zmq.Context()
        self.pub_zmq = self.context.socket(zmq.PUB)
        self.pub_zmq.bind(pub_topic)

        self.sub_zmq = self.context.socket(zmq.SUB)
        self.sub_zmq.connect(sub_topic)
        self.sub_zmq.setsockopt_string(zmq.SUBSCRIBE, u"")

    def receive_img(self, img_msg):
        rospy.loginfo("Received message")
        t = time.time()
        self.pub_zmq.send(img_msg.data)
        message = self.sub_zmq.recv()
        rospy.loginfo("{} : took {}s".format(message, time.time() - t))
        cmd = float(message)

        car_control_msg = duckietown_msgs.msg.Twist2DStamped()
        car_control_msg.header = img_msg.header
        car_control_msg.v = -0.2
        car_control_msg.omega = -cmd   # TODO: CHANGE BACK
        self.pub_ros.publish(car_control_msg)


def main():
    rospy.init_node("cnn_lanecontrol")

    pth = rospy.get_param("~checkpoint_path")

    rospy.loginfo("model path: {}".format(pth))

    # controller = CNNController(pth, "cuda:0")
    connector = Python3Connector()

    rospy.spin()


if __name__ == "__main__":
    main()