#!/usr/bin/env python
import torch

import rospy
import cv_bridge
import cv2
import sensor_msgs.msg

import duckietown_msgs.msg

import cnn_lanefollowing.networks


use_caffe = False
if use_caffe:
    MODEL_DEF = '/home/dominik/workspace/duckietown_imitation_learning/deploy.prototxt'
    MODEL_WEIGHT = '/home/dominik/workspace/duckietown_imitation_learning/duckie_model_iter_10000.caffemodel'
    import caffe

    class CNNController:

        def __init__(self, path):
            self.cnn = caffe.Net(MODEL_DEF, MODEL_WEIGHT, caffe.TEST)

            self.cvbridge = cv_bridge.CvBridge()

            self.pub = rospy.Publisher("~car_cmd", duckietown_msgs.msg.Twist2DStamped)

            rospy.Subscriber("~compressed", sensor_msgs.msg.CompressedImage, self.receive_img)

        def receive_img(self, img_msg):
            rospy.loginfo("received img")

            img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)

            img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_NEAREST)  # img.shape = [120 x 160 x 3]
            img = img[40:, :, :]  # img.shape = [80 x 160 x 3]
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # img.shape = [80 x 160]
            img = cv2.normalize(img.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
            img = img[None, None, :, :]

            self.cnn.blobs['data'].data[...] = img
            out = self.cnn.forward(end='out')

            car_control_msg = duckietown_msgs.msg.Twist2DStamped()
            car_control_msg.header = img_msg.header
            car_control_msg.v = 0.386400014162
            car_control_msg.omega = out["out"][0][0]

            rospy.loginfo("publishing cmd: %f" % out["out"][0][0])

            self.pub.publish(car_control_msg)

else:
    class CNNController:

        def __init__(self, path):
            self.cnn = cnn_lanefollowing.networks.NImagesNet(n=1)
            self.cnn.load_state_dict(torch.load(path))
            self.cvbridge = cv_bridge.CvBridge()

            self.pub = rospy.Publisher("~car_cmd", duckietown_msgs.msg.Twist2DStamped)

            rospy.Subscriber("~compressed", sensor_msgs.msg.CompressedImage, self.receive_img)

        def receive_img(self, img_msg):
            rospy.loginfo("received img")

            img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)

            img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_NEAREST)  # img.shape = [120 x 160 x 3]
            img = img[40:, :, :]  # img.shape = [80 x 160 x 3]
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # img.shape = [80 x 160]
            img = cv2.normalize(img.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
            img = img[None, None, :, :]
            img = torch.Tensor(img)

            out = self.cnn(img)

            car_control_msg = duckietown_msgs.msg.Twist2DStamped()
            car_control_msg.header = img_msg.header
            car_control_msg.v = 0.386400014162
            car_control_msg.omega = out[0]

            rospy.loginfo("publishing cmd: %f" % out[0])

            self.pub.publish(car_control_msg)


def main():
    rospy.init_node("cnn_lanecontrol")

    controller = CNNController(rospy.get_param("~checkpoint_path"))

    rospy.spin()


if __name__ == "__main__":
    main()
