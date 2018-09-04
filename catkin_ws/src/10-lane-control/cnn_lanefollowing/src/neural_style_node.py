#!/usr/bin/env python

import time

import rospy
import zmq
import sensor_msgs.msg


class Python3Connector(object):
    def __init__(self):
        pub_topic = "ipc:///home/dominik/tmp/image.zeromq"
        sub_topic = "ipc:///home/dominik/tmp/image_corrected.zeromq"
        rospy.Subscriber("~uncorrected", sensor_msgs.msg.CompressedImage, self.receive_img, queue_size=1)
        self.pub_ros = rospy.Publisher("~corrected", sensor_msgs.msg.CompressedImage)

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
        rospy.loginfo("took {}s".format(message, time.time() - t))

        corrected_img = sensor_msgs.msg.CompressedImage()
        corrected_img.header = img_msg.header
        corrected_img.data = message

        self.pub_ros.publish(corrected_img)


def main():
    rospy.init_node("neural_style")

    connector = Python3Connector()

    rospy.spin()


if __name__ == "__main__":
    main()