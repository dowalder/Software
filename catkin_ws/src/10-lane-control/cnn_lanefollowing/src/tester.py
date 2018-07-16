#!/usr/bin/env python
import time

import rospy
import std_msgs.msg

import duckietown_msgs.msg


def stop_msg():
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.seq = 0
    cmd = duckietown_msgs.msg.WheelsCmdStamped()
    cmd.header = header
    cmd.vel_left = 0
    cmd.vel_right = 0
    return cmd


def go_straight_factory(speed):
    def go_straight(i):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.seq = i
        cmd = duckietown_msgs.msg.WheelsCmdStamped()
        cmd.header = header
        cmd.vel_right = - speed
        cmd.vel_left = - speed
        return cmd
    return go_straight


def drive(steps, frequency, step_to_wheelscmdstamped):
    pub = rospy.Publisher("~wheels_cmd", duckietown_msgs.msg.WheelsCmdStamped)

    rate = rospy.Rate(frequency)
    for i in range(steps):
        cmd = step_to_wheelscmdstamped(i)
        pub.publish(cmd)
        rate.sleep()
    pub.publish(stop_msg())


def main():
    rospy.init_node("simple_trajectory")

    time.sleep(1)

    drive(50, 10, go_straight_factory(0.5))


if __name__ == "__main__":
    main()
