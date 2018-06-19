#!/usr/bin/env python
import rospy

import cnn_lanefollowing.train


def run_basic_training():
    """
    Trains a network copy of https://github.com/syangav/duckietown_imitation_learning/blob/master/demo.py

    :return:
    """
    test_dir = rospy.get_param("~test_dir")
    train_dir = rospy.get_param("~train_dir")
    model_dir = rospy.get_param("~checkpoint_dir")

    net, train_loader, test_loader, criterion, optimizer = cnn_lanefollowing.train.exact_caffe_copy_factory(
        train_dir, test_dir)

    cnn_lanefollowing.train.train_net(net, train_loader, test_loader, criterion, optimizer, model_dir, device="cuda:0",
                                      save_interval=1000, num_epoch=400)


def main():
    rospy.init_node("cnn_training")

    run_basic_training()


if __name__ == "__main__":
    main()
