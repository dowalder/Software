#!/usr/bin/env python
import rospy
import torch.utils.data

import cnn_lanefollowing.networks
import cnn_lanefollowing.dataset


def main():
    rospy.init_node("analyze_cnn")

    net_paths = rospy.get_param("~trained_networks")
    nets = []
    for path in net_paths:
        net = cnn_lanefollowing.networks.NImagesNet(n=1)
        net.load_state_dict(torch.load(path))
        nets.append(net)

    data_dir = rospy.get_param("~data_dir")
    data_set = cnn_lanefollowing.dataset.DataSet(data_dir)
    data_loader = torch.utils.data.DataLoader(data_set)

    tgt_file = rospy.get_param("~out_file")

    out = []

    for lbl, img in data_loader:
        current = [float(lbl[0])]
        for net in nets:
            current.append(float(net(img)[0]))
        out.append(current)

    with open(tgt_file, "w") as fid:
        fid.write(",".join(["gt"] + net_paths) + "\n")
        for line in out:
            fid.write(",".join(map(str, line)) + "\n")


if __name__ == "__main__":
    main()
