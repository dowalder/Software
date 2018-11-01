
import collections

import rosbag
import rospy
import cv_bridge


MessageCollection = collections.namedtuple("MessageCollection", ["topic", "type", "messages"])


class TimeLine:

    def __init__(self, start_state):
        self._time = [0.0]
        self._state = [start_state]
        self._idx = 0

    def add(self, t, state):
        assert self._time[-1] < t
        self._time.append(t)
        self._state.append(state)

    def which_state(self, t_in):
        for i in range(self._idx, len(self._time) - 1):
            if self._time[i] < t_in < self._time[i + 1]:
                self._idx = i
                return self._state[i]
        return self._state[-1]

    def reset_idx(self):
        self._idx = 0


def extract_messages(path, requested_topics, dont_panic=False):
    """
    Sorts the content of the bag after types.

    :param path: str -> path to rosbag
    :param requested_topics: List[str] -> topics for which the messages should be extraced
    :return:
    """
    assert isinstance(path, str)
    assert isinstance(requested_topics, list)

    bag = rosbag.Bag(path)

    _, available_topics = bag.get_type_and_topic_info()

    extracted_messages = {}
    for topic in requested_topics:
        if topic not in available_topics:
            panic_msg = "Could not find the requested topic (%s) in the bag %s" % (topic, path)
            if dont_panic:
                rospy.loginfo(panic_msg)
            else:
                raise ValueError(panic_msg)
        extracted_messages[topic] = MessageCollection(topic=topic, type=available_topics[topic].msg_type, messages=[])

    for msg in bag.read_messages():
        topic = msg.topic
        if topic not in requested_topics:
            continue
        extracted_messages[topic].messages.append(msg)
    bag.close()

    return extracted_messages


def create_runs(msg_collections, img_topic, mode_topic):
    assert isinstance(msg_collections, dict)
    imgs = msg_collections[img_topic]
    modes = msg_collections[mode_topic]

    time_line = TimeLine(0.0)

    for mode in modes.messages:
        if mode.message.state == "LANE_FOLLOWING":
            time_line.add(mode.timestamp.to_sec(), 1)
        else:
            time_line.add(mode.timestamp.to_sec(), 0)

    runs = []
    on = False
    cvbridge = cv_bridge.CvBridge()
    for img in imgs.messages:
        if time_line.which_state(img.timestamp.to_sec()):
            if on:
                runs[-1].append(img)
            else:
                on = True
                runs.append([img])
        else:
            if on:
                on = False

    return runs


def img_gen(img_msgs):
    bridge = cv_bridge.CvBridge()

    if img_msgs.type == "sensor_msgs/CompressedImage":
        converter = bridge.compressed_imgmsg_to_cv2
    elif img_msgs.type == "sensor_msgs/Image":
        converter = bridge.imgmsg_to_cv2
    else:
        raise ValueError("Could not read the image message: unknown image message type %s from the topic %s" %
                         (img_msgs.type, img_msgs.topic))

    for img_msg in img_msgs.messages:
        img = converter(img_msg.message)
        yield img


