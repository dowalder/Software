import rosbag
import cv2
import numpy as np
import cv_bridge
import collections


# General parametrization of a control signal for a car.
class ControlInput:
    def __init__(self, speed=0.0, steering=0.0):
        self.speed = speed
        self.steering = steering


# A collection of ros messages coming from a single topic.
MessageCollection = collections.namedtuple("MessageCollection", ["topic", "type", "messages"])


def extract_messages(path, requested_topics):
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
            raise ValueError("Could not find the requested topic (%s) in the bag %s" % (topic, path))
        extracted_messages[topic] = MessageCollection(topic=topic, type=available_topics[topic].msg_type, messages=[])

    for msg in bag.read_messages():
        topic = msg.topic
        if topic not in requested_topics:
            continue
        extracted_messages[topic].messages.append(msg)
    bag.close()

    return extracted_messages


def assign_nearest_points(references, points, comp=lambda x, y: x <= y, diff=lambda x, y: x - y):
    """
    Takes two *sorted* lists and finds the closest point of every entry of the second list to an entry of the first one.

    :param references: List -> sorted list containing the reference points
    :param points: List -> sorted list containing the point to be assigned to one of the reference points
    :param comp: Function -> To compare elements. It should be the same that was used to sort the list, so that for both
                            lists the following statement holds: comp(l[i], l[i+1]) OR l[i] == l[i+1]
    :param diff: Function -> The difference between two elements. Use for complex data types.
    :return: List -> of the same length as points, containing the index of the reference point assigning a point
                    in points to the closest point in references
    """
    ref_idx = 0
    pts_idx = 0
    correspondences = []
    while pts_idx < len(points):
        point = points[pts_idx]
        if ref_idx == 0:
            if comp(point, references[0]):
                correspondences.append(0)
                pts_idx += 1
                continue
        if ref_idx == len(references) - 1:
            correspondences.append(len(references) - 1)
            pts_idx += 1
        elif comp(diff(point, references[ref_idx]), diff(references[ref_idx + 1], point)):
            correspondences.append(ref_idx)
            pts_idx += 1
        elif comp(point, references[ref_idx + 1]):
            correspondences.append(ref_idx + 1)
            pts_idx += 1
        else:
            ref_idx += 1

    return correspondences


def average_correspondences(references, points, correspondences, compute_average=lambda l: sum(l) / len(l)):
    """
    For every entry in references, it computes the average value of points that correspond to this reference.

    :param references: List
    :param points: List
    :param correspondences: List -> len(correspondences) == len(points): assigns every entry in point to an entry in
                                    references
    :param compute_average: Function -> Compute average of a list of points.
    :return:
    """
    new_points = [[] for _ in range(len(references))]

    for i, pt_idx in enumerate(correspondences):
        assert pt_idx < len(references), "Invalid correspondence: the index must point to an element in 'references'"
        new_points[pt_idx].append(points[i])

    return map(compute_average, new_points)


def draw_steering_arrow(angle, speed, img, max_speed=1.0):
    """
    Draws an arrow to the image that show angle and speed of the steering

    :param angle: float
    :param speed: float
    :param img: np.ndarray -> (N x M x [1,3])
    :param max_speed: float -> speed for which the arrow has its maximal size
    :return: np.ndarray
    """
    if speed == 0:
        return img

    assert isinstance(img, np.ndarray)
    assert img.ndim >= 2
    height, width = img.shape[:2]

    pt1 = np.array([width / 2.0, height * 3 / 4], np.int)

    pt2 = np.array([-np.sin(angle), 1.0])
    pt2 = pt2 / np.linalg.norm(pt2) * (speed / max_speed) * (height / 2.0)
    pt2[0] += width / 2.0
    pt2 = pt2.astype(np.int)

    cv2.arrowedLine(img=img, pt1=tuple(pt1), pt2=tuple(pt2), color=(255, 0, 0), thickness=2)

    return img


def img_generator(bag_file, img_topic, control_topic, verbose=False):
    """
    Returns a generator that serves images which have the steering command drawn as an arrow on it.

    :param bag_file: str -> path to rosbag file
    :param img_topic: str -> the topic on which the images are published in the rosbag
    :param control_topic: str -> the topic on which the control commands are published in the rosbag
    :return: generator that yields images (np.ndarray)
    """

    # rospy.loginfo("Extracting messages...")
    sorted_bag = extract_messages(path=bag_file, requested_topics=[img_topic, control_topic])

    control_msgs = sorted_bag[control_topic]
    img_msgs = sorted_bag[img_topic]
    # rospy.loginfo("Found %d messages on the topic %s and %d messages on the topic %s." %
    #               (len(control_msgs.messages), control_topic, len(img_msgs.messages), img_topic))

    def _subtract_message_tstamps(msg1, msg2):
        """
        Creates a dummy class that contains the timestamp differences between msg1 and msg2. Very specific to use in
        assign_nearest_points, so we keep it local.
        """

        class Dummy:
            def __init__(self):
                self.timestamp = msg1.timestamp - msg2.timestamp

        return Dummy()

    # rospy.loginfo("Assigning all control messages to a image message...")
    correspondences = assign_nearest_points(references=img_msgs.messages,
                                            points=control_msgs.messages,
                                            comp=lambda x, y: x.timestamp <= y.timestamp,
                                            diff=_subtract_message_tstamps)

    # rospy.loginfo("Averaging multiple control messages per image...")
    averages = average_correspondences(references=img_msgs.messages,
                                       points=control_msgs.messages,
                                       correspondences=correspondences,
                                       compute_average=select_control_averager(control_msgs.type))

    # rospy.loginfo("Serving the images...")
    bridge = cv_bridge.CvBridge()
    for i, img_msg in enumerate(img_msgs.messages):
        if img_msgs.type == "sensor_msgs/CompressedImage":
            img = bridge.compressed_imgmsg_to_cv2(img_msg.message)
        elif img_msgs.type == "sensor_msgs/Image":
            img = bridge.imgmsg_to_cv2(img_msg.message)
        else:
            raise ValueError("Could not read the image message: unknown image message type %s from the topic %s" %
                             (img_msgs.type, img_msgs.topic))

        control = averages[i]

        yield draw_steering_arrow(control.steering, control.speed, img=img, max_speed=1.0)


def compute_carcontrol_average(controls):
    """
    Computes the average of a list of CarControls
    :param controls: List[duckietown_msgs.msg.CarControl]
    :return: ControlInput
    """
    avg = ControlInput(speed=0.0, steering=0.0)

    for msg in controls:
        control = msg.message
        avg.speed += control.speed / len(controls)
        avg.steering += control.steering / len(controls)
    return avg


def compute_wheels_cmd_average(controls):
    """
    Computes converts WheelsCmd to ControlInput and averages over a list.

    :param controls: List[duckietown_msgs.msg.CarControl]
    :return: ControlInput
    """
    avg = ControlInput(speed=0.0, steering=0.0)

    for msg in controls:
        control = msg.message
        avg.speed += (control.vel_left + control.vel_right) / len(controls)
        # The 0.1 for the second argument for arctan2 is the estimated distance between the duckiebot wheels.
        avg.steering += np.arctan2(1 - control.vel_left / (control.vel_right + 1e-6), 0.1) / len(controls)

    return avg


def select_control_averager(control_type):
    """
    Selects the correct function that averages a list of control messages of a certain type. The output of these
    functions is a ControlInput() instance containing the result.

    :param control_type: str -> ros message type
    :return:
    """
    assert isinstance(control_type, str)

    if control_type == "duckietown_msgs/WheelsCmd":
        return compute_wheels_cmd_average
    elif control_type == "duckietown_msgs/CarControl":
        return compute_carcontrol_average
    else:
        raise ValueError("The message type is not supported: %s" % control_type)