import logging
import rospy
import rosbag
import sys

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logging.debug('This is a log message.')

bag_path = sys.argv[1]
#bag = rosbag.Bag(bag_path)
# topics = bag.get_type_and_topic_info()[1].keys()
# logging.debug((topics)

aruco_topic = '/aruco_marker_publisher/markers'
hsv_topic = '/hsv_detector/objects'
left_state_topic = '/action_provider/left/state'
right_state_topic = '/action_provider/right/state'
speech_topic = 'Rios_speech2text/user_output'
web_topic  = '/web_interface/log'

right_obj_dict = {
    1:   0,
    2:   0,
    3:   0
}
left_obj_dict = {
    150: 0,
    151: 0,
    152: 0,
    153: 0,
    200: 0
}

def get_time_btw_state(arm_topic):
    bag = rosbag.Bag(bag_path)
    a = []
    avg_diff_list = []
    t_frame = ()
    last_t = ()
    s = False
    for topic, msg, t in bag.read_messages(topics=[arm_topic]):
        if (msg.state == 'START' or msg.state == 'DONE') and not s:
            t_frame += (t,)
            s = True
        elif s and msg.state == 'WORKING':
            t_frame += (t,)
            a.append(t_frame)
            avg_diff_list.append(t_frame[1] - t_frame[0])

            s = False
            t_frame = ()

        ## THIS is will be appended as a singleton
        # This denotes an arm has reached its end state
        last_t = (t,)

    avg_diff = sum([i.to_sec() for i in avg_diff_list]) / len(avg_diff_list)
    avg_diff = rospy.Duration.from_sec(avg_diff)
    a.append(last_t + (avg_diff + last_t[0], ))
    bag.close()
    return a

# Will construct a vector of seen objects for a given arm
def get_env_vec_for_arm(arm_topic, obj_dict):
    bag = rosbag.Bag(bag_path)
    # will store the vectors corresponding to the state of
    # the world after each action
    vec_of_vecs = []

    # A list of tuples, where each tuple corresponds
    # to the time frames 
    t_frames = get_time_btw_state(arm_topic)
    t_frame = t_frames.pop(0)

    total_msgs = 0.

    if arm_topic == right_state_topic:
        cam_topic = hsv_topic
    else:
        cam_topic = aruco_topic

    for topic, msg, t in bag.read_messages(topics=[cam_topic]):
        if t < t_frame[1] and t > t_frame[0]:
            total_msgs += 1
            if arm_topic == right_state_topic:
                for o in filter(lambda x: x.id != 0, msg.objects):
                    obj_dict[o.id] += 1
            else:
                for o in filter(lambda x: x.id != 0, msg.markers):
                    obj_dict[o.id] += 1
        elif t < t_frame[0]:
            continue
        elif t > t_frame[1]:
            env_vec = []

            for k in sorted(obj_dict.keys()):
                if total_msgs != 0:
                    env_vec.append(int(round(obj_dict[k] / total_msgs)))
                else:
                    env_vec.append(0)

            #logging.debug( obj_dict)
            obj_dict = obj_dict.fromkeys(obj_dict,0)
            total_msgs = 0.
            vec_of_vecs.append(env_vec)

            try:
                t_frame = t_frames.pop(0)
            except IndexError:
                bag.close()
                return vec_of_vecs

    # This bit of repeated code accounts for the case when the calculated duration
    # of the final state is greater than the actual last timestamp of
    # aruco/hsv msggs
    env_vec = []

    for k in sorted(obj_dict.keys()):
        if total_msgs != 0:
            env_vec.append(int(round(obj_dict[k] / total_msgs)))
        else:
            env_vec.append(0)

    #logging.debug( obj_dict)
    obj_dict = obj_dict.fromkeys(obj_dict,0)
    total_msgs = 0.
    vec_of_vecs.append(env_vec)
    bag.close()
    return vec_of_vecs

def concat_arm_states():
    left = get_env_vec_for_arm(left_state_topic, left_obj_dict)
    right = get_env_vec_for_arm(right_state_topic, right_obj_dict)

    l_and_r = []

    ll = len(left)
    lr = len(right)
    if ll > lr:
        for i in range(0, ll - lr):
            right.append(right[-1])
    elif ll < lr:
        for i in range(0, lr - ll):
            left.append(left[-1])

    for z in zip(right, left):
        l_and_r.append(z[0] + z[1])
    return l_and_r


# a = get_time_btw_state(left_state_topic)
logging.debug(get_time_btw_state(right_state_topic))
#logging.debug( get_env_vec_for_arm(right_state_topic,right_obj_dict))
logging.debug(concat_arm_states())
bag = rosbag.Bag(bag_path)
for topic, msg, t in bag.read_messages(topics=[right_state_topic]):
    logging.debug(msg)

# bag.close()
