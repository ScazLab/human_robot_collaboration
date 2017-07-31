import logging
import rospy
import rosbag
import sys
import os
import collections as c
import numpy as np
import sklearn

logging.basicConfig(
    level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logging.debug('This is a log message.')

bag_dir_path = sys.argv[1]
#bag = rosbag.Bag(bag_path)
# topics = bag.get_type_and_topic_info()[1].keys()
# logging.debug((topics)

aruco_topic = '/aruco_marker_publisher/markers'
hsv_topic = '/hsv_detector/objects'
left_state_topic = '/action_provider/left/state'
right_state_topic = '/action_provider/right/state'
speech_topic = '/ros_speech2text/user_output'
web_topic = '/web_interface/log'

right_obj_dict = {1: 0, 2: 0, 3: 0}

left_obj_dict = {150: 0, 151: 0, 152: 0, 153: 0, 200: 0}


def get_time_btw_state(arm_topic, bag_path):
    bag = rosbag.Bag(bag_path)
    states = []
    avg_diff_list = []
    t_frame = ()
    last_t = ()
    s = False
    for topic, msg, t in bag.read_messages(topics=[arm_topic]):
        if (msg.state == 'START' or msg.state == 'DONE' or
                msg.state == 'ERROR') and not s:
            t_frame += (t, )
            s = True
        elif s and msg.state == 'WORKING':
            t_frame += (t, )
            states.append((t_frame, msg.action + '_' + msg.object))
            avg_diff_list.append(t_frame[1] - t_frame[0])

            s = False
            t_frame = ()

        ## THIS is will be appended as a singleton
        # This denotes an arm has reached its end state
        last_t = (t, )

    avg_diff = sum([i.to_sec() for i in avg_diff_list]) / len(avg_diff_list)
    avg_diff = rospy.Duration.from_sec(avg_diff)
    states.append((last_t + (avg_diff + last_t[0], ),
              msg.action + '_' + msg.object))
    bag.close()
    return states


""" Will construct a vector of seen objects for a given arm"""
def get_env_vec_for_arm(arm_topic, obj_dict, bag_path):
    bag = rosbag.Bag(bag_path)
    obj_dict = obj_dict.copy()
    # will store the vectors corresponding to the state of
    # the world after each action
    vec_of_vecs = []

    # A list of tuples, where each tuple corresponds
    # to the time frames
    t_frames = [t[0] for t in get_time_btw_state(arm_topic, bag_path)]
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
            obj_dict = obj_dict.fromkeys(obj_dict, 0)
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
            obj_state = round(obj_dict[k] / total_msgs)
            env_vec.append(int(obj_state))
        else:
            env_vec.append(0)

    # logging.debug( obj_dict)
    obj_dict = obj_dict.fromkeys(obj_dict, 0)
    total_msgs = 0.
    vec_of_vecs.append(env_vec)
    bag.close()
    return vec_of_vecs


def concat_arm_states(bag_path):
    left = get_env_vec_for_arm(left_state_topic, left_obj_dict, bag_path)
    right = get_env_vec_for_arm(right_state_topic, right_obj_dict, bag_path)

    left_t_frames = get_time_btw_state(left_state_topic, bag_path)
    right_t_frames = get_time_btw_state(right_state_topic, bag_path)

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


def match_speech_with_state_vec(bag_path, vocab):
    # Turn OrdereredDicts into lists for easy manipulation
    left_t_frames = get_time_btw_state(left_state_topic,
                                       bag_path)
    right_t_frames = get_time_btw_state(right_state_topic,
                                        bag_path)

    left_vec = get_env_vec_for_arm(left_state_topic,
                                   left_obj_dict, bag_path)
    right_vec = get_env_vec_for_arm(right_state_topic,
                                    right_obj_dict, bag_path)

    state_vecs = [] # an ordered list of state + speech vecs
    labels = [] # corresponding action labels

    # Item 0 of tuple labels the arm for easy parsing below
    # Item 1 is the label
    # item 2 is the time of the state change (i.e. WORKING)
    l_state_changes = [("L", i[1], i[0][1]) for i in left_t_frames[0:-1]]
    r_state_changes = [("R", i[1], i[0][1]) for i in right_t_frames[0:-1]]


    print left_vec
    print right_vec
    # sort by time
    srted_state_changes = (l_state_changes + r_state_changes)
    srted_state_changes.sort(key=lambda x: x[2])

    l_i = 0
    r_i = 0

    print len(srted_state_changes)
    print len(left_t_frames) ,len(right_t_frames)
    for s in srted_state_changes:
        state_vecs.append(right_vec[r_i] + left_vec[l_i])
        labels.append(s[1])
        if s[0] == "L":
            l_i += 1
        else:
            r_i += 1

    print state_vecs

    speech_lst = []
    dict_list = []

    for s in srted_state_changes:
        bag = rosbag.Bag(bag_path)
        min_delta = sys.maxint
        best_s = ""
        for topic, msg, t in bag.read_messages(topics=[speech_topic]):
            # print msg.transcript
            diff = abs(s[2] - t).to_sec()
            if diff < min_delta:
                min_delta = diff
                best_s = msg.transcript
        speech_lst.append(best_s)
        bag.close()
    assert len(speech_lst ) == len(state_vecs)
    # Creates a dict with counts for each word that appears in
    # the speech utterance
    logging.debug(speech_lst)
    for i in range(len(speech_lst)):
        v = vocab.fromkeys(vocab, 0)
        sent = speech_lst[i].split(" ")
        for w in sent:
            v[w.lower()] += 1

        state_vecs[i] += v.values()
        dict_list.append(v)

    
    return state_vecs, labels


def create_vocab(bag_dir_path):
    vocab = c.OrderedDict()
    for filename in os.listdir(bag_dir_path):
        path = os.path.join(bag_dir_path, filename)
        bag = rosbag.Bag(path)
        for topic, msg, t in bag.read_messages(topics=[speech_topic]):
            sent = msg.transcript.split(" ")
            for w in sent:
                vocab[w.lower()] = 0

        bag.close()
    return vocab


def create_training_set(bag_dir_path, vocab):
    training_X = []
    training_Y = []
    for filename in os.listdir(bag_dir_path):
        path = os.path.join(bag_dir_path, filename)
        X, Y = match_speech_with_state_vec(path, v)
        training_X.append(X)
        training_Y.append(Y)
    return np.asarray(training_X), np.asarray(training_Y)

# a = get_time_btw_state(left_state_topic)
# logging.debug(get_time_btw_state(right_state_topic))
# logging.debug( get_env_vec_for_arm(right_state_topic,right_obj_dict))
# logging.debug(concat_arm_states())
# print 'hi', match_speech_with_state_vec()
v  = create_vocab(bag_dir_path)

# training_x, training_y = match_speech_with_state_vec(bag_dir_path
#                                                       + "DataCollectionTest_2017-07-26-14-13-57.bag",
#                                                       v)
training_X, training_Y = create_training_set(bag_dir_path, v)
test_X, test_Y = create_training_set("speech_prediction_bags/JakeTest", v)
print training_Y
# print match_speech_with_state_vec(bag_dir_path + "JakeTest_2017-07-26-15-13-27.bag", v)
#print concat_arm_states(bag_dir_path + "JakeTest_2017-07-26-15-13-27.bag")
# bag = rosbag.Bag(bag_dir_path + "JakeTest_2017-07-26-15-13-27.bag")
# for topic, msg, t in bag.read_messages(topics=[right_state_topic,left_state_topic]):
#     logging.debug(msg)

# bag.close()
