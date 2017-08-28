#!/usr/bin/env python
from trainSpeechPredictor import *
from human_robot_collaboration_msgs.msg import *
from ros_speech2text.msg import *
from aruco_msgs.msg import *
from human_robot_collaboration_msgs.srv import *

""" This node Will let Baxter predict the next action to take based on speech + world info.
    Model is trained from data contained in rosbags
"""

def createStateAndSpeechVec():
    """Creates vec of world state and speech online"""
    counter = 0.
    left_dict = left_obj_dict.copy()
    right_dict = right_obj_dict.copy()
    vec = []
    # Take 10 readings to mitigate noise illusions
    while counter < 10:
        aruco_msg = rospy.wait_for_message(aruco_topic, MarkerArray).markers
        hsv_msg = rospy.wait_for_message(hsv_topic, ObjectsArray).objects
        for o in aruco_msg:
            left_dict[o.id] += 1
        for o in hsv_msg:
            right_dict[o.id] += 1
        counter += 1

    for k in right_dict.keys():
        obj_state = round(right_dict[k] / counter)
        vec.append(int(obj_state))
    for k in left_dict.keys():
        obj_state = round(left_dict[k] / counter)
        vec.append(int(obj_state))

    return vec



def speechCB(msg, vocab):
    print "Received speech"
    v = vocab.copy()
    sent = msg.transcript.split()
    for w in sent:
        try:
            v[w.lower()] += 1
        except KeyError:
            pass

    for i in v.values():
        speech_vec.append(i)

if __name__ == '__main__':
    rospy.init_node("predictor", anonymous=True)
    state_vec = []
    speech_vec = []


    # load ML model and vocab from storage, if they exists
    if os.path.isfile(vocab_path) and os.path.isfile(model_path):
        v = joblib.load(vocab_path)
        model = joblib.load(model_path)
    # If they dont, create them and save
    else:
        print "No model saved, creating new model!"
        v = create_vocab(bag_dir_path)
        X,Y = create_data_set(bag_dir_path, v)

        model = GaussianNB()
        model.fit(X,Y)

        with open(model_path,"wb") as m:
            joblib.dump(model,m, compress=9)

        with open(vocab_path,"wb") as m:
            joblib.dump(v,m, compress=9)


    service_left = rospy.ServiceProxy(
    "/action_provider/service_left", DoAction)
    service_right = rospy.ServiceProxy(
    "/action_provider/service_right", DoAction)
    speech_sub = rospy.Subscriber(speech_topic,
                                  transcript,
                                  lambda msg: speechCB(msg, v))
    # Predicts next action based on state of world + speech
    while  not rospy.is_shutdown():
        # gets arm states
        left_home = False
        right_home = False

        left_state = rospy.wait_for_message(left_state_topic, ArmState).state
        right_state = rospy.wait_for_message(right_state_topic, ArmState).state

        if left_state == "START" or left_state == "DONE": left_home = True
        if right_state == "START" or right_state == "DONE": right_home = True

        print left_state, right_state
        # if both arms are in the home position
        if left_home and right_home:
            # Get state vec
            state_vec = createStateAndSpeechVec()
            # Speech vec is constantly being update
            print(state_vec)
            print(speech_vec)
            if state_vec and speech_vec:
                x_input = np.asarray(state_vec + speech_vec).reshape(1, -1)
                predicted_label= model.predict(x_input)[0].split(":")
                predicted_action = predicted_label[0]
                print(predicted_label)
                if len(predicted_label) > 1:
                    predicted_obj = [id_dict[predicted_label[1]]]
                else:
                    predicted_obj = []
                service_left(predicted_action, predicted_obj )
                service_right(predicted_action, predicted_obj )

                state_vec = []
                speech_vec = []

    rospy.spin()
