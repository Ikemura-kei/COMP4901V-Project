import configparser
import cv2
import mediapipe as mp
import time
import rospy
from hkust_rgd_gesture_recog.data_gathering import getDataset, flatten
from hkust_rgd_gesture_recog.predict import predict
from hkust_rgd_gesture_recog.train import train
import os 
from hkust_rgd_msgs.msg import gestures
CONFIG_FILE = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/main.py", ""), "config.cfg")
config = configparser.ConfigParser()
config.read(CONFIG_FILE)
mp_min_detect = config.getfloat("General", "mp_min_detect")
mp_max_tracking = config.getfloat("General", "mp_max_tracking")
re_train = config.getboolean("General", "re_train")

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=mp_min_detect, min_tracking_confidence=mp_max_tracking)
confidence_threshold = 4.01
def main():
    pub = rospy.Publisher("/gestures", gestures, queue_size=10)
    rospy.init_node('gesture_recog')
    cap = cv2.VideoCapture(0)
    if re_train:
        getDataset(cap, mp_hands, mp_draw, hands)
        time.sleep(3)
        train()
        print('Training complete')
        cv2.destroyAllWindows()

    cur_label = "NONE"
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hand_track_results = hands.process(frame_rgb)
        if hand_track_results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_track_results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)
            res = predict(flatten(hand_track_results.multi_hand_landmarks[0].landmark))
            if res['label'] is not None:
                cur_label = res["label"] if res["confidence"] > confidence_threshold else "NONE"
                text = f'Label: {cur_label}'
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cur_label = "NONE"
        else:
            cur_label = "NONE"
        cv2.imshow("predict", frame)
        k = cv2.waitKey(1)
        if k == ord("q"):
            break
        
        g = gestures()
        # g.header = time.time()
        g.msg = cur_label
        pub.publish(g)
