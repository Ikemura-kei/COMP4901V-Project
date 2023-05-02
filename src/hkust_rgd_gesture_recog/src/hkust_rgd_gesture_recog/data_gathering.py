import configparser
import csv
import queue
import threading
import time
import cv2
import mediapipe as mp
import os

CONFIG_FILE = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/data_gathering.py", ""), "config.cfg")
config = configparser.ConfigParser()
config.read(CONFIG_FILE)

data_file = config.get("Collection", "data_file")
image_per_gesture = config.getint("Collection", "image_per_gesture")

mp_hands = None
mp_draw = None
hands = None
cap = None
runningFlag = True
semaphore = threading.Semaphore(0)
workQueue = queue.Queue(10)
data_id = 0
gesture_num = 0


class MyThread(threading.Thread):
    def __init__(self, q):
        threading.Thread.__init__(self)
        self.q = q

    def run(self):
        data = csv.writer(open(data_file, "w"))
        data.writerow(["id", "gesture_num", "gesture",
                       "wrist_x", "wrist_y", "wrist_z",
                       "thumb_cmc_x", "thumb_mcp_x", "thumb_ip_x", "thumb_tip_x", "thumb_cmc_y", "thumb_mcp_y", "thumb_ip_y",
                       "thumb_tip_y", "thumb_cmc_z", "thumb_mcp_z", "thumb_ip_z", "thumb_tip_z",
                       "index_cmc_x", "index_mcp_x", "index_ip_x", "index_tip_x", "index_cmc_y", "index_mcp_y", "index_ip_y",
                       "index_tip_y", "index_cmc_z", "index_mcp_z", "index_ip_z", "index_tip_z",
                       "middle_cmc_x", "middle_mcp_x", "middle_ip_x", "middle_tip_x", "middle_cmc_y", "middle_mcp_y",
                       "middle_ip_y", "middle_tip_y", "middle_cmc_z", "middle_mcp_z", "middle_ip_z", "middle_tip_z",
                       "ring_cmc_x", "ring_mcp_x", "ring_ip_x", "ring_tip_x", "ring_cmc_y", "ring_mcp_y", "ring_ip_y",
                       "ring_tip_y", "ring_cmc_z", "ring_mcp_z", "ring_ip_z", "ring_tip_z",
                       "pinky_cmc_x", "pinky_mcp_x", "pinky_ip_x", "pinky_tip_x", "pinky_cmc_y", "pinky_mcp_y", "pinky_ip_y",
                       "pinky_tip_y", "pinky_cmc_z", "pinky_mcp_z", "pinky_ip_z", "pinky_tip_z",
                       ])
        video_stream(self.q, data)


def flatten(l):
    return [item for sublist in l for item in [sublist.x, sublist.y, sublist.z]]

gestures = ["forward", "left", "right", "stop"]
cnt = 0
def video_stream(q, data):
    global data_id, gesture_num, runningFlag, cnt
    while cnt < len(gestures):
        key = input("Please press enter to start recording gestures for command: {}. (press q to give-up)".format(gestures[cnt]))
        if key == "q":
            break
        name = gestures[cnt]
        gesture_num += 1
        sub_total = 0
        start_time = time.time()
        while True:
            time.sleep(0.1) # record gestures in 10Hz
            for i in range(image_per_gesture):
                semaphore.release()
                l = flatten(workQueue.get())
                l.insert(0, name)
                l.insert(0, gesture_num)
                l.insert(0, data_id)
                data.writerow(l)
                data_id += 1
                sub_total += 1
            if sub_total % 150 == 0:
                print("Done recording gesture " + name + " for " + str(sub_total) + " images")
            if (time.time() - start_time) > 60.0 and sub_total > 2000:
                break
        cnt += 1 # increment indicating we have recorded "cnt" number of gestures
    runningFlag = False



def getDataset(v, mh, md, h):
    global cap, mp_hands, mp_draw, hands
    cap = v
    mp_hands = mh
    mp_draw = md
    hands = h

    thread = MyThread(workQueue)
    thread.start()

    while runningFlag:
        ret, frame = cap.read()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hand_track_results = hands.process(frame_rgb)
        if hand_track_results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_track_results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)
            if semaphore.acquire(timeout=0.01):
                workQueue.put(hand_track_results.multi_hand_landmarks[0].landmark, block=False)
        cv2.imshow("image", frame)
        cv2.waitKey(1)

    thread.join()
    cv2.destroyAllWindows()
    print(cnt == len(gestures))
    return cnt == len(gestures)
    # v.release()
