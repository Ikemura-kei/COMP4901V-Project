import torch
import configparser
import os
from hkust_rgd_gesture_recog.model import GestureNetwork

CONFIG_FILE = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/predict.py", ""), "config.cfg")
config = configparser.ConfigParser()
config.read(CONFIG_FILE)
training_model_file = config.get("Testing", "model_file")
training_model_file = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/predict.py", ""), training_model_file)
threshold = config.getfloat("Testing", "threshold")
model = None


def predict(x):
    global model
    if model is None:
        params = torch.load(training_model_file)
        model = GestureNetwork(params['mapping'])
        model.load_state_dict(params['param'])
    raw = model.solve(torch.FloatTensor(x))
    raw.sort(key=lambda x: x[1], reverse=True)
    return {'label': raw[0][0] if raw[0][1] > threshold else None, 'confidence': raw[0][1], 'raw': raw}
