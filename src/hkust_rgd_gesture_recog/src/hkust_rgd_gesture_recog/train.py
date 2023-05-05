import configparser
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
import torch.optim as optim
import pandas as pd
import numpy as np
import os
from hkust_rgd_gesture_recog.model import GestureNetwork

CONFIG_FILE = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/train.py", ""), "config.cfg")
config = configparser.ConfigParser()
config.read(CONFIG_FILE)
training_data_file = None
training_model_file = None
output_dict = None
device = "cuda" if torch.cuda.is_available() else "cpu"


class GestureDataset(Dataset):
    def __init__(self, data_file):
        self.df = pd.read_csv(data_file, header=0)

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.df.iloc[idx][3:].tolist()).to(device), torch.tensor(
            self.df.iloc[idx]['gesture_num'] - 1).to(device)


def verify(data_loader, used_model):
    correct = 0
    total = 0
    with torch.no_grad():
        for data in data_loader:
            inputs, labels = data
            outputs = used_model(inputs)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
        return correct / total


def train(custom_gesture_register):
    global output_dict, training_data_file, training_model_file
    training_data_file = config.get("Training", "data_file")
    training_data_file = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/train.py", ""), training_data_file)
    training_model_file = config.get("Training", "model_file")
    training_model_file = os.path.join(__file__.replace("src/hkust_rgd_gesture_recog/train.py", ""), training_model_file)
    if custom_gesture_register:
        training_data_file = training_data_file.replace("gesture.data", "custom.data")
        training_model_file = training_model_file.replace("model.pt", "custom.pt")
    print("save model to: ", training_model_file)
    print("--> Use the following data file for training:",training_data_file)
    csv_data = pd.read_csv(training_data_file, header=0)
    uni_cols = csv_data.iloc[np.unique(csv_data['gesture_num'], return_index=True)[1]]
    output_dict = dict(zip(uni_cols['gesture_num'] - 1, uni_cols['gesture']))
    data = GestureDataset(training_data_file)
    val_set, test_set, train_set = torch.utils.data.random_split(data, [int(len(data) * 0.1), int(len(data) * 0.1),
                                                                        len(data) - 2 * int(len(data) * 0.1)])
    loader = [
        DataLoader(train_set, batch_size=32),
        DataLoader(val_set),
        DataLoader(test_set)
    ]

    model = GestureNetwork(output_dict).to(device)
    print(model)

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    for epoch in range(5):
        print(f"-------------- Training Epoch {epoch + 1} Start --------------\n")

        running_loss = 0.0
        for i, data in enumerate(loader[0], 0):
            inputs, labels = data
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            if i % 5 == 4:
                print(f'[{epoch + 1}, {i + 1:3d}] loss: {running_loss:.3f}')
                running_loss = 0.0

        print(f'Accuracy: {100 * verify(loader[1], model):.3f} %')
        print(f"\n-------------- Training Epoch {epoch + 1} Ends --------------\n")

    print('Finished Training')

    print(f'Test Set Accuracy: {100 * verify(loader[2], model):.3f} %')
    torch.save({'param': model.state_dict(), 'mapping': output_dict}, training_model_file)
