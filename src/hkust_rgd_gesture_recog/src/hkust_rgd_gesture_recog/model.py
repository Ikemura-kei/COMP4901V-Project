from torch import nn


class GestureNetwork(nn.Module):
    def __init__(self, label_map):
        super(GestureNetwork, self).__init__()
        self.label_map = label_map
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(63, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, len(label_map)),
        )

    def forward(self, x):
        x = self.linear_relu_stack(x)
        return x

    def solve(self, x):
        values = self.linear_relu_stack(x)
        return [(self.label_map[i], values[i].item()) for i in range(len(self.label_map))]
