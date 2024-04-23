# coding=utf-8
# ----tuzixini@gmail.com----
# WIN10 Python3.6.6
# 用途: DRL_Lane Pytorch 实现
# model.py

import torch.nn as nn
import pdb
import torch
import torch.nn.functional as F
import math
from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
from torchvision.models import mobilenet_v3_large, MobileNet_V3_Large_Weights

class DRL_LANE(nn.Module):
    def __init__(self, cfg):
        super(DRL_LANE,self).__init__()
        # 使用默认的strid和padding
        self.landmark_num = cfg.LANDMARK_NUM
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 48, kernel_size=3),
            nn.ReLU(),
            nn.Conv2d(48, 96, kernel_size=3),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            nn.Conv2d(96, 128, kernel_size=2),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            nn.Conv2d(128, 192, kernel_size=2),
            nn.ReLU(),
            nn.Conv2d(192, 256, kernel_size=2),
            nn.ReLU())
        
        self.compress = nn.Sequential(
            nn.Conv2d(256, 128, kernel_size=1),
            nn.ReLU())

        qhead_list = []
        for _ in range(self.landmark_num):
            qhead_list.append(QHead(56448))
        self.qhead_list = nn.ModuleList(qhead_list)

    def forward(self, img, states):
        img = self.encoder(img)
        img = self.compress(img)
        img = img.view(img.shape[0],-1)
        qvals = []
        for i in range(self.landmark_num):
            qhead = self.qhead_list[i]
            state = states[:,i,...]
            q = qhead(img, state)
            qvals.append(q)
        qvals = torch.stack(qvals,dim=1)
        return qvals


class Mobilenet3_DRL(nn.Module):
    def __init__(self, cfg, size = 'large'):
        super(Mobilenet3_DRL,self).__init__()
        self.landmark_num = cfg.LANDMARK_NUM
        if size == 'small':
            weights = MobileNet_V3_Small_Weights.DEFAULT
            model = mobilenet_v3_small(weights=weights)
            node_num = 576
        else:
            weights = MobileNet_V3_Large_Weights.DEFAULT
            model = mobilenet_v3_large(weights=weights)
            node_num = 960
        self.model = nn.Sequential(*list(model.children())[:-1])
        qhead_list = []
        for _ in range(self.landmark_num):
            qhead_list.append(QHead(node_num))
        self.qhead_list = nn.ModuleList(qhead_list)
    def forward(self, x, states):
        img = self.model(x)
        img = img.view(img.size(0), -1)
        qvals = []
        for i in range(self.landmark_num):
            qhead = self.qhead_list[i]
            state = states[:,i,...]
            q = qhead(img, state)
            qvals.append(q)
        qvals = torch.stack(qvals,dim=1)
        return qvals
    

# class QHead(nn.Module):
#     def __init__(self, node_num):
#         super(QHead, self).__init__()
#         self.fc1 = nn.Sequential(
#             nn.Linear(node_num, 512),
#             nn.ReLU())
#         self.fc2 = nn.Sequential(
#             nn.Linear(545, 256),
#             nn.ReLU())
#         self.fc3 = nn.Sequential(
#             nn.Linear(256, 4))

#     def forward(self, img, state):
#         img = self.fc1(img)
#         img = torch.cat((img, state),1)
#         img = self.fc2(img)
#         img = self.fc3(img)
#         return img

class QHead(nn.Module):
    def __init__(self, node_num):
        super(QHead, self).__init__()
        self.fc1 = nn.Sequential(
            nn.Linear(node_num+33, 256),
            nn.ReLU())
        self.fc2 = nn.Sequential(
            nn.Linear(256, 4))

    def forward(self, img, state):
        img = torch.cat((img, state),1)
        img = self.fc1(img)
        img = self.fc2(img)
        return img

def getModel(cfg):
#     net = DRL_LANE(cfg)
    net = Mobilenet3_DRL(cfg)
    return net