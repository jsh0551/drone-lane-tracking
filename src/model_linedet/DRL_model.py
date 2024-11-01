import random
import numpy as np
import cv2
import torch
import torch.nn as nn
import os
from torchvision.models import efficientnet_v2_m, EfficientNet_V2_M_Weights, efficientnet_v2_s, EfficientNet_V2_S_Weights,\
MobileNet_V3_Large_Weights, mobilenet_v3_large
points_num = 7

class Ghostnet_reg(nn.Module):
    def __init__(self, points_num=5):
        super(Ghostnet_reg, self).__init__()
        model = torch.hub.load('huawei-noah/ghostnet', 'ghostnet_1x', pretrained=True)
        self.model = nn.Sequential(*list(model.children())[:-1])
        self.reg_layer = nn.Sequential(nn.Dropout(0.3),nn.Linear(1280,points_num))
    def forward(self, x):
        img = self.model(x)
        img = img.view(img.size(0), -1)
        reg_value = self.reg_layer(img)
        return reg_value

class Efficient_V2_reg(nn.Module):
    def __init__(self, points_num=5, size='medium'):
        super(Efficient_V2_reg, self).__init__()
        if size=='small':
            weights = EfficientNet_V2_S_Weights.DEFAULT
            model = efficientnet_v2_s(weights=weights)
        else:
            weights = EfficientNet_V2_M_Weights.DEFAULT
            model = efficientnet_v2_m(weights=weights)
        self.model = nn.Sequential(*list(model.children())[:-1])
        self.reg_layer = nn.Sequential(nn.Dropout(0.3),nn.Linear(1280,points_num))
    def forward(self, x):
        img = self.model(x)
        img = img.view(img.size(0), -1)
        reg_value = self.reg_layer(img)
        return reg_value
    
class MobilenetV3_reg(nn.Module):
    def __init__(self, points_num=5):
        super(MobilenetV3_reg, self).__init__()
        weights = MobileNet_V3_Large_Weights.DEFAULT
        model = mobilenet_v3_large(weights=weights)
        self.model = nn.Sequential(*list(model.children())[:-1])
        self.reg_layer = nn.Sequential(nn.Dropout(0.3),nn.Linear(960,points_num))
    def forward(self, x):
        img = self.model(x)
        img = img.view(img.size(0), -1)
        reg_value = self.reg_layer(img)
        return reg_value  

def getModel():
#     net = DRL_LANE(cfg)
    net = MobilenetV3_reg(points_num=points_num)
    return net