import torch
import cv2
import torchvision
import os
import numpy as np
import torch.nn as nn
import DRL_model
from PIL import Image
from torchvision.models.detection import FasterRCNN_MobileNet_V3_Large_320_FPN_Weights
MEAN = np.array([0.485, 0.456, 0.406])
STD = np.array([0.229, 0.224, 0.225])
model_path = '/home/base/Workspace/ros_ws/customModule/DQLL_weights'


class localizator:
    def __init__(self, cuda = True):
        self.net = DRL_model.getModel()
        net_path = os.path.join(model_path,'localizator.pth')
        saved_state = torch.load(net_path)
        self.net.load_state_dict(saved_state['weights'])
        self.net.eval()
        if cuda:
            self.net.cuda() if torch.cuda.is_available() else self.net.cpu()
        self.initMarky = [11.0, 31.0, 51.0, 71.0, 91.0]


    def inference(self, imgs, labels, coords, ratios):
        landmarks = []
        for img,label,coord,ratio in zip(imgs,labels,coords,ratios):
            w_ratio, h_ratio = ratio
            x0, y0 = coord

            fea_t = (img / 255. - MEAN) / STD
            fea_t = np.transpose(fea_t, (2, 0, 1))
            fea_t = fea_t.astype(np.float32)
            fea_t = fea_t.reshape((1,fea_t.shape[0],fea_t.shape[1],fea_t.shape[2]))
            fea_t = torch.from_numpy(fea_t).cuda()
            pred = self.net(fea_t)

            pred = pred.squeeze().cpu().detach().numpy().astype(np.uint16) * 100
            ypoint = self.initMarky
            xpoint, ypoint = np.array(xpoint)*w_ratio + x0, np.array(ypoint)*h_ratio + y0
            landmarks.append(((x,y) for x,y in zip(xpoint, ypoint)))

        return landmarks