import torch
import cv2
import torchvision
import os
import numpy as np
import torch.nn as nn
import DRL_model
from PIL import Image
from config import cfg
from torchvision.models.detection import FasterRCNN_MobileNet_V3_Large_320_FPN_Weights
import reward
MEAN = np.array([0.485, 0.456, 0.406])
STD = np.array([0.229, 0.224, 0.225])
model_path = '/home/base/Workspace/ros_ws/customModule/DQLL_weights'
from config import cfg

class localizator:
    def __init__(self, cuda = True):
        self.net = DRL_model.getModel(cfg)
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
            landmark = []
            img = (img / 255. - MEAN) / STD
            w_ratio, h_ratio = ratio
            x0, y0 = coord

            if label == 'upright':
                initMarkX = [81.0, 65.0, 51.0, 36.0, 21.0]
            else:
                initMarkX = [21.0, 36.0, 51.0, 65.0, 81.0]
            cur_x_list = [[] for _ in range(cfg.LANDMARK_NUM)]
            step = 0
            status = 1

            fea_t = img
            fea_t = np.transpose(fea_t, (2, 0, 1))
            fea_t = fea_t.astype(np.float32)
            fea_t = fea_t.reshape((1,fea_t.shape[0],fea_t.shape[1],fea_t.shape[2]))
            fea_t = torch.from_numpy(fea_t).cuda()
            cur_points = initMarkX

            for cur_x, cur_point in zip(cur_x_list, cur_points):
                cur_x.append(cur_point)

            hist_vecs = np.repeat(np.zeros([cfg.HIS_NUM*cfg.ACT_NUM]).reshape(1,-1), cfg.LANDMARK_NUM, axis=0)
            states = [reward.get_state(cur_point, hist_vec) for cur_point, hist_vec in zip(cur_points, hist_vecs)]
            states = np.array(states)

            while (status == 1) & (step < 10):
                step += 1
                sta_list = np.zeros((cfg.LANDMARK_NUM, cfg.HIS_NUM*cfg.ACT_NUM+1),dtype=np.float32)
                for i in range(cfg.LANDMARK_NUM):
                    sta_list[i, :] = states[i]
                sta_list = torch.from_numpy(sta_list).unsqueeze(0).cuda()
                qvals = np.squeeze(self.net(fea_t, sta_list).detach().cpu().numpy())
                new_states, new_cur_points, new_hist_vecs = [], [], []
                for k, (qval, cur_point, hist_vec, cur_x) in enumerate(zip(qvals, cur_points, hist_vecs, cur_x_list)):
                    action = (np.argmax(qval)) + 1
                    if action != 4:
                        if action == 1:
                            cur_point = -20
                        elif action == 2:
                            cur_point -= 3
                        elif action == 3:
                            cur_point += 3
                        cur_x.append(cur_point)
                    else:
                        status = 0
                    if cfg.HIS_NUM != 0:
                        hist_vec = reward.update_history_vector(
                            hist_vec, action)
                    state = reward.get_state(cur_point, hist_vec)
                    new_states.append(state)
                    new_cur_points.append(cur_point)
                    new_hist_vecs.append(hist_vec)
                states = np.array(new_states)
                cur_points = np.array(new_cur_points)
                hist_vecs = np.array(new_hist_vecs)

            xpoint = cur_points
            ypoint = self.initMarky
            xpoint, ypoint = np.array(xpoint)*w_ratio + x0, np.array(ypoint)*h_ratio + y0
            landmarks.append(((x,y) for x,y in zip(xpoint, ypoint)))

        return landmarks