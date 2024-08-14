#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import sys
import os
from tqdm import tqdm
BASE = os.getcwd()

sys.path.append(os.path.join(BASE, "src"))
sys.path.append(os.path.join(BASE, "src", "model_linedet"))
sys.path.append(os.path.join(BASE, "src", "model_linedet", "model_yolox"))
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *
import DQLL
from models.yolox import Detector
from yolox_config import opt
det_model = Detector(opt)
loc_model = DQLL.localizator()

font = cv2.FONT_HERSHEY_SIMPLEX  # 글꼴 스타일
font_scale = 0.4  # 글꼴 크기
thickness = 1  # 텍스트 굵기
EVENT = 0
HFOV = 110
WIDTH = 640
HEIGHT = 480
ASPECT_RATIO = 4 / 3
CAM_TILT = -45
POS_RATIO = 0.25
SIMILARITY_LIMIT = [0.75, 0.75, 0.6, 0.6]

class VizCamera:
    def __init__(self, height, width):
        self.h, self.w = height, width
        self.s_points = np.array([(0, self.h),(self.w, self.h),(self.w/4, self.h),(self.w*3/4, self.h),(0, self.h*3/4),(self.w, self.h*3/4)]).astype(np.int16)
        self.fov = np.radians(HFOV)
        self.predictions = []
        self.polypoints = []
        self.landmarksX, self.landmarksY = [], []
        self.similaritys = []
        self.targetpoly, self.t_targetpoly = [], []

    def get_predictions(self, predictions):
        self.predictions = predictions

    def get_polyline(self, landmarksX, landmarksY):
        self.landmarksX = landmarksX
        self.landmarksY = landmarksY

    def drawing(self, bgr_image, pitch):
        self.cy = self.h//2 + self.w/(2*np.tan(self.fov/2))*np.tan(pitch)
        self.cx = self.w//2
        # draw bounding boxes
        for l,c,b in self.predictions:
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            y1 += self.h//2
            y2 += self.h//2
            bgr_image = cv2.rectangle(bgr_image, (x1,y1),(x2,y2),(0,255,0),2)
        # draw baselines
        # for tmp_x, tmp_y in self.s_points:
        #     m = (self.cy - tmp_y) / (self.cx - tmp_x)
        #     uy = int(max(0,self.cy))
        #     ux = int((uy - self.cy)/m + self.cx)
        #     bgr_image = cv2.line(bgr_image, (tmp_x,tmp_y), (ux,uy), (255,0,255),2)
        # draw track lines
        for landmarkX, landmarkY in zip(self.landmarksX, self.landmarksY):
            # draw landmarks
            for x,y in zip(landmarkX, landmarkY):
                x,y = int(x), int(y)
                bgr_image = cv2.line(bgr_image,(x,y),(x,y),(0,0,255), 5)
                # cv2.circle(bgr_image, (x, y), 7, (0,0,255), -1)
        return bgr_image


def scaling_bbox(predictions, w, h, scale_limit = 0.15, boundary = 0.75):
    scaled_predictions = []
    for item in predictions:
        _, _, b = item
        x1,y1,x2,y2 = b
        pad_x1, pad_y1, pad_x2, pad_y2 = x1, y1, w-x2, h-y2
        upleft_scale = min(pad_x1/(x2-x1), pad_y1/(y2-y1), scale_limit)
        downright_scale = min(pad_x2/(x2-x1), pad_y2/(y2-y1), scale_limit)
        # 1.5 : width comprehension
        b[0] = max(x1 - (x2-x1)*upleft_scale*1., 0.0)
        b[1] = y1 - (y2-y1)*upleft_scale
        b[2] = min(x2 + (x2-x1)*downright_scale*1., float(w-1))
        b[3] = y2 + (y2-y1)*downright_scale
        # if b[0] >= w*side_decision and b[2] <= w*(1-side_decision):
        if b[3] >= h*boundary:
            scaled_predictions.append(item)
    return scaled_predictions

def object_localize(np_img, predictions):
    if len(predictions)==0:
        return [],[],[]
    h, w, _ = np_img.shape
    predictions = scaling_bbox(predictions, w, h)
    np_labels = []
    crop_imgs = []
    crop_coords = []
    crop_ratios = []

    for label, conf, b in predictions:
        x1,y1,x2,y2 = np.array(b).astype(np.uint16)
        w_ratio, h_ratio = (x2-x1)/100, (y2-y1)/100
        crop = np_img[y1:y2+1, x1:x2+1,:]
        crop = cv2.resize(crop, (100,100))
        crop_imgs.append(crop)
        crop_coords.append((x1,y1))
        crop_ratios.append((w_ratio,h_ratio))
        np_labels.append(label)

    landmarksX, landmarksY = loc_model.inference(crop_imgs, np_labels, crop_coords, crop_ratios)
    return landmarksX, landmarksY, predictions
    
def draw_tracklines(bgr_img, vizcam, predictions, landmarksX, landmarksY):

    vizcam.get_predictions(predictions)
    vizcam.get_polyline(landmarksX, landmarksY)
    bgr_img = vizcam.drawing(bgr_img, CAM_TILT)
        # write information on the window
    return bgr_img

def main():
    os.makedirs('drone_video_viz', exist_ok=True)
    file_name = '20240304_152954'
    video = cv2.VideoCapture(f'drone_video/{file_name}.mp4')
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    flag = False
    pbar = tqdm(total=total_frames)
    fps = int(video.get(cv2.CAP_PROP_FPS))
    print(fps)
    while True:
        ret, bgr_image = video.read()
        if not flag:
            height, width, _ = bgr_image.shape
            vizcam = VizCamera(height, width)
            out = cv2.VideoWriter(f'drone_video_viz/{file_name}_viz.mp4', fourcc, fps, (width, height))
            flag = True
        pbar.update(1)
        if not ret:
            break
        h, w, _ = bgr_image.shape
        bgr_image_lower = bgr_image[h//2:,:,:]
        rgb_image_lower =  cv2.cvtColor(bgr_image_lower, cv2.COLOR_BGR2RGB)
        predictions = det_model.run(bgr_image_lower, vis_thresh=0.3)
        landmarksX, landmarksY, predictions = object_localize(rgb_image_lower, predictions)
        landmarksY = np.array(landmarksY)
        landmarksY += h//2

        drawing_img = draw_tracklines(bgr_image, vizcam, predictions, landmarksX, landmarksY)
        out.write(drawing_img)
    pbar.close()
    video.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    
