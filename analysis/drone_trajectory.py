import pandas as pd
import numpy as np
import cv2
import os
import plotly.graph_objects as go
import time

DATA_PATH = 'drone_data'
file_name = 'drone_data(1).csv'
freq = 10
HEIGHT, WIDTH = 500, 600
scale = 2

def get_trackmap(df, h, w, start):
    track_map = np.ones((h,w,3), dtype=np.uint8) * 255
    track_map = draw_track(track_map, df, start)
    for point in df[['x','y']].values:
        x,y = point
        tmp_point = (x*scale + start[0], y*scale + start[1])
        tmp_point = np.array(tmp_point).astype(np.uint16)
        # cv2.circle(track_map, (tmp_point[0], tmp_point[1]), radius=3, color=(255, 0, 0), thickness=-1)
        cv2.line(track_map, (tmp_point[0], tmp_point[1]), (tmp_point[0], tmp_point[1]), color=(255, 0, 0), thickness=2)

    return track_map

def draw_track(track_map, df, start):
    for i in range(len(df.values)):
        tmp_point = (start[0] + i*scale, start[1])
        tmp_point = np.array(tmp_point).astype(np.uint16)
        cv2.line(track_map, (tmp_point[0], tmp_point[1]), (tmp_point[0], tmp_point[1]), color=(0, 255, 0), thickness=3)
        if i >=120:
            break

    # r = 350/np.pi
    r = 105
    cx,cy = tmp_point[0], tmp_point[1]+r
    for angle in range(180):
        dx,dy = np.cos(np.radians(angle)-np.pi/2)*r, np.sin(np.radians(angle)-np.pi/2)*r
        tmp_point = (int(cx+dx), int(cy+dy))
        cv2.line(track_map, (tmp_point[0], tmp_point[1]), (tmp_point[0], tmp_point[1]), color=(0, 255, 0), thickness=3)
    
    next_point = tmp_point
    for i in range(len(df.values)):
        tmp_point = (next_point[0] - i*scale, next_point[1])
        tmp_point = np.array(tmp_point).astype(np.uint16)
        cv2.line(track_map, (tmp_point[0], tmp_point[1]), (tmp_point[0], tmp_point[1]), color=(0, 255, 0), thickness=3)
        if i >=120:
            break
    
    cx,cy = tmp_point[0], tmp_point[1]-r
    for angle in range(180):
        dx,dy = np.cos(-np.radians(angle)-np.pi/2)*r, np.sin(-np.radians(angle)-np.pi/2)*r
        tmp_point = (int(cx+dx), int(cy+dy))
        cv2.line(track_map, (tmp_point[0], tmp_point[1]), (tmp_point[0], tmp_point[1]), color=(0, 255, 0), thickness=3)
    return track_map


def draw_trajectory(df, start = [150,150]):
    frames = []
    track_map = get_trackmap(df, HEIGHT, WIDTH, start)
    current_position = start.copy()
    for vel in df[['velx','vely']].values:
        velx, vely = vel
        current_position[0] += (velx / freq)*scale
        current_position[1] += (vely / freq)*scale
        tmp_point = np.array(current_position).astype(np.uint16)
        # cv2.circle(track_map, (tmp_point[0], tmp_point[1]), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.line(track_map, (tmp_point[0], tmp_point[1]), (tmp_point[0], tmp_point[1]), color=(0, 0, 255), thickness=2)
        frames.append(cv2.flip(track_map, 0))
    return frames, cv2.flip(track_map,0)

def is_exist_file(file_name, path):
    src_files = os.listdir(path)
    for f in src_files:
        src_name = os.path.splitext(f)[0]
        target_name = os.path.splitext(file_name)[0]
        if src_name == target_name:
            print(f'{file_name} is exist')
            return True
    return False

def main():
    file_names = [f for f in os.listdir(os.path.join(DATA_PATH)) if '.csv' in f]

    for file_name in file_names:
        if is_exist_file(file_name, os.path.join(DATA_PATH, 'image')):
            continue
        csv_path = os.path.join(DATA_PATH, file_name)
        df = pd.read_csv(csv_path)
        frames, track_map = draw_trajectory(df)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_name = os.path.splitext(file_name)[0] + '.mp4'
        image_name = os.path.splitext(file_name)[0] + '.png'
        os.makedirs(os.path.join(DATA_PATH, 'video'), exist_ok=True)
        os.makedirs(os.path.join(DATA_PATH, 'image'), exist_ok=True)
        os.makedirs(os.path.join(DATA_PATH, 'plot'), exist_ok=True)
        # save video
        video = cv2.VideoWriter(os.path.join(DATA_PATH, 'video', video_name), fourcc, 10, (WIDTH, HEIGHT))
        for i, (frame, euler) in enumerate(zip(frames, df[['roll','pitch','yaw']].values)):
            roll, pitch, yaw = euler
            frame = cv2.putText(frame,f'roll(deg):{round(np.degrees(roll),5)}',(450,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
            frame = cv2.putText(frame,f'pitch(deg):{round(np.degrees(pitch),5)}',(450,60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
            frame = cv2.putText(frame,f'yaw(deg):{round(np.degrees(yaw),5)}',(450,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
            video.write(frame)
        video.release()
        # # save image
        cv2.imwrite(os.path.join(DATA_PATH, 'image', image_name), track_map)
        # save plot
        fig = go.Figure()
        fig.add_trace(go.Scatter(y=df['h_error'].values, name='error', line=dict(color='blue')))
        fig.add_trace(go.Scatter(y=df['yaw'].values, name='yaw', line=dict(color='red')))
        fig.update_layout(title='horizontal error & angle',
                        xaxis_title='Index',
                        yaxis_title='Value')
        fig.write_image(os.path.join(DATA_PATH, 'plot', image_name))
if __name__ == '__main__':
    main()
