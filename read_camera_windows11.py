import airsim
import numpy as np
import cv2
import socket
import struct
import pprint
import math
# WSL의 IP 주소 및 포트 설정
WSL_IP = '172.25.113.219'  # WSL IP 주소로 수정하세요
PORT = 50101

# TCP 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((WSL_IP, PORT))
# sock.bind((WSL_IP, PORT))
# sock.listen(1)
# conn, addr = sock.accept()
pp = pprint.PrettyPrinter(indent=4)

client = airsim.MultirotorClient()
camera_info = client.simGetCameraInfo(str(2))
image_width = 480
fov_horizontal = camera_info.fov * np.pi / 180
print("Horizontal FOV:", camera_info.fov)

focal_length = image_width / (2.0 * np.tan(fov_horizontal / 2.0))
print("CameraInfo")
pp.pprint(camera_info)
print("focal length : ",focal_length)
while True:
    try:
        # AirSim에서 이미지 요청
        responses = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)])
        response = responses[0]

        camera_info = client.simGetCameraInfo(str(2))
        pose = camera_info.pose
        orientation = pose.orientation
        orientation_q = airsim.Quaternionr(orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val)
        # 쿼터니언을 오일러 각도로 변환 (라디안)
        pitch, roll, yaw = airsim.to_eularian_angles(orientation_q)
        yaw += np.pi/2
        # 피치 각도를 도(degree)로 변환
        yaw_deg = math.degrees(yaw)
        pitch_deg = math.degrees(pitch)
        roll_deg = math.degrees(roll)
        # 이미지 데이터 추출
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        print(img_rgb.shape)
        print(f'yaw : {yaw_deg},({yaw}), pitch : {pitch_deg}, roll : {roll_deg}')
        # 이미지를 JPEG로 인코딩
        _, img_encoded = cv2.imencode('.jpg', img_rgb)
        data = img_encoded.tobytes()

        # 데이터 크기 전송
        # sock.sendall(struct.pack('>I', len(data)))

        # # 이미지 데이터 전송
        # sock.sendall(data)
        # 데이터 크기 전송
        message = struct.pack('>I', len(data)) + data + struct.pack('>fff', yaw, pitch, roll)
        sock.sendall(message)
        # sock.sendall(struct.pack('>I', len(data)))
        # cv2.imshow("object", img_rgb)                             # 使用OpenCV显示处理后的图像效果
        # cv2.waitKey(10)
        # # 이미지 데이터 전송
        # sock.sendall(data)
        # sock.sendall(struct.pack('>ff', pitch, roll))
    except KeyboardInterrupt:
        break

sock.close()
