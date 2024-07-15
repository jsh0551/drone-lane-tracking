import os
import numpy as np
import cv2
from tf_transformations import quaternion_from_euler
from scipy.spatial.distance import directed_hausdorff

def adjustable_scale(x, alpha = 5):
    return x**3 + alpha*x

def calculate_quaternion(yaw_deg, pitch_deg, roll_deg):
    """Calculate quaternion from yaw, pitch, and roll in degrees."""
    yaw = np.radians(yaw_deg)
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def calculate_curvature(a, b, c, x):
    # 1차 도함수 y' = 2ax + b
    y_prime = 2 * a * x + b
    
    # 2차 도함수 y'' = 2a
    y_double_prime = 2 * a
    
    # 곡률 공식 적용
    kappa = np.abs(y_double_prime) / (1 + y_prime**2)**(3/2)
    
    return kappa

def calculate_slope(bot_point, pitch, width, height):
    # principle point
    cx = width/2
    cy = height/2 + 240*np.tan(pitch)
    # cal slope
    bx, by = bot_point
    slope = (cy-by) / (cx-bx)
    return slope

def affine_transform(polyline, slope):
    t_polyline = np.zeros_like(polyline)
    t_polyline[:,0] = polyline[:,0] - polyline[:,1]*(1/slope)
    t_polyline[:,1] = polyline[:,1]
    return t_polyline

def get_new_filename(file_path):
    """
    Check if a file exists and generate a new filename with a numeric suffix if it does.
    """
    base, ext = os.path.splitext(file_path)
    counter = 1

    new_file_path = f"{base}({counter}){ext}"
    while os.path.exists(new_file_path):
        counter += 1
        new_file_path = f"{base}({counter}){ext}"
    
    return new_file_path

def nms(boxes, iou_threshold):
    if not boxes:
        return []

    # Extract the coordinates and confidences from the boxes
    labels = np.array([box[0] for box in boxes])
    confidences = np.array([box[1] for box in boxes])
    bboxes = np.array([box[2] for box in boxes])
    whs = np.array([(x2-x1+1)*(y2-y1+1) for x1,y1,x2,y2 in bboxes])

    x1 = bboxes[:, 0]
    y1 = bboxes[:, 1]
    x2 = bboxes[:, 2]
    y2 = bboxes[:, 3]

    # Compute the area of the bounding boxes and sort by confidences
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = confidences.argsort()[::-1]
    # order = whs.argsort()[::-1]

    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(boxes[i])

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return keep

class PIDController():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
    
    def compute(self, error):
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    
class KalmanFilterPolylineTracker:
    def __init__(self, initial_polyline):
        self.n_points = len(initial_polyline)
        self.kalman_filters = []

        for point in initial_polyline:
            kf = cv2.KalmanFilter(4, 2)
            kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0]], np.float32)
            kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                            [0, 1, 0, 1],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]], np.float32)
            kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
            kf.statePre = np.array([point[0], point[1], 0, 0], np.float32)
            self.kalman_filters.append(kf)

    def predict(self):
        return [kf.predict()[:2].flatten() for kf in self.kalman_filters]

    def correct(self, measurements):
        return [kf.correct(m.astype(np.float32).reshape(-1, 1))[:2].flatten() 
                for kf, m in zip(self.kalman_filters, measurements)]
    
def normalize_polyline(polyline):
    """
    Normalize a polyline by moving it to start at (0,0) and scaling to fit in a 1x1 box.
    """
    polyline = np.array(polyline)
    # Move to start at (0,0)
    normalized = polyline - polyline[0]
    # Scale to fit in 1x1 box
    max_dim = np.max(np.abs(normalized))
    if max_dim != 0:
        normalized = normalized / max_dim
    return normalized

def resample_polyline(polyline, num_points):
    """Resample a polyline to have a specific number of points."""
    polyline = np.array(polyline)
    if polyline.ndim == 1:
        # If input is 1D, reshape it to 2D
        if len(polyline) % 2 != 0:
            raise ValueError("1D polyline must have even number of elements")
        polyline = polyline.reshape(-1, 2)
    elif polyline.ndim != 2 or polyline.shape[1] != 2:
        raise ValueError("Polyline must be a 2D array with shape (n, 2)")
    
    if len(polyline) < 2:
        raise ValueError("Polyline must have at least 2 points")
    
    cumulative_distances = np.cumsum(np.sqrt(np.sum(np.diff(polyline, axis=0)**2, axis=1)))
    cumulative_distances = np.insert(cumulative_distances, 0, 0)
    
    distances = np.linspace(0, cumulative_distances[-1], num_points)
    return np.array([np.interp(distances, cumulative_distances, polyline[:, i]) for i in range(2)]).T

def calculate_similarity(polyline1, polyline2, num_points=100):
    """
    Calculate the similarity between two polylines, considering only their shape.
    
    Returns:
    - similarity: A float between 0 and 1.
      1 indicates identical shapes, 
      values close to 0 indicate very different shapes.
    """
    try:
        # Normalize both polylines
        norm1 = normalize_polyline(polyline1)
        norm2 = normalize_polyline(polyline2)
        
        # Resample the normalized polylines
        resampled1 = resample_polyline(norm1, num_points)
        resampled2 = resample_polyline(norm2, num_points)
    except ValueError as e:
        print(f"Error in processing: {e}")
        print(f"polyline1 shape: {np.array(polyline1).shape}")
        print(f"polyline2 shape: {np.array(polyline2).shape}")
        return 0  # Return 0 similarity for error cases
    
    distances = np.sqrt(np.sum((resampled1 - resampled2)**2, axis=1))
    mean_distance = np.mean(distances)
    
    # Convert distance to similarity
    similarity = 1 / (1 + mean_distance)
    
    return similarity