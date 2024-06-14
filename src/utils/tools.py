import numpy as np
from tf_transformations import quaternion_from_euler

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

class PIDController():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
    
    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output