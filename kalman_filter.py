import numpy as np 
import matplotlib.pyplot as plt

from tdmclient import ClientAsync



# thymio data

SPEED_TO_MM_S = 0.32    # [Thymio raw unit -> mm/s] -> tune it on our robot (~0.3-0.4) TUNING
WHEEL_BASE = 95.0   # [mm] -> distance between wheels -> tune it on our robot TUNING
LOOP_DT = 0.05  # [s] -> target loop period for the filter
GATE = 11.34    # threshold for innovation -> chi-square (3 DoF) for 99% 

Q_RATE = np.diag([8.30418771, 8.30418771, 0]) # process noise per second TUNING
MAX_AGE = 0.3 # [s] -> ignore camera frames older than this (at 30fps its reasonable)

# class of the Extended Kalman Filter

def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

class EKF:
    

    def __init__(self, x0, P0, Q, R, wheel_base, H):
        
        self.x = np.array(x0, dtype=float) 
        self.P = np.array(P0, dtype=float)
        self.Q, self.R, self.L, self.H = Q, R, wheel_base, H
        

    def predict(self, v_l, v_r, dt):
        
        v = (v_r + v_l) / 2
        w = (v_r - v_l) / self.L
        x, y, theta = self.x
        
        self.x = np.array([x + v*np.cos(theta)*dt,y + v*np.sin(theta)*dt, wrap(theta + w*dt)])
        
        J = np.array([[1, 0, -v*np.sin(theta)*dt], [0, 1,  v*np.cos(theta)*dt], [0, 0,  1]])
        self.P = J @ self.P @ J.T + self.Q
        

    def update(self, z):      
                        
        y = z - self.H @ self.x
        y[2] = wrap(y[2])
        S = self.H @ self.P @ self.H.T + self.R
        S_inv = np.linalg.inv(S)
        
        if y @ S_inv @ y > GATE:
            return False
        
        K = self.P @ self.H.T @ S_inv
        
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - K @ self.H) @ self.P
        return True
    
    
    
    

def read_wheels_speed(node):
    vl = node.v.motor.left.speed * SPEED_TO_MM_S
    vr = node.v.motor.right.speed * SPEED_TO_MM_S
    return vl, vr