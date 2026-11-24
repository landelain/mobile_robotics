import numpy as np 
import matplotlib.pyplot as plt

from tdmclient import ClientAsync
import asyncio, threading, time
import Vision



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


def get_camera_pose():

    return Vision.get_current_thymio_pose()



# =======================================================================================
# =======================================================================================



log = None
last_stamp = None
ekf = None
latest = None
node = None
client = None


async def kalman_loop():
    global last_stamp, running, client, ekf, log, node
    
    t0 = t_prev = time.perf_counter()
    
    try:
        while running:
            await client.sleep(LOOP_DT)
            t = time.perf_counter()
            dt, t_prev = t - t_prev, t

            # predict from wheel odometry
            v_l, v_r = read_wheels_speed(node)
            ekf.Q = Q_RATE * dt
            ekf.predict(v_l, v_r, dt)

            # update with new/recent camera observation
            z, rejected = None, False
            if latest["stamp"] != last_stamp and t - latest["t"] < MAX_AGE:
                last_stamp = latest["stamp"]
                z = latest["z"].copy()
                rejected = not ekf.update(z)

            # log from the filters
            log["t"].append(t - t0)
            log["x"].append(ekf.x.copy())
            log["z"].append(z if z is not None else np.full(3, np.nan))
            log["Pdiag"].append(np.diag(ekf.P).copy())
            log["rejected"].append(rejected)
            
    except Exception as e:
        print("Main loop crashed : ", repr(e))
        raise




async def start_kalman(cl, nd):
    global log, running, last_stamp, ekf, latest, client, node
    
    
    # setting up thymio variables 
    
    client = cl
    node = nd
    
    # camera thread
    
    latest = {"z": None, "stamp": 0, "t": 0.0} 
    cam_running = True

    def camera_worker():
        while cam_running:
            z = get_camera_pose()
            if z is not None:
                latest["z"] = z
                latest["t"] = time.perf_counter()
                latest["stamp"] += 1 

    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start() # thread

    # initialize filter 

    while latest["z"] is None:
        await asyncio.sleep(0.05)

    ekf = EKF(x0=latest["z"],
            P0=np.diag([25, 25, 0.01]),
            Q=np.zeros((3, 3)),
            R=np.diag([9, 9, 0.005]),
            wheel_base=WHEEL_BASE,
            H=np.eye(3))
    
    running = True
    log = {"t": [], "x": [], "z": [], "Pdiag": [], "rejected": []}
    last_stamp = 0
    
    # launch main loop
    
    task = asyncio.create_task(kalman_loop())
    return task
    
    
async def stop_kalman(task):
    global running, cam_running
    
    running = False
    await task              
    cam_running = False
    