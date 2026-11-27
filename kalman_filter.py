import numpy as np 
import matplotlib.pyplot as plt

from tdmclient import ClientAsync
import asyncio, threading, time



# thymio data and variables

SPEED_TO_MM_S = 0.335    # [Thymio raw unit -> mm/s] -> tune it on our robot (~0.3-0.4) TUNING
WHEEL_BASE = 95.0   # [mm] -> distance between wheels -> tune it on our robot TUNING
LOOP_DT = 0.01  # [s] -> target loop period for the filter
GATE = 11.34    # threshold for innovation -> chi-square (3 DoF) for 99% 

Q_RATE = np.diag([8.30418771, 8.30418771, 1e-3]) # process noise per second TUNING
MAX_AGE = 0.3 # [s] -> ignore camera frames older than this (at 30fps its reasonable)

R_TUNED = np.diag([0.05869203500000153, 0.05094229499999994, 9.027500000000001e-05])


LOG_BOOL = True
LOG_TIME = 0.05 # [s]
V_TEST = 20.0  # [mm/s]


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
    l = node.v.motor.left.speed
    r = node.v.motor.right.speed
    if l is None or r is None:
        return 0.0, 0.0
    return l * SPEED_TO_MM_S, r * SPEED_TO_MM_S


  
def get_camera_pose(): # test version
    t = time.perf_counter() - _t0
    return np.array([V_TEST * t, 0.0, 0.0])
    
    



# =======================================================================================
# =======================================================================================


running = False
cam_running = False

log = None
last_stamp = None
ekf = None
latest = None
node = None
client = None


async def kalman_loop():
    global last_stamp, running, client, ekf, log, node
    
    t0 = t_prev = time.perf_counter()
    next_log = 0.0
    
    try:
        while running:
            await asyncio.sleep(LOOP_DT)
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
            log["vl"].append(v_l)
            log["vr"].append(v_r)
            
            
            if LOG_BOOL and (t - t0 >= next_log):
                next_log = (t - t0) + LOG_TIME
                # next_log += LOG_TIME
                z_str = "none" if z is None else np.round(z, 1)
                print(f"t={t - t0:.3f}s  x={np.round(ekf.x, 1)}  z={z_str}  rejected={rejected}")
                
    except Exception as e:
        print("Main loop crashed : ", repr(e))
        raise




async def start_kalman(cl, nd):
    global log, running, last_stamp, ekf, latest, client, node, cam_running, _t0    
    
    
    # setting up thymio variables 
    
    client = cl
    node = nd
    
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
    
    # camera thread
    
    latest = {"z": None, "stamp": 0, "t": 0.0} 
    # latest = {"z": np.array([0, 0, 0]), "stamp": 0, "t": 0.0} 
    
    cam_running = True

    def camera_worker():
        import Vision
        while cam_running:
            # z = Vision.get_current_thymio_pose()
            z = get_camera_pose()
            if z is not None:
                latest["z"] = z
                latest["t"] = time.perf_counter()
                latest["stamp"] += 1 
            time.sleep(1)
            # time.sleep(1 / 30)   # camera frame rate

    _t0 = time.perf_counter()
    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start() # thread


    # initialize filter 

    while latest["z"] is None:
        await asyncio.sleep(0.05)

    ekf = EKF(x0=latest["z"],
            P0=np.diag([25, 25, 0.01]),
            Q=np.zeros((3, 3)),
            R=R_TUNED,
            wheel_base=WHEEL_BASE,
            H=np.eye(3))
    
    running = True
    log = {"t": [], "x": [], "z": [], "Pdiag": [], "rejected": [], "vl" : [], "vr" : []}
    last_stamp = 0
    
    # launch main loop
    
    task = asyncio.create_task(kalman_loop())
    return task
    
    
async def stop_kalman(task):
    global running, cam_running
    
    running = False
    try:
        await task
    finally:
        cam_running = False
    
    return log