from pathlib import Path
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
import platform

TERRAIN_WIDTH_MM = 1095.0
TERRAIN_HEIGHT_MM = 751.0
PIXELS_PER_MM = 1.0
MAP_WIDTH = round(TERRAIN_WIDTH_MM * PIXELS_PER_MM)
MAP_HEIGHT = round(TERRAIN_HEIGHT_MM * PIXELS_PER_MM)

MARKER_ROBOT = 4
MARKER_GOAL = 5
#CHANGE CAMERA INDEX DEPENDING ON COMPUTER
CAMERA_INDEX = 0
if platform.system() == "Windows":
    CAMERA_BACKEND = cv2.CAP_DSHOW
elif platform.system() == "Darwin":
    CAMERA_BACKEND = cv2.CAP_AVFOUNDATION
else:
    CAMERA_BACKEND = cv2.CAP_ANY
CAPTURE_NEW_IMAGE = False
IMAGE_PATH = Path('terrain_reference.jpg')
OUTPUT_IMAGE = Path('navigation_annotated.png')

ROBOT_LENGTH_MM = 110.0
ROBOT_WIDTH_MM = 110.0
ROBOT_EXTRA_MARGIN_MM = 15.0
MARKER_MARGIN_MM = 10.0
DARK_THRESHOLD = 150
MIN_OBSTACLE_AREA_MM2 = 1000.0
RECTANGULARITY_MIN = 0.65
MIN_SIDE_COVERAGE = 0.40
SIDE_DISTANCE_RATIO = 0.15
CORNER_DISTANCE_RATIO = 0.20

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())


#this big function takes a picture and computes the positions of everything
def initialize_vision(capture_new_image=None):
    global img, marker_shapes, H, H_mm, terrain_corrected
    global obstacle_data, obstacles_px, robot_pose, goal_pose, pixel_to_mm, map_px_to_mm
    if capture_new_image is not None:
        global CAPTURE_NEW_IMAGE
        CAPTURE_NEW_IMAGE = capture_new_image
    if CAPTURE_NEW_IMAGE:
        camera = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
        if not camera.isOpened():
            camera.release()
            raise RuntimeError('Camera unavailable')
        try:
            for _ in range(15):
                ok, frame = camera.read()
            if not ok:
                raise RuntimeError('Camera capture failed')
            img = frame.copy()
        finally:
            camera.release()
        if not cv2.imwrite(str(IMAGE_PATH), img):
            raise RuntimeError('Could not save reference image')
    elif IMAGE_PATH.exists():
        img = cv2.imread(str(IMAGE_PATH))
        if img is None:
            raise RuntimeError('Could not load reference image')
    elif 'img' not in globals() or not isinstance(img, np.ndarray):
        raise FileNotFoundError('No reference image. Set CAPTURE_NEW_IMAGE=True or provide terrain_reference.jpg')
    plt.figure(figsize=(11, 7))
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title('Image de référence — pixels caméra')
    
    marker_corners, marker_ids, _ = detector.detectMarkers(img)
    marker_shapes = {}
    if marker_ids is not None:
        for corners, marker_id in zip(marker_corners, marker_ids.flatten()):
            marker_shapes[int(marker_id)] = corners[0].astype(np.float32)

    # Marker IDs: 0 top-left, 1 top-right, 2 bottom-left, 3 bottom-right.
    required = [2, 3, 1, 0]
    missing = [i for i in required if i not in marker_shapes]
    if missing:
        raise RuntimeError(f'Missing calibration markers: {missing}')

    source = np.float32([marker_shapes[i].mean(axis=0) for i in required])
    target_mm = np.float32([
        [0, 0],
        [TERRAIN_WIDTH_MM, 0],
        [TERRAIN_WIDTH_MM, TERRAIN_HEIGHT_MM],
        [0, TERRAIN_HEIGHT_MM]
    ])
    H_mm = cv2.getPerspectiveTransform(source, target_mm)

    def pixel_to_mm(x, y):
        p = np.float32([[[x, y]]])
        result = cv2.perspectiveTransform(p, H_mm)[0, 0]
        return float(result[0]), float(result[1])
    
    #calculation of goal position
    goal_pose = None

    if MARKER_GOAL in marker_shapes:
        pts = marker_shapes[MARKER_GOAL]
        x, y = pixel_to_mm(*pts.mean(axis=0))

        goal_pose = {
            'x_mm': x,
            'y_mm': y
        }

    # Rectified image uses downward-positive image Y.
    target_px = np.float32([
        [0, MAP_HEIGHT - 1],
        [MAP_WIDTH - 1, MAP_HEIGHT - 1],
        [MAP_WIDTH - 1, 0],
        [0, 0]
    ])
    H = cv2.getPerspectiveTransform(source, target_px)
    terrain_corrected = cv2.warpPerspective(
        img, H, (MAP_WIDTH, MAP_HEIGHT)
    )

    def map_px_to_mm(x, y):
        return (
            float(x * TERRAIN_WIDTH_MM / (MAP_WIDTH - 1)),
            float((MAP_HEIGHT - 1 - y) * TERRAIN_HEIGHT_MM / (MAP_HEIGHT - 1))
        )

    exclusion_mask = np.zeros(
        (MAP_HEIGHT, MAP_WIDTH), dtype=np.uint8
    )

    # Exclude ArUco markers
    for pts in marker_shapes.values():
        pts_map = cv2.perspectiveTransform(
            pts.reshape(1, 4, 2), H
        )[0]

        cv2.fillConvexPoly(
            exclusion_mask,
            np.rint(pts_map).astype(np.int32),
            255
        )

    # Initial robot pose
    robot_pose = None

    if MARKER_ROBOT in marker_shapes:
        pts = marker_shapes[MARKER_ROBOT]
        x, y = pixel_to_mm(*pts.mean(axis=0))
        fx, fy = pixel_to_mm(*((pts[0] + pts[1]) / 2))

        robot_pose = {
            'x_mm': x,
            'y_mm': y,
            'theta_rad': math.atan2(fy - y, fx - x)
        }
    # Robot position and direction in rectified image
    robot_pts = marker_shapes[MARKER_ROBOT]

    center_px = robot_pts.mean(axis=0)
    front_px = (robot_pts[0] + robot_pts[1]) / 2

    center_map = cv2.perspectiveTransform(
        center_px.reshape(1, 1, 2).astype(np.float32), H
    )[0, 0]

    front_map = cv2.perspectiveTransform(
        front_px.reshape(1, 1, 2).astype(np.float32), H
    )[0, 0]

    forward = front_map - center_map
    forward = forward / np.linalg.norm(forward)

    right = np.array([-forward[1], forward[0]])
    half_l = (ROBOT_LENGTH_MM / 2 + ROBOT_EXTRA_MARGIN_MM) * PIXELS_PER_MM
    half_w = (ROBOT_WIDTH_MM / 2 + ROBOT_EXTRA_MARGIN_MM) * PIXELS_PER_MM

    footprint = np.array([
        center_map + sx * half_l * forward + sy * half_w * right
        for sx, sy in [(-1, -1), (-1, 1), (1, 1), (1, -1)]
        ])

    cv2.fillConvexPoly(
        exclusion_mask,
        np.rint(footprint).astype(np.int32),
        255
    )

    margin = max(1, round(MARKER_MARGIN_MM * PIXELS_PER_MM))
    k = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * margin + 1, 2 * margin + 1)
    )
    exclusion_mask = cv2.dilate(exclusion_mask, k)

    gray = cv2.cvtColor(terrain_corrected, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    threshold, binary = cv2.threshold(
        blurred, 0, 255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    print("Automatic threshold:", threshold)
    binary[exclusion_mask > 0] = 0
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles_px = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_OBSTACLE_AREA_MM2 * PIXELS_PER_MM ** 2:
            continue
        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        if rw <= 0 or rh <= 0:
            continue
        box = cv2.boxPoints(rect)
        diagonal = np.hypot(rw, rh)
        pts = contour.reshape(-1, 2).astype(float)
        fill_ratio = area / (rw * rh)
        corner_distances = [np.min(np.linalg.norm(pts - c, axis=1)) / diagonal for c in box]
        coverages = []
        for j in range(4):
            samples = np.linspace(box[j], box[(j + 1) % 4], 30)
            d = np.linalg.norm(samples[:, None, :] - pts[None, :, :], axis=2).min(axis=1)
            coverages.append(float(np.mean(d < SIDE_DISTANCE_RATIO * diagonal)))
        if (fill_ratio >= RECTANGULARITY_MIN and
                max(corner_distances) <= CORNER_DISTANCE_RATIO and
                min(coverages) >= MIN_SIDE_COVERAGE):
            obstacles_px.append({'box': box, 'center': box.mean(axis=0)})

    obstacles_px.sort(key=lambda o: (float(o['center'][1]), float(o['center'][0])))
    obstacle_data = [
        {'id': i, 'points_mm': [map_px_to_mm(*p) for p in obs['box']]}
        for i, obs in enumerate(obstacles_px, start=1)
    ]
    print('Obstacles detected:', len(obstacle_data))
    annotated = terrain_corrected.copy()
    for obs, source_obs in zip(obstacle_data, obstacles_px):
        box = np.rint(source_obs['box']).astype(np.int32)
        cv2.polylines(annotated, [box], True, (0, 255, 0), 2)
        for j, (x, y) in enumerate(box, start=1):
            cv2.circle(annotated, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.putText(annotated, f'{obs["id"]}.{j}', (int(x) + 6, int(y) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cx, cy = np.rint(source_obs['center']).astype(int)
        cv2.putText(annotated, f'Obs {obs["id"]}', (int(cx), int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # show goal
    if MARKER_GOAL in marker_shapes:
        center = marker_shapes[MARKER_GOAL].mean(axis=0)
        center = cv2.perspectiveTransform(
            center.reshape(1, 1, 2), H
        )[0, 0]

        cx, cy = np.rint(center).astype(int)

        cv2.circle(annotated, (cx, cy), 7, (255, 0, 0), -1)
        cv2.putText(annotated, 'Goal', (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
    cv2.imwrite(str(OUTPUT_IMAGE), annotated)
    plt.figure(figsize=(13,8))
    plt.imshow(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB))
    plt.title('Carte annotée — 4 points par obstacle (voir valeurs en mm cellule 7)')
    plt.axis('off')
    plt.show()
    print('Image enregistrée :', OUTPUT_IMAGE.resolve())
    if MARKER_ROBOT in marker_shapes:
        center = cv2.perspectiveTransform(marker_shapes[MARKER_ROBOT].mean(axis=0).reshape(1, 1, 2), H)[0, 0]
        cx, cy = np.rint(center).astype(int)
        cv2.circle(annotated, (cx, cy), 7, (255, 0, 255), -1)
        cv2.putText(annotated, 'Thymio', (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    cv2.imwrite(str(OUTPUT_IMAGE), annotated)


def pixel_to_mm(x, y):
        p = np.float32([[[x, y]]])
        result = cv2.perspectiveTransform(p, H_mm)[0, 0]
        return float(result[0]), float(result[1])

def get_obstacles_pose():
    return [len(obstacle_data)] + [
        [(round(float(x), 2), round(float(y), 2)) for x, y in obs['points_mm']]
        for obs in obstacle_data
    ]


def get_initial_thymio_pose():
    if robot_pose is None:
        return None
    return [round(robot_pose[k], 2) for k in ('x_mm', 'y_mm', 'theta_rad')]

def get_goal_pose():
    if goal_pose is None:
        return None
    return [round(goal_pose[k], 2) for k in ('x_mm', 'y_mm')]

def get_current_thymio_pose():
    # Each call captures a fresh frame; calibration and obstacles stay unchanged.
    camera = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
    if not camera.isOpened():
        camera.release()
        return 1
    try:
        ok, frame = camera.read()
    finally:
        camera.release()
    if not ok:
        return 2

    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None or MARKER_ROBOT not in ids.flatten():
        return 3
    idx = int(np.where(ids.flatten() == MARKER_ROBOT)[0][0])
    pts = corners[idx][0]
    x, y = pixel_to_mm(*pts.mean(axis=0))
    fx, fy = pixel_to_mm(*((pts[0] + pts[1]) / 2))
    return [round(x, 2), round(y, 2), round(math.atan2(fy - y, fx - x), 2)]