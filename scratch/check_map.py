import yaml
import cv2
import numpy as np

# Load yaml
with open('/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/opil_factory.yaml', 'r') as f:
    map_data = yaml.safe_load(f)

img_path = '/home/mohamed-azimal/ros2_ws/src/amr_robot/maps/' + map_data['image']
img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
h, w = img.shape

origin = map_data['origin']
resolution = map_data['resolution']

# Coordinates to check
coords = [
    ("spawn/dock", 4.0, 4.0),
    ("station_A_001", -4.0, 2.0),
    ("station_A_002", -4.0, -2.0),
    ("station_B_002", 4.0, -2.0),
    ("station_B_001", 3.33, 2.13),
    # Let's also check the actual station poses from world file
    ("station_A_001_world", -4.88, 2.0),
    ("station_A_002_world", -4.88, -2.0),
    ("station_B_001_world", 4.88, 2.0),
    ("station_B_002_world", 4.88, -2.0),
]

print(f"Map dimensions: {w}x{h}, origin: {origin}, resolution: {resolution}")
for name, x, y in coords:
    col = int(round((x - origin[0]) / resolution))
    # Note: ROS occupancy grid y-axis is inverted relative to OpenCV image rows (0,0 is bottom-left in ROS, top-left in OpenCV)
    # So row = h - 1 - int(round((y - origin[1]) / resolution))
    row = h - 1 - int(round((y - origin[1]) / resolution))
    
    if 0 <= col < w and 0 <= row < h:
        val = img[row, col]
        # 0 = occupied (black), 255 = free (white), 205 = unknown (gray)
        status = "FREE" if val > 250 else ("OCCUPIED" if val < 10 else f"UNKNOWN/OTHER ({val})")
        print(f"{name:20} at ({x:6.2f}, {y:6.2f}) -> pixel (col={col}, row={row}): val={val} -> {status}")
    else:
        print(f"{name:20} at ({x:6.2f}, {y:6.2f}) -> OUT OF BOUNDS")
