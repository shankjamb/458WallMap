import time
import roslibpy
import math
import numpy as np

# Connect to rosbridge server
ros = roslibpy.Ros(host='192.168.8.104', port=9012)
ros.run()

# Create the occupancy grid topic
topic = roslibpy.Topic(ros, '/omega/map', 'nav_msgs/OccupancyGrid')

robot_name = "omega"

# LiDAR topic setup
lidar_topic_name = f'/{robot_name}/scan'
lidar_topic_type = 'sensor_msgs/LaserScan'
lidar_topic = roslibpy.Topic(ros, lidar_topic_name, lidar_topic_type)

# Store latest LiDAR scan
latest_scan = None

def lidar_callback(message):
    global latest_scan
    latest_scan = message
    print(f"Received LiDAR scan with {len(message['ranges'])} ranges.")

lidar_topic.subscribe(lidar_callback)

#Bresenham’s line algorithm
def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points

# Update occupancy grid from LiDAR
def update_grid_from_lidar(scan_msg, grid, resolution, origin):
    angle_min = scan_msg['angle_min']
    angle_increment = scan_msg['angle_increment']
    ranges = scan_msg['ranges']
    height, width = grid.shape
    ox, oy = origin

    for i, r in enumerate(ranges):
        if math.isinf(r) or math.isnan(r):
            continue

        angle = angle_min + i * angle_increment
        end_x = ox + int((r * math.cos(angle)) / resolution)
        end_y = oy + int((r * math.sin(angle)) / resolution)

        line_points = bresenham_line(ox, oy, end_x, end_y)

        for px, py in line_points[:-1]:
            if 0 <= px < width and 0 <= py < height:
                grid[py, px] = 0  # Free

        if 0 <= end_x < width and 0 <= end_y < height:
            grid[end_y, end_x] = 1  # Obstacle

    return grid

# Make grid from LiDAR scan
def make_grid():
    width, height = 200, 200  # in cells
    resolution = 0.05  # meters per cell
    grid = np.full((height, width), -1, dtype=int)  # Start with all unknowns

    # Assume robot is centered in grid
    origin = (width // 2, height // 2)

    if latest_scan:
        update_grid_from_lidar(latest_scan, grid, resolution, origin)

    # Build occupancy grid message
    grid_msg = {
        'header': {
            'frame_id': 'map',
            'stamp': {'secs': int(time.time()), 'nsecs': 0}
        },
        'info': {
            'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
            'resolution': resolution,
            'width': width,
            'height': height,
            'origin': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        },
        'data': grid.flatten().tolist()
    }

    return grid_msg

# Main loop
try:
    while ros.is_connected:
        msg = make_grid()
        topic.publish(roslibpy.Message(msg))
        print(f"Published map with {msg['info']['width']}x{msg['info']['height']} cells.")
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down...")

topic.unadvertise()
ros.terminate()
