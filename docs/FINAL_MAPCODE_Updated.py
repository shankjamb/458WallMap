import time
import roslibpy
import math
import threading
from pynput import keyboard

# Connect to ROS bridge server
ros = roslibpy.Ros(host='192.168.8.104', port=9012)
ros.run()

robot_name = "omega"

# Define ROS topics for map, lidar, odometry, and velocity commands
occupancy_grid_topic = roslibpy.Topic(ros, f'/{robot_name}/coolmap', 'nav_msgs/OccupancyGrid')
lidar_topic = roslibpy.Topic(ros, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
odom_topic = roslibpy.Topic(ros, f'/{robot_name}/odom', 'nav_msgs/Odometry')
cmd_vel_topic = roslibpy.Topic(ros, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')

# Map parameters
width, height = 200, 200
resolution = 0.1
origin_x = width // 2
origin_y = height // 2

# Robot and map state variables
latest_lidar = {}
latest_yaw = 0.0
latest_position = {'x': 0.0, 'y': 0.0}
occupancy_data = []
scanning_enabled = False  # Controlled with Enter key

# Initialize the occupancy grid to all unknown values (-1)
def create_map():
    global occupancy_data
    occupancy_data = [-1] * (width * height)

# Update the occupancy grid using the latest lidar data and pose
def update_map():
    global occupancy_data
    if not latest_lidar:
        return

    # Unpack lidar scan and robot pose
    angle_min = latest_lidar['angle_min']
    angle_increment = latest_lidar['angle_increment']
    ranges = latest_lidar['ranges']
    pose = latest_lidar['pose']

    # Convert robot world position to grid coordinates
    robot_map_x = int(origin_x + pose['x'] / resolution)
    robot_map_y = int(origin_y + pose['y'] / resolution)
    yaw = pose['yaw']

    for i, distance in enumerate(ranges):
        if math.isinf(distance) or math.isnan(distance):
            continue

        # Compute endpoint of the lidar beam in grid coordinates
        angle = angle_min + i * angle_increment
        world_angle = angle + yaw
        dx = distance * math.cos(world_angle)
        dy = distance * math.sin(world_angle)
        cell_x = int(robot_map_x + dx / resolution)
        cell_y = int(robot_map_y + dy / resolution)

        # Mark occupied cells with increasing intensity
        if 0 <= cell_x < width and 0 <= cell_y < height:
            index = cell_y * width + cell_x
            if occupancy_data[index] < 100:
                if occupancy_data[index] == -1:
                    occupancy_data[index] = 20
                else:
                    occupancy_data[index] = min(100, occupancy_data[index] + 10)

        # Mark free space along the beam
        steps = int(distance / resolution)
        for step in range(steps):
            fx = int(robot_map_x + (step * resolution) * math.cos(world_angle) / resolution)
            fy = int(robot_map_y + (step * resolution) * math.sin(world_angle) / resolution)
            if 0 <= fx < width and 0 <= fy < height:
                free_index = fy * width + fx
                if occupancy_data[free_index] == -1:
                    occupancy_data[free_index] = 0

# Create a message for the occupancy grid to publish to ROS
def make_grid_message():
    return {
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
        'data': occupancy_data
    }

# Callback to process incoming lidar data and attach current pose
def lidar_callback(message):
    global latest_lidar, latest_yaw, latest_position

    # Include robot pose with lidar scan
    lidar_pose = {
        'x': latest_position['x'],
        'y': latest_position['y'],
        'yaw': latest_yaw
    }

    latest_lidar = {
        'ranges': message['ranges'],
        'angle_min': message['angle_min'],
        'angle_increment': message['angle_increment'],
        'pose': lidar_pose
    }

    if scanning_enabled:
        print(f"LiDAR: {len(message['ranges'])} points")

# Callback to extract yaw and position from odometry
def odom_callback(msg):
    global latest_yaw, latest_position
    pose = msg['pose']['pose']
    position = pose['position']
    orientation = pose['orientation']

    latest_position['x'] = position['x']
    latest_position['y'] = position['y']

    # Convert quaternion to yaw
    x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    latest_yaw = math.atan2(siny_cosp, cosy_cosp)

# Send velocity command to the robot
def send_cmd_vel(linear_x=0.0, angular_z=0.0):
    cmd_msg = {
        'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
    }
    cmd_vel_topic.publish(roslibpy.Message(cmd_msg))
    print(f"Sent cmd_vel: linear={linear_x}, angular={angular_z}")

# Listen for keyboard input to control the robot and toggle scanning
def key_listener():
    def on_press(key):
        global scanning_enabled
        try:
            if key.char == 'w':
                send_cmd_vel(0.1, 0.0)
            elif key.char == 'a':
                send_cmd_vel(0.0, 0.1)
            elif key.char == 'd':
                send_cmd_vel(0.0, 0.1)
            elif key.char == 'r':
                create_map()
                print("Occupancy grid reset.")
        except AttributeError:
            if key == keyboard.Key.enter:
                scanning_enabled = not scanning_enabled
                print(f"[ENTER] Scanning {'ENABLED' if scanning_enabled else 'DISABLED'}")

    def on_release(key):
        send_cmd_vel(0.0, 0.0)
        if key == keyboard.Key.esc:
            return False

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

# Subscribe to lidar and odometry topics
lidar_topic.subscribe(lidar_callback)
odom_topic.subscribe(odom_callback)

# Start map and keyboard input
create_map()
key_listener()

# Main loop: updates and publishes map if scanning is enabled
try:
    while ros.is_connected:
        if scanning_enabled:
            update_map()
        occupancy_grid_topic.publish(roslibpy.Message(make_grid_message()))
        print("Published occupancy grid.")
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down...")

# Cleanup
occupancy_grid_topic.unadvertise()
ros.terminate()
