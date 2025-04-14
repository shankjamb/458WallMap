import time
import roslibpy
import math
import threading
from pynput import keyboard

# Connect to rosbridge server using the specified host and port
ros = roslibpy.Ros(host='192.168.8.104', port=9012)
ros.run()

robot_name = "india"  # Define the name of the robot for topic identification

# Define topics for occupancy grid, LiDAR scan, odometry data, and command velocity
occupancy_grid_topic = roslibpy.Topic(ros, f'/{robot_name}/coolmap', 'nav_msgs/OccupancyGrid')
lidar_topic = roslibpy.Topic(ros, f'/{robot_name}/scan', 'sensor_msgs/LaserScan')
odom_topic = roslibpy.Topic(ros, f'/{robot_name}/odom', 'nav_msgs/Odometry')
cmd_vel_topic = roslibpy.Topic(ros, f'/{robot_name}/cmd_vel', 'geometry_msgs/Twist')

# Define grid parameters for the occupancy grid
width, height = 200, 200  # Grid size (200x200)
resolution = 0.1  # Each cell represents 0.1 meters (10 cm)
origin_x = width // 2  # The robot’s initial position in the center of the grid (x)
origin_y = height // 2  # The robot’s initial position in the center of the grid (y)

# Initialize state variables
latest_lidar = {}  # To store the most recent LiDAR data
latest_yaw = 0.0  # Robot’s yaw (orientation) angle
latest_position = {'x': 0.0, 'y': 0.0}  # Robot’s x, y position
occupancy_data = []  # Array to hold the occupancy grid data

def create_map():
    """
    Initialize the occupancy grid with 'unknown' values (-1).
    """
    global occupancy_data
    occupancy_data = [-1] * (width * height)  # All cells start as unknown (-1)

def update_map():
    """
    Update the occupancy grid based on the latest LiDAR data and robot position.
    Clears and rebuilds the grid each time it's called.
    """
    global occupancy_data
    if not latest_lidar:
        return  # Exit if there is no LiDAR data yet

    # Reset map to unknown (-1) for each update
    occupancy_data = [-1] * (width * height)

    # Extract LiDAR scan parameters
    angle_min = latest_lidar['angle_min']
    angle_increment = latest_lidar['angle_increment']
    ranges = latest_lidar['ranges']

    # Convert robot position to grid coordinates
    robot_map_x = int(origin_x + latest_position['x'] / resolution)
    robot_map_y = int(origin_y + latest_position['y'] / resolution)

    # Iterate over all LiDAR range measurements
    for i, distance in enumerate(ranges):
        if math.isinf(distance) or math.isnan(distance):
            continue  # Skip invalid (infinite or NaN) measurements

        # Calculate the angle of this LiDAR beam
        angle = angle_min + i * angle_increment
        world_angle = angle + latest_yaw  # Adjust for robot's orientation (yaw)

        # Convert polar coordinates (distance and angle) to Cartesian coordinates
        dx = distance * math.cos(world_angle)
        dy = distance * math.sin(world_angle)

        # Convert the Cartesian coordinates to grid cell indices
        cell_x = int(robot_map_x + dx / resolution)
        cell_y = int(robot_map_y + dy / resolution)

        # Mark the grid cell as occupied (obstacle) if it's within bounds
        if 0 <= cell_x < width and 0 <= cell_y < height:
            index = cell_y * width + cell_x
            occupancy_data[index] = 100  # 100 = obstacle

        # Mark free space along the ray from the robot to the obstacle
        steps = int(distance / resolution)  # Calculate how many steps to take
        for step in range(steps):
            fx = int(robot_map_x + (step * resolution) * math.cos(world_angle) / resolution)
            fy = int(robot_map_y + (step * resolution) * math.sin(world_angle) / resolution)

            # Mark each cell along the ray as free (0), if it's within bounds
            if 0 <= fx < width and 0 <= fy < height:
                free_index = fy * width + fx
                if occupancy_data[free_index] == -1:  # Only mark if it's still unknown
                    occupancy_data[free_index] = 0  # 0 = free

def make_grid_message():
    """
    Create a message for the occupancy grid to publish to ROS.
    """
    return {
        'header': {
            'frame_id': 'map',  # The reference frame for the map
            'stamp': {'secs': int(time.time()), 'nsecs': 0}  # Timestamp for the message
        },
        'info': {
            'map_load_time': {'secs': int(time.time()), 'nsecs': 0},  # Time the map was loaded
            'resolution': resolution,  # Grid resolution (meters per cell)
            'width': width,  # Grid width (number of cells in x-direction)
            'height': height,  # Grid height (number of cells in y-direction)
            'origin': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # The origin of the map in 3D space
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}  # Orientation of the map
            }
        },
        'data': occupancy_data  # The actual occupancy grid data (cells with obstacles or free space)
    }

# Callbacks for handling incoming ROS messages

def lidar_callback(message):
    """
    Callback function to update LiDAR data.
    """
    global latest_lidar
    latest_lidar = message  # Store the latest LiDAR scan data
    print(f"LiDAR: {len(message['ranges'])} points")  # Log the number of LiDAR points received

def odom_callback(msg):
    """
    Callback function to update robot position and orientation (yaw).
    """
    global latest_yaw, latest_position
    pose = msg['pose']['pose']
    position = pose['position']  # Extract position (x, y)
    orientation = pose['orientation']  # Extract orientation (quaternion)

    latest_position['x'] = position['x']  # Update x position
    latest_position['y'] = position['y']  # Update y position

    # Convert quaternion to yaw angle using atan2
    x = orientation['x']
    y = orientation['y']
    z = orientation['z']
    w = orientation['w']

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    latest_yaw = math.atan2(siny_cosp, cosy_cosp)  # Calculate the robot's yaw

# Function to send velocity commands to the robot

def send_cmd_vel(linear_x=0.0, angular_z=0.0):
    """
    Send velocity commands to the robot.
    """
    cmd_msg = {
        'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},  # Linear velocity in x-direction
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}  # Angular velocity (yaw)
    }
    cmd_vel_topic.publish(roslibpy.Message(cmd_msg))  # Publish the message to ROS
    print(f"Sent cmd_vel: linear={linear_x}, angular={angular_z}")  # Log the command

# Function to listen for keypresses to control the robot manually

def key_listener():
    """
    Listen for key presses to manually control the robot.
    """
    def on_press(key):
        """
        Handle key press events for robot movement.
        """
        try:
            if key.char == 'w':
                send_cmd_vel(0.5, 0.0)  # Move forward
            elif key.char == 'a':
                send_cmd_vel(0.0, 1.0)  # Turn left
            elif key.char == 'd':
                send_cmd_vel(0.0, -1.0)  # Turn right
            elif key.char == 'r':
                create_map()  # Reset the occupancy grid to unknown
                print("Occupancy grid reset to unknown (-1).")
        except AttributeError:
            pass

    def on_release(key):
        """
        Handle key release events to stop the robot.
        """
        send_cmd_vel(0.0, 0.0)  # Stop the robot
        if key == keyboard.Key.esc:
            return False  # Stop the listener if 'esc' is pressed

    # Start listening for key presses
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

# Subscribe to the LiDAR and odometry topics
lidar_topic.subscribe(lidar_callback)
odom_topic.subscribe(odom_callback)

# Start keyboard control
key_listener()

# Main loop
create_map()  # Initialize the map

try:
    while ros.is_connected:  # Run as long as ROS is connected
        update_map()  # Update the map with new LiDAR data
        occupancy_grid_topic.publish(roslibpy.Message(make_grid_message()))  # Publish the updated grid
        print("Published occupancy grid.")
        time.sleep(1)  # Wait for 1 second before updating again

except KeyboardInterrupt:
    print("Shutting down...")  # Handle Ctrl+C to exit the program

# Cleanup after the program ends
occupancy_grid_topic.unadvertise()  # Unadvertise the occupancy grid topic
ros.terminate()  # Terminate the ROS connection
