import time
import threading
import roslibpy
import math

class RobotController:
    def __init__(self, width=100, height=100, resolution=0.05):
        # ROS connection setup
        self.robot_name = 'omega'
        self.ros = roslibpy.Ros(host='192.168.8.104', port=9012)
        self.ros.run()

        print(f"[INFO] Connected to ROS: {self.ros.is_connected}")

        # Map params (width, height, resolution)
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -width // 2 * resolution  # Grid origin in meters
        self.origin_y = -height // 2 * resolution
        self.grid = [-1] * (width * height)  # Initialize grid as unknown (-1)

        # Robot position and orientation
        self.position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.lidar_ranges = []

        # Subscriptions to LIDAR and Odometry
        self.topic_lidar = roslibpy.Topic(self.ros, f'/{self.robot_name}/scan', 'sensor_msgs/LaserScan')
        self.topic_odom = roslibpy.Topic(self.ros, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')

        # Map publisher topic
        self.map_topic = roslibpy.Topic(self.ros, '/echo/map', 'nav_msgs/OccupancyGrid')

        # Subscribe to LIDAR and Odometry
        self.topic_lidar.subscribe(self.lidar_callback)
        self.topic_odom.subscribe(self.callback_odom)

        # Start the map publishing loop
        threading.Thread(target=self.publish_loop, daemon=True).start()

    def callback_odom(self, msg):
        """Odometry callback to update robot's position and orientation."""
        self.position['x'] = msg['pose']['pose']['position']['x']
        self.position['y'] = msg['pose']['pose']['position']['y']

        o = msg['pose']['pose']['orientation']
        x, y, z, w = o['x'], o['y'], o['z'], o['w']
        self.position['yaw'] = math.atan2(2*(x*y + z*w), 1 - 2*(y**2 + z**2))

    def lidar_callback(self, msg):
        """LIDAR callback to receive ranges."""
        self.lidar_ranges = msg['ranges']
        self.angle_min = msg['angle_min']
        self.angle_increment = msg['angle_increment']

    def world_to_grid(self, x, y):
        """Convert world coordinates (x, y) to grid cell indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to return grid cells between two points."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def update_grid(self):
        """Update the occupancy grid with LIDAR data."""
        if not self.lidar_ranges:
            return

        x_r, y_r = self.position['x'], self.position['y']
        yaw = self.position['yaw']

        for i, distance in enumerate(self.lidar_ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            angle = self.angle_min + i * self.angle_increment
            angle_world = yaw + angle

            # LIDAR hit point in world coordinates
            x_l = distance * math.cos(angle_world)
            y_l = distance * math.sin(angle_world)

            x_hit = x_r + x_l
            y_hit = y_r + y_l

            # Convert robot and hit point to grid coordinates
            gx_r, gy_r = self.world_to_grid(x_r, y_r)
            gx_h, gy_h = self.world_to_grid(x_hit, y_hit)

            # Raytrace from robot to hit point and mark free space
            for x_cell, y_cell in self.bresenham(gx_r, gy_r, gx_h, gy_h):
                idx = y_cell * self.width + x_cell
                if 0 <= idx < len(self.grid):
                    self.grid[idx] = 0  # Free space

            # Mark the hit point as occupied
            idx_hit = gy_h * self.width + gx_h
            if 0 <= idx_hit < len(self.grid):
                self.grid[idx_hit] = 1  # Occupied

    def make_grid_msg(self):
        """Create the OccupancyGrid message to publish."""
        return {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': int(time.time()), 'nsecs': 0}
            },
            'info': {
                'map_load_time': {'secs': int(time.time()), 'nsecs': 0},
                'resolution': self.resolution,
                'width': self.width,
                'height': self.height,
                'origin': {
                    'position': {'x': self.origin_x, 'y': self.origin_y, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            'data': self.grid
        }

    def publish_loop(self):
        """Continuously publish the updated occupancy grid."""
        while self.ros.is_connected:
            self.update_grid()
            msg = self.make_grid_msg()
            self.map_topic.publish(roslibpy.Message(msg))
            print("[INFO] Published occupancy grid")
            time.sleep(1)  # Publish every second


# Run the controller
if __name__ == "__main__":
    try:
        robot = RobotController(width=150, height=150, resolution=0.05)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down.")
