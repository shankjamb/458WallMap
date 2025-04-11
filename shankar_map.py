import time
import math
import threading
import numpy as np
import roslibpy

class RoomMapper:
    def __init__(self, width=200, height=200, resolution=0.05, sample_interval=0.05):
        # ROS connection setup
        self.robot_name = 'omega'
        self.ros = roslibpy.Ros(host='192.168.8.104', port=9012)
        self.ros.run()
        print(f"[INFO] Connected to ROS: {self.ros.is_connected}")

        # Occupancy grid parameters
        self.width = width      # number of cells in x-direction
        self.height = height    # number of cells in y-direction
        self.resolution = resolution  # meters per cell
        self.sample_interval = sample_interval  # the interval r at which to sample along each beam

        # Define world coordinate of bottom-left cell of the grid.
        self.origin_x = -width / 2 * resolution
        self.origin_y = -height / 2 * resolution

        # Persistent occupancy grid (actual map)
        # Initial value: -1 means "unknown"
        self.actual_grid = np.full((self.height, self.width), -1, dtype=int)

        # Global variables for sensor data
        self.latest_scan = None  # Latest LiDAR scan (LaserScan message)
        self.current_pose = None # Latest odometry as dict: {'x', 'y', 'yaw'}

        # Create publisher for occupancy grid map
        self.map_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/map', 'nav_msgs/OccupancyGrid')

        # Set up subscriptions for LiDAR and Odometry
        self.lidar_topic = roslibpy.Topic(self.ros,
                                          f'/{self.robot_name}/scan',
                                          'sensor_msgs/LaserScan')
        self.odom_topic = roslibpy.Topic(self.ros,
                                         f'/{self.robot_name}/odom',
                                         'nav_msgs/Odometry')

        self.lidar_topic.subscribe(self.lidar_callback)
        self.odom_topic.subscribe(self.odom_callback)

        # Start the publishing loop in a background thread
        threading.Thread(target=self.publish_loop, daemon=True).start()

    def odom_callback(self, msg):
        """Update current_pose based on Odometry message."""
        pos = msg['pose']['pose']['position']
        o = msg['pose']['pose']['orientation']
        # Convert quaternion to yaw (assuming planar motion)
        yaw = math.atan2(2*(o['x']*o['y'] + o['z']*o['w']),
                         1 - 2*(o['y']**2 + o['z']**2))
        self.current_pose = {'x': pos['x'], 'y': pos['y'], 'yaw': yaw}

    def lidar_callback(self, msg):
        """Store the latest LiDAR scan."""
        self.latest_scan = msg
        print(f"[INFO] Received LiDAR scan with {len(msg['ranges'])} ranges.")

    def world_to_grid(self, x, y):
        """Convert world coordinates (x, y) to grid cell indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham’s line algorithm: returns list of grid cells between two points."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
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

    def create_updated_grid(self):
        """
        Process the latest LiDAR scan and current robot pose to create an updated grid.
        For each beam:
          - Sample along the beam at multiples of sample_interval: r, 2r, 3r, ... until the measured range.
          - Mark the cells along the beam as visited free (update value: -1).
          - Mark the final cell (at the measured range) as an obstacle (update value: 1).
        """
        updated_grid = np.zeros_like(self.actual_grid, dtype=int)
        if self.latest_scan is None or self.current_pose is None:
            return updated_grid

        rx, ry, yaw = self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw']
        robot_cell = self.world_to_grid(rx, ry)

        angle_min = self.latest_scan['angle_min']
        angle_increment = self.latest_scan['angle_increment']
        ranges = self.latest_scan['ranges']

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            beam_angle = angle_min + i * angle_increment
            global_angle = yaw + beam_angle

            # Sample along the beam at intervals: r, 2r, 3r, etc.
            d = self.sample_interval
            while d < r:
                ix = rx + d * math.cos(global_angle)
                iy = ry + d * math.sin(global_angle)
                grid_x, grid_y = self.world_to_grid(ix, iy)
                if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                    updated_grid[grid_y, grid_x] = -1  # Free space (visited)
                d += self.sample_interval

            # Mark the final cell as obstacle
            ex = rx + r * math.cos(global_angle)
            ey = ry + r * math.sin(global_angle)
            grid_ex, grid_ey = self.world_to_grid(ex, ey)
            if 0 <= grid_ex < self.width and 0 <= grid_ey < self.height:
                updated_grid[grid_ey, grid_ex] = 1

        return updated_grid

    def accumulate_grid(self, updated):
        """
        Merge the updated grid into the persistent actual grid.
        Rule:
          - If a cell in the actual grid is unknown (-1) and the updated cell indicates free (-1),
            set the persistent cell to free (0).
          - If the updated cell indicates an obstacle (1), set the persistent cell to 1.
          - Otherwise, leave the persistent cell unchanged.
        """
        for y in range(self.height):
            for x in range(self.width):
                upd = updated[y, x]
                current = self.actual_grid[y, x]
                if upd == 1:
                    self.actual_grid[y, x] = 1  # Obstacle
                elif upd == -1 and current == -1:
                    self.actual_grid[y, x] = 0  # Now known to be free

    def make_grid_msg(self):
        """Create a nav_msgs/OccupancyGrid message from the persistent actual grid."""
        grid_msg = {
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
            'data': self.actual_grid.flatten().tolist()
        }
        return grid_msg

    def publish_loop(self):
        """Continuously update and publish the occupancy grid map."""
        while self.ros.is_connected:
            updated_grid = self.create_updated_grid()
            self.accumulate_grid(updated_grid)
            msg = roslibpy.Message(self.make_grid_msg())
            self.map_topic.publish(msg)
            print(f"[INFO] Published map at {time.time()}")
            time.sleep(1)

    def run(self):
        """Keep running so that the background thread can update and publish."""
        while self.ros.is_connected:
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        mapper = RoomMapper(width=200, height=200, resolution=0.05, sample_interval=0.05)
        mapper.run()
    except KeyboardInterrupt:
        print("Shutting down...")
        mapper.map_topic.unadvertise()
        mapper.ros.terminate()
