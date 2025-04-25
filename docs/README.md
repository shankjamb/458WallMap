# Introduction
This project aimed to use a ROSLIBPY robot in order to build a map of the 2nd Floor of Hopper Hall. 

# Inspriation
This project was inspired by the LIDAR capabilities of the RosLibpy system and how it could be used to create a map of the classroom. In this 



# Depedencies
In the terminal of your python code editor

‘Python

pip install roslibpy’


# RosLibPy Installation

# Pseudocode Map
‘Initialize grid as 2D array filled with -1  // -1 = unknown
Set grid resolution (meters per cell)

Loop:
    Get robot (x, y, yaw) from odometry
    Get LiDAR ranges and angle parameters

    For each LiDAR beam:
        If range is invalid, skip

        Compute beam angle in world frame (yaw + angle_offset)
        Calculate endpoint (x_end, y_end) using robot pose + range

        Use raycasting to find all cells between (x, y) and (x_end, y_end)

        For each cell along the ray (except endpoint):
            Decrease occupancy value (toward free), but don't go below 0

        For the endpoint cell:
            Increase occupancy value (toward occupied), up to max (e.g., 100)

    Publish or visualize the updated occupancy grid’



# How to Run
1) Set the desired height, width, and resolution of the map
2) Connect to the robot from your computer
3) Drive the robot to the desired starting point and start the LIDAR data colleciton
4) Move the robot to another spot and collect again
5) Repeat steps 3-4 until the area you wish to map has been covered

# Example Map
![Final](recording.gif)



# About the Creators
Thomas Bui
1. From Ridgecrest, CA
2. Weapons, Robotics, Control Engineering Major at USNA
3. 3rd Company
4. Navy Pilot Select


Shankar Jambunathan
1. From Aurora, IL
2. Weapons, Robotics, Control Engineering Major
3. 1st Company
4. Submarine Select

