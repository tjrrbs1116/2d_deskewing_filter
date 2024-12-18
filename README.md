# 2dlidar_deskewing

<img src="https://github.com/user-attachments/assets/a0e436d5-22eb-416d-8656-ea0618ed8171">



- **🔴 Red points**: Raw data
- **⚪ White points**: Filtered data

## Parameters

- **lidar_topic**: Name of the subscribed topic for the 2D lidar.
- **odom_topic**: Name of the odom topic.
- **max_velocity**: The maximum velocity based on the robot's kinematics (values exceeding this velocity will trigger exception handling).
- **poses_size**: The number of accumulated odom poses (recommended to declare a larger value if the odom topic's frequency is high).
- **odom_hz**: Frequency of receiving the odom topic.
- **lidar_hz**: Frequency of receiving the lidar topic.
- **sync_tolerance**: Tolerance for time synchronization between lidar and odom.
- **scan_duration**: (Used only when `time_increment` is not available within the lidar topic) Time required for one full rotation of the lidar.
- **scan_point**: (Used only when `time_increment` is not available within the lidar topic) Number of points in one full rotation of the lidar