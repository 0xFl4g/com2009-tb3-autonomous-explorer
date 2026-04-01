# COM2009 - TurtleBot3 Autonomous Explorer

A suite of autonomous navigation and perception tasks for the TurtleBot3 (Waffle), built with ROS 1 and Python. Developed as part of the COM2009 Robotics module at the University of Sheffield.

## Tasks

### Task 1 - Figure-Eight Motion
Open-loop velocity control to drive the robot in a figure-eight pattern — two 0.5m-radius circles with a straight-line transition.

### Task 2 - Obstacle Avoidance
Reactive obstacle avoidance using 360-degree LIDAR. The robot explores freely while dodging walls and obstacles using sector-based range thresholds.

### Task 3 - Wall Following (Maze Solver)
Right-hand wall-following algorithm that navigates through unknown maze environments. Uses PD-like bearing correction to maintain a consistent distance from the right wall, with dead-end detection and emergency turning.

### Task 4 - Colour Beacon Detection
The robot identifies its start zone colour via camera, then explores the environment searching for a matching coloured beacon. Combines HSV colour detection (6 colours) with LIDAR proximity checks for confirmation.

### Task 5 - SLAM Mapping + Beacon Photography
Two-node system running alongside `turtlebot3_slam`:
- **Wall follower** navigates and periodically saves the occupancy grid map
- **Camera node** detects and photographs the target beacon, saving the best snapshot

## Tech Stack

- **ROS 1** (Noetic) with `rospy`
- **Python 3** for all nodes
- **OpenCV** + `cv_bridge` for colour detection and image processing
- **SLAM** via `turtlebot3_slam` (GMapping)
- **Sensors**: 360-degree LIDAR (`/scan`), RGB camera, wheel odometry (`/odom`)

## Project Structure

```
.
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── launch/
│   └── task1.launch ... task5.launch
├── maps/                    # SLAM-generated occupancy grid maps
├── snaps/                   # Beacon photographs captured by Task 5
└── src/
    ├── tb3.py               # Shared TurtleBot3 utility classes
    ├── camera_saver.py      # Colour beacon detector + snapshot saver
    ├── task1_node.py         # Figure-eight motion
    ├── task2_node.py         # Obstacle avoidance
    ├── task3_node.py         # Wall following / maze solver
    ├── task4_node.py         # Colour beaconing
    └── task5_node.py         # SLAM mapping + wall following
```

## Setup

1. Navigate to your Catkin workspace `src` directory:

    ```bash
    cd ~/catkin_ws/src
    ```

2. Clone the repository:

    ```bash
    git clone https://github.com/0xFl4g/com2009-tb3-autonomous-explorer.git com2009_tb3_explorer
    ```

3. Build the package:

    ```bash
    catkin build com2009_tb3_explorer
    ```

4. Re-source your environment:

    ```bash
    source ~/.bashrc
    ```

## Usage

Launch any task with:

```bash
roslaunch com2009_tb3_explorer task1.launch
```

Task 5 requires a target colour and mode:

```bash
# Simulation
roslaunch com2009_tb3_explorer task5.launch target_colour:=red mode:=1

# Real robot
roslaunch com2009_tb3_explorer task5.launch target_colour:=blue mode:=0
```

## License

[MIT](LICENSE)
