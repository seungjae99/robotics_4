# Term Project

CNU AVSE mobile robotics term project

team 4

- **Task 2: Parking**
- **Task 3: Waypoint Navigation**

---

## Installation

### 1. Clone this repository
```
mkdir -p termproject_ws/src
cd ~/termproject_ws/src
git clone https://github.com/seungjae99/robotics_4.git termproject
```

### 2. Build the workspace
```
cd ~/termproject_ws
colcon build --symlink-install
```

### 3. Source the workspace
```
source install/setup.bash
```

---

## How to Use

### Task 2: Parking
```
ros2 launch termproject task2.launch.py
```

- parking node.
- You can modify the goal pose (x, y, theta) by changing the `goal` parameter in the launch file.


### Task 3: Waypoint Navigation
```
ros2 launch termproject task3.launch.py
```

- waypoint navigation node.
- You can change the waypoint position (x, y) by editing the `goal` parameter in the launch file.

---

## TODO

- ~~Task3: Add collision avoidance~~

---

## Notes

- Tested on ROS 2 Humble
