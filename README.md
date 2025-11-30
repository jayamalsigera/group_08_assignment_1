# **Intelligent Robotics – Assignment 1**

This ROS 2 Jazzy project implements the pipeline for Assignment 1 of the Intelligent Robotics Course 2025 in the Control Systems Engineering Master's Program at the  University of Padova, Italy.

---

## Build

```bash
cd ${HOME}/${workspace}
colcon build --symlink-install --packages-select group_8_assignment_1
source ${HOME}/${workspace}/install/setup.bash
```

---
## Run

# Terminal 1

This launches the Gazebo Simulation and RViz Visualization and all the nodes and start movin the robot towards the goal.
```bash
cd ${HOME}/${workspace}
source install/setup.bash
ros2 launch group_8_assignment_1 pipeline.launch.py
```

# Terminal 1

This is used to get the data from the Table Detection Node.
```bash
cd ${HOME}/${workspace}
source install/setup.bash
ros2 run group_8_assignment_1 table_detection_node
```


Example Output:

```bash
[TURTLEBOT] Generated N=7  Fixed S=15
[TURTLEBOT] Response received: enough = TRUE
```

---