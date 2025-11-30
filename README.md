# **Intelligent Robotics – Assignment 1**

This ROS 2 (Jazzy) project implements the complete pipeline for Assignment 1 of the Intelligent Robotics course (2025), part of the Control Systems Engineering Master’s Program at the University of Padova, Italy.

---

## **Build**

Before building, ensure your ROS 2 workspace is sourced correctly and that no stale build or install files are present. If this is the first build it is recommended to remove previous build artifacts to avoid CMake or ament caching issues:
```bash
rm -rf build/ install/ log/
```
Then build the whole workspace, including the `ir_2526` package, using:
```bash
cd ${HOME}/${workspace}
colcon build --symlink-install
source ${HOME}/${workspace}/install/setup.bash
```

---
## **Run**

### **Terminal 1**

This terminal launches Gazebo, RViz, and all ROS 2 nodes to start moving the robot toward the goal.

[OPTIONAL] Before launching the pipeline, terminate any leftover Gazebo/RViz processes from previous simulations:
```bash
pkill -9 -f "ruby|ign|gz|gzclient|gzserver"
```
Then launch the pipeline:
```bash
cd ${HOME}/${workspace}
source install/setup.bash
ros2 launch group_8_assignment_1 pipeline.launch.py
```

### **Terminal 2**

This terminal is used to run and view the output from the Table Detection Node.
```bash
cd ${HOME}/${workspace}
source install/setup.bash
ros2 run group_8_assignment_1 table_detection_node
```

Example Output:

```bash
=== TABLE DETECTION ===
Tables found: 3
  [1] x=13.26, y=-1.64
  [2] x=13.78, y=0.62
  [3] x=9.63, y=-4.94
------------------------
```
---

### **Terminate the System**

To stop the entire system, switch to **Terminal 1** and press:

```
Ctrl + C
```

This automatically triggers the cleanup routine (the `pkill` commands) to ensure all Gazebo/IGN/RViz processes are terminated.

---
