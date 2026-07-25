# ROS2 Mobile Robot Path Tracking and Multi-Sensor State Estimation System

基于 ROS2 Jazzy、Gazebo Harmonic 和 C++ 的移动机器人路径跟踪与多传感器状态估计仿真项目。

## Development Environment

- Host: Apple M4 Mac
- Virtualization: Parallels Desktop 26
- Guest OS: Ubuntu 24.04 ARM64
- ROS: ROS2 Jazzy
- Simulator: Gazebo Harmonic
- Visualization: RViz2
- Main language: C++
- Data analysis: Python
- Build system: colcon
- Dependency management: rosdep

Gazebo Harmonic GUI is launched with the Ogre renderer, shadows disabled in the project world, and Mesa OpenGL 3.3 compatibility mode to avoid rendering instability with the Parallels `virgl` virtual GPU. Use `scripts/run_gazebo_gui.sh` to start the verified configuration. Headless simulation remains available for automated tests.

## Progress

- [x] Stage 0: Ubuntu ARM64 virtual machine setup
- [x] Stage 1: Linux, Git and SSH fundamentals
- [x] Stage 2: ROS2 Jazzy and Gazebo Harmonic installation
- [ ] Stage 3: ROS2 core concepts
- [ ] Stage 4: Differential-drive robot simulation
- [ ] Stage 5: Path tracking controller
- [ ] Stage 6: Multi-sensor state estimation
- [ ] Stage 7: Safety, testing and engineering
- [ ] Stage 8: Experiments and project presentation

## Next Step

Learn ROS2 nodes, topics, messages, services, parameters, launch files and workspace structure.
