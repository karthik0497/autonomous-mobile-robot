# Autonomous Mobile Robot (AMR)

An end-to-end Autonomous Mobile Robot project developed using ROS 2 and Gazebo, progressing from fundamental robot modeling to autonomous navigation and real-world hardware integration.

The project follows a **simulation-first approach**: each robotics concept is learned, implemented, tested, and validated in simulation before moving to physical hardware.

## Project Objectives

- Understand mobile robot fundamentals and differential-drive kinematics.
- Build robot models using URDF and Xacro.
- Simulate robot physics and environments using Gazebo.
- Visualize robot models, coordinate frames, and sensor data in RViz2.
- Integrate LiDAR, camera, and IMU sensors.
- Implement robot control, odometry, and sensor fusion.
- Perform mapping and localization using SLAM.
- Achieve autonomous navigation using Nav2.
- Explore computer vision and visual target following.
- Integrate the simulation with physical hardware after completing the simulation stages.

## Technology Stack

- **Operating System:** Ubuntu Linux
- **Robotics Middleware:** ROS 2
- **Robot Modeling:** URDF, Xacro
- **Simulation:** Gazebo
- **Visualization:** RViz2
- **Programming:** Python, XML, YAML, C++ (where required)
- **Mapping:** SLAM Toolbox
- **Navigation:** Nav2
- **Computer Vision:** OpenCV
- **Sensor Fusion:** robot_localization / EKF
- **Future Hardware:** ESP32, motor drivers, encoders, LiDAR, IMU

## Development Roadmap

- [ ] **Phase 1:** Simulation Environment and Robot Fundamentals
- [ ] **Phase 2:** Simulated Sensor Integration
- [ ] **Phase 3:** Simulated Robot Control and Odometry
- [ ] **Phase 4:** Sensor Fusion and State Estimation
- [ ] **Phase 5:** SLAM and Mapping
- [ ] **Phase 6:** Autonomous Navigation with Nav2
- [ ] **Phase 7:** Computer Vision and Target Following
- [ ] **Phase 8:** Hardware Electronics and Assembly
- [ ] **Phase 9:** Motor Control and Encoder Feedback
- [ ] **Phase 10:** ROS 2 and Hardware Communication
- [ ] **Phase 11:** Real Sensor Integration
- [ ] **Phase 12:** Sim-to-Real Deployment

Each phase will include practical exercises, validation steps, and a mini-project.

## Phase 1: Simulation Environment and Robot Fundamentals

**Goal:** Build and visualize a differential-drive AMR in RViz2 and Gazebo.

Planned topics:

- AMR fundamentals and coordinate systems
- Differential-drive kinematics
- ROS 2 nodes, topics, packages, and workspaces
- URDF and Xacro
- Robot links, joints, collision, visual, and inertial properties
- TF2 coordinate frames
- Gazebo worlds, physics, and robot spawning
- RViz2 visualization

**Phase 1 Mini-Project:** Build a differential-drive robot with two drive wheels and a caster wheel, visualize its TF tree in RViz2, and spawn it in a custom Gazebo environment.

## Repository Structure

```text
autonomous-mobile-robot/
├── README.md
├── phase-01-simulation-foundation/
├── phase-02-simulated-sensors/
├── phase-03-robot-control/
├── phase-04-sensor-fusion/
├── phase-05-slam/
├── phase-06-nav2/
├── phase-07-computer-vision/
├── phase-08-hardware/
├── phase-09-motor-control/
├── phase-10-ros2-hardware-bridge/
├── phase-11-real-sensors/
└── phase-12-sim-to-real/
```

Folders will be populated as each phase is started and completed.

## Learning Approach

1. Learn the theory behind each concept.
2. Implement the concept in ROS 2 and Gazebo.
3. Test and debug the implementation.
4. Complete the phase mini-project.
5. Document results, commands, and lessons learned.
6. Move to the next phase only after completing the current one.

## Project Status

**Current stage:** Phase 1 — Planning and preparation.

This repository is a work in progress. Features will be documented as they are implemented and tested.

## Author

**Karthik B.**

Software Engineer | Backend & Robotics Software

- GitHub: [karthik0497](https://github.com/karthik0497)
- Portfolio: [karthiktechie.vercel.app](https://karthiktechie.vercel.app)
