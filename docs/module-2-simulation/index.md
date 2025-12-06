---
id: module-2-simulation-index
title: "Module 2: Gazebo & Unity (Digital Twins)"
sidebar_label: "Module 2: Simulation"
description: Learn robot simulation with Gazebo and Unity, including physics engines, sensor simulation, and ROS 2 integration
keywords:
  - gazebo
  - unity
  - simulation
  - digital twin
  - physics engine
difficulty: intermediate
module_number: 2
estimated_duration: "2-3 weeks"
---

# Module 2: Gazebo & Unity (Digital Twins)

## Module Overview

Robot simulation enables testing and development without physical hardware. This module covers two industry-standard simulators: Gazebo (open-source robotics simulator with physics) and Unity (game engine with ROS 2 integration). You'll learn to create virtual environments, simulate sensors, and test robot behaviors in realistic scenarios before deploying to real hardware.

## Learning Outcomes

By completing this module, you will be able to:

1. **Configure** Gazebo simulation environments with physics engines and plugins
2. **Create** 3D worlds and robot models for Gazebo simulation
3. **Simulate** sensors (LiDAR, cameras, IMU) and process synthetic data
4. **Integrate** Unity with ROS 2 using ROS-TCP connector
5. **Build** custom Unity environments for robot testing and visualization

## Prerequisites

- Completed [Module 1: ROS 2](../module-1-ros2/index.md)
- Basic 3D geometry concepts (coordinate frames, transformations)
- Familiarity with XML/YAML configuration files
- Ubuntu 22.04 with ROS 2 Humble installed

:::tip New to Simulation?
This module assumes ROS 2 knowledge but no prior simulation experience. We'll build understanding from first principles.
:::

## Module Structure

This module contains 4 chapters and a comprehensive exercise set:

### Chapter 2.1: Gazebo Introduction
- Gazebo architecture and components
- Installing Gazebo Harmonic
- World files and SDF format
- Launching simulations with ROS 2

### Chapter 2.2: Unity for Robotics
- Unity installation and setup
- ROS-TCP connector integration
- Creating Unity scenes for robots
- Publishing/subscribing to ROS 2 topics from Unity

### Chapter 2.3: Sensor Simulation
- LiDAR, camera, and IMU simulation
- Sensor plugins and configuration
- Processing simulated sensor data
- Comparing Gazebo vs Unity sensor models

### Chapter 2.4: Advanced Simulation
- Custom Gazebo plugins
- Physics engine tuning
- Multi-robot simulation
- Sim-to-real transfer considerations

### Exercises
- 5 hands-on exercises (beginner to advanced)
- Solutions with explanations
- Validation criteria for self-assessment

:::note Coming Soon
Chapter content for Module 2 is under development. Complete Module 1 first!
:::

## Estimated Time

- **Reading**: 6-8 hours
- **Coding/Setup**: 10-12 hours
- **Exercises**: 5-7 hours
- **Total**: 21-27 hours over 2-3 weeks (7-10 hours/week)

## Software Requirements

- **Gazebo Version**: Gazebo Harmonic (recommended) or Gazebo Garden
- **Unity Version**: Unity 2022.3 LTS or later
- **ROS 2 Version**: Humble Hawksbill
- **GPU**: Recommended for Unity (Intel/NVIDIA/AMD with Vulkan support)
- **Disk Space**: ~15GB (Gazebo models + Unity editor)

Installation instructions: [Software Setup Guide](../resources/software-setup.md)

## Hardware Requirements

**Simulation Only** - No physical robot required. GPU recommended but not required.

**Recommended Specs**:
- CPU: 4+ cores
- RAM: 8GB minimum, 16GB recommended
- GPU: Dedicated GPU for Unity (integrated graphics may work with reduced quality)

## Learning Approach

### Recommended Study Path

1. **Read chapter content**: Understand simulation concepts before hands-on work
2. **Install software**: Follow setup guides carefully
3. **Execute examples**: Run all provided simulations
4. **Experiment**: Modify worlds, sensors, and physics parameters
5. **Complete exercises**: Practice with guided tasks
6. **Validate**: Check solutions after honest attempts

### Common Challenges

:::warning Watch Out For
- **Gazebo resource paths**: Models and worlds must be in GAZEBO_MODEL_PATH
- **Unity-ROS connection**: Ensure ROS-TCP endpoint is running before Unity play
- **Physics instability**: Tune solver parameters for stable simulations
- **GPU drivers**: Unity may require updated graphics drivers
:::

## Teaching Notes (For Instructors)

### Week-by-Week Breakdown

- **Week 1**: Chapters 2.1-2.2 (Gazebo introduction, Unity setup)
- **Week 2**: Chapter 2.3 (Sensor simulation, data processing)
- **Week 3**: Chapter 2.4 + Exercises (Advanced topics, practice)

### Common Student Misconceptions

1. **"Simulation is always accurate"** - Sim-to-real gap exists; tune physics for realism
2. **"Unity is just for games"** - Unity is increasingly used for robotics simulation
3. **"Faster simulation is always better"** - Real-time factor affects physics accuracy

### Assessment Suggestions

- **Quiz**: Gazebo architecture, Unity-ROS integration (Week 2)
- **Lab**: Create custom Gazebo world with sensors (Week 2)
- **Project**: Multi-robot Unity simulation with ROS 2 (Week 3)

## Next Steps

After completing Module 2, you'll be ready for:
- [Module 3: NVIDIA Isaac](../module-3-isaac/index.md) - GPU-accelerated simulation and RL training
- Physical robot testing (using skills from Modules 1-2)

## Additional Resources

- **Gazebo Documentation**: [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic/tutorials)
- **Unity Robotics Hub**: [GitHub Repository](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- **Glossary**: See [technical terms](../resources/glossary.md#gazebo) for definitions
- **Community**: Gazebo Answers, Unity Robotics Forum

---

**Ready to start?** Complete [Module 1: ROS 2](../module-1-ros2/index.md) first, then return here for simulation training!
