---
id: 2-1-gazebo-intro
title: "2.1: Introduction to Gazebo"
sidebar_label: "Gazebo Introduction"
sidebar_position: 1
description: "An introduction to the Gazebo simulation environment, a powerful tool for robotics development and testing."
keywords: ['gazebo', 'ros2', 'simulation', 'robotics', 'simulator']
difficulty: beginner
estimated_time: "45-60 minutes"
learning_outcomes:
  - "Understand the benefits of using simulation in robotics."
  - "Describe the architecture of Gazebo and its components."
  - "Create a simple Gazebo world file."
  - "Spawn a robot model into a Gazebo simulation."
prerequisites:
  - "module-1-ros2/1-5-urdf"
hardware_required: false
---

## Introduction to Gazebo

Gazebo is a powerful open-source 3D robotics simulator. It allows you to simulate robots in complex and realistic indoor and outdoor environments. For a roboticist, a simulator is like a playground, a laboratory, and a test track all rolled into one. It's a place to safely and efficiently develop, test, and refine your robot's software before deploying it on real hardware.

### Why Use Simulation?

- **Safety:** Testing unproven code on a physical robot can be dangerous to the robot, its environment, and people. Simulation provides a safe space to fail and iterate.
- **Cost-Effectiveness:** Physical robots are expensive. A simulator allows you to work with a virtual fleet of robots for the cost of a single computer.
- **Speed:** You can run simulations much faster than real-time, accelerating development and testing cycles. You can also parallelize tests in the cloud.
- **Accessibility:** Not everyone has access to expensive robotic hardware. Simulators democratize robotics development.

### Gazebo Architecture

Gazebo has a client/server architecture.

- **Gazebo Server (`gzserver`):** This is the core of the simulator. It runs the physics simulation, generates sensor data, and applies forces. It can run headless (without a graphical interface).
- **Gazebo Client (`gzclient`):** This is the graphical interface. It visualizes the simulation and provides tools to interact with the world.

```mermaid
graph TD
    A[Gazebo Server (gzserver)]
    B[Gazebo Client (gzclient)]
    A -- Visualization Data --> B
    B -- User Commands --> A
    subgraph Simulation Core
        A
    end
    subgraph User Interface
        B
    end
```
**Figure 2.1.1**: Gazebo's client/server architecture.

The server and client communicate over a network, meaning you can run the simulation on a powerful remote machine and visualize it on your local laptop.

### Creating a Simple World

A Gazebo world is defined in a `.world` file, which is an XML-based format called Simulation Description Format (SDF).

Here’s a simple world with a ground plane and a sun for lighting:

```xml title="simple.world"
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

You can find many pre-built models like `sun` and `ground_plane` in Gazebo's model database.

To run this world, save it as `simple.world` and run:

```bash title="Launching a Gazebo world"
gazebo simple.world
```

### Spawning a Robot

To add a robot to the simulation, you can include its model in the world file or spawn it dynamically. Spawning is more common when integrating with ROS 2.

First, ensure you have a robot description in URDF format. We created one in [Module 1, Chapter 5: URDF Robot Description](../module-1-ros2/1-5-urdf.md).

The `ros_gz_sim` package provides a bridge between ROS 2 and Gazebo. You can use it to spawn a URDF model.

```bash title="Spawning a URDF robot"
ros2 run ros_gz_sim create -file your_robot.urdf -name my_robot
```
This command will spawn the robot defined in `your_robot.urdf` into the running Gazebo simulation with the name `my_robot`.

In the next chapters, we will explore how to build more complex worlds and integrate sensors to make our simulation more realistic and useful.
