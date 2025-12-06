---
id: glossary
title: "Glossary"
sidebar_label: "Glossary"
description: Comprehensive glossary of robotics, AI, and technical terms used throughout the textbook
keywords:
  - glossary
  - definitions
  - terminology
  - robotics terms
---

# Glossary

Comprehensive definitions of technical terms used throughout this textbook.

## A

### Action
A long-running ROS 2 task with goal, feedback, and result. Used for operations that take time and require progress updates (e.g., navigation to a goal location).

**Related**: [Service](#service), [Topic](#topic)
**Modules**: [1.3 Services and Actions](../module-1-ros2/1-3-services-actions.md)

### Actuator
A mechanical device that creates motion in a robot (motors, servos, hydraulics, pneumatics).

## C

### Computational Graph
The network of nodes and communication channels (topics, services, actions) that form a ROS 2 system.

**Related**: [Node](#node), [Topic](#topic)
**Modules**: [1.1 Introduction](../module-1-ros2/1-1-ros2-intro.md)

## D

### DDS (Data Distribution Service)
The middleware standard used by ROS 2 for inter-process communication.

### Digital Twin
A virtual representation of a physical robot or environment used for simulation and testing.

**Related**: [Gazebo](#gazebo), [Unity](#unity)
**Modules**: [Module 2](../module-2-simulation/index.md)

### Domain Randomization
Technique for varying simulation parameters during training to improve real-world transfer.

**Modules**: [Module 3: Isaac](../module-3-isaac/index.md)

## G

### Gazebo
Open-source 3D robot simulator with physics engine, sensor simulation, and ROS 2 integration.

**Related**: [Unity](#unity), [Isaac Sim](#isaac-sim)
**Modules**: [Module 2: Simulation](../module-2-simulation/index.md)

## I

### Isaac Gym
NVIDIA's physics simulation environment for reinforcement learning training.

**Related**: [Isaac Sim](#isaac-sim), [Reinforcement Learning](#reinforcement-learning)
**Modules**: [Module 3: Isaac](../module-3-isaac/index.md)

### Isaac Sim
NVIDIA's photorealistic robot simulator built on Omniverse platform.

**Related**: [Gazebo](#gazebo), [Digital Twin](#digital-twin)
**Modules**: [Module 3: Isaac](../module-3-isaac/index.md)

## L

### LiDAR (Light Detection and Ranging)
Sensor that measures distances using laser pulses, creating 3D point clouds.

**Modules**: [Module 2: Simulation](../module-2-simulation/index.md)

## N

### Node
An independent executable process in ROS 2 that performs a specific task.

**Related**: [Computational Graph](#computational-graph), [Topic](#topic)
**Modules**: [1.2 Nodes and Topics](../module-1-ros2/1-2-nodes-topics.md)

## O

### Omniverse
NVIDIA's platform for 3D simulation and collaboration, underlying Isaac Sim.

## P

### Parameter
A configuration value in ROS 2 that nodes can read and update dynamically.

**Modules**: [1.4 Parameters](../module-1-ros2/1-4-parameters.md)

### Publisher
A node that sends messages to a topic in ROS 2.

**Related**: [Subscriber](#subscriber), [Topic](#topic)
**Modules**: [1.2 Nodes and Topics](../module-1-ros2/1-2-nodes-topics.md)

## R

### Reinforcement Learning (RL)
Machine learning approach where agents learn through trial and error with rewards.

**Related**: [Isaac Gym](#isaac-gym)
**Modules**: [Module 3: Isaac](../module-3-isaac/index.md)

### ROS 2 (Robot Operating System 2)
Open-source middleware framework for robot software development.

**Modules**: [Module 1](../module-1-ros2/index.md)

## S

### Service
Synchronous request/response communication pattern in ROS 2.

**Related**: [Action](#action), [Topic](#topic)
**Modules**: [1.3 Services and Actions](../module-1-ros2/1-3-services-actions.md)

### Subscriber
A node that receives messages from a topic in ROS 2.

**Related**: [Publisher](#publisher), [Topic](#topic)
**Modules**: [1.2 Nodes and Topics](../module-1-ros2/1-2-nodes-topics.md)

## T

### Topic
Named communication channel in ROS 2 for asynchronous message passing.

**Related**: [Publisher](#publisher), [Subscriber](#subscriber), [Node](#node)
**Modules**: [1.2 Nodes and Topics](../module-1-ros2/1-2-nodes-topics.md)

## U

### Unity
Game engine used for robot simulation with ROS 2 integration via ROS-TCP connector.

**Related**: [Gazebo](#gazebo), [Digital Twin](#digital-twin)
**Modules**: [Module 2: Simulation](../module-2-simulation/index.md)

### URDF (Unified Robot Description Format)
XML format for describing robot kinematics, dynamics, and visual properties.

**Modules**: [1.5 URDF](../module-1-ros2/1-5-urdf.md)

## V

### Vision-Language-Action (VLA)
Integration of computer vision, natural language processing, and action planning in robotics.

**Modules**: [Module 4](../module-4-vla/index.md)

---

**Terms will be added as modules are completed**

[Return to Homepage](/)
