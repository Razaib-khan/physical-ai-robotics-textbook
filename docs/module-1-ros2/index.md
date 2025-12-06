---
id: module-1-ros2-index
title: "Module 1: ROS 2 (Robotic Nervous System)"
sidebar_label: "Module 1: ROS 2"
description: Learn the fundamentals of ROS 2, including nodes, topics, services, parameters, and robot description formats
keywords:
  - ros2
  - robot operating system
  - nodes
  - topics
  - services
difficulty: beginner
module_number: 1
estimated_duration: "2-3 weeks"
---

# Module 1: ROS 2 (Robotic Nervous System)

## Module Overview

ROS 2 (Robot Operating System 2) is the foundational software framework for modern robotics. This module introduces core concepts: nodes as independent processes, topics for asynchronous communication, services for request/response patterns, and URDF for robot description. You'll learn to build distributed robotic systems using industry-standard tools.

## Learning Outcomes

By completing this module, you will be able to:

1. **Explain** ROS 2 architecture, computational graph concepts, and design patterns
2. **Implement** publisher/subscriber nodes in Python for inter-process communication
3. **Create** service servers and clients for synchronous request/response operations
4. **Configure** robot systems using parameters and launch files
5. **Design** robot descriptions using URDF with links, joints, and visual properties

## Prerequisites

- Basic Python programming (functions, classes, modules)
- Linux command line familiarity (cd, ls, mkdir, file navigation)
- Understanding of processes and inter-process communication concepts
- Ubuntu 22.04 LTS installed (virtual machine acceptable)

:::tip First Time with ROS?
No prior ROS or ROS 2 experience required! We'll build understanding from first principles.
:::

## Module Structure

This module contains 5 chapters and a comprehensive exercise set:

### [Chapter 1.1: Introduction to ROS 2](./1-1-ros2-intro.md)
- ROS 2 overview and history
- Architecture and computational graph
- Installation and environment setup
- First commands and verification

### [Chapter 1.2: Nodes and Topics](./1-2-nodes-topics.md)
- Understanding nodes as processes
- Publisher/subscriber pattern
- Creating your first publisher and subscriber
- Topic introspection and debugging

### [Chapter 1.3: Services and Actions](./1-3-services-actions.md)
- Synchronous services vs asynchronous actions
- Service server and client implementation
- Action goals, feedback, and results
- When to use services vs topics

### [Chapter 1.4: Parameters and Launch Files](./1-4-parameters.md)
- Parameter declaration and access
- Parameter callbacks for dynamic updates
- Launch file structure and syntax
- Multi-node system orchestration

### [Chapter 1.5: URDF Robot Description](./1-5-urdf.md)
- URDF structure and syntax
- Links, joints, and kinematic chains
- Visual and collision properties
- Robot visualization and validation

### [Exercises](./exercises/module-1-exercises.md)
- 5 hands-on exercises (beginner to advanced)
- Solutions with explanations
- Validation criteria for self-assessment

## Estimated Time

- **Reading**: 6-8 hours
- **Coding**: 10-12 hours
- **Exercises**: 4-6 hours
- **Total**: 20-26 hours over 2-3 weeks (7-10 hours/week)

## Software Requirements

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2 Version**: Humble Hawksbill (LTS)
- **Python**: 3.10 or later
- **Disk Space**: ~5GB for ROS 2 and dependencies

Installation instructions: [Software Setup Guide](../resources/software-setup.md)

## Hardware Requirements

**Simulation Only** - No physical robot required for this module. All examples run in software.

For learners interested in physical robots, see [Hardware Specifications](../resources/hardware-specs.md) after Module 2.

## Learning Approach

### Recommended Study Path

1. **Read chapter content**: Understand concepts before coding
2. **Execute code examples**: Copy and run all provided code
3. **Experiment**: Modify examples to explore behavior
4. **Complete exercises**: Practice with guided tasks
5. **Validate**: Check solutions after honest attempts

### Common Challenges

:::warning Watch Out For
- **Environment sourcing**: Must run `source /opt/ros/humble/setup.bash` in each terminal
- **Node name conflicts**: Each node must have a unique name
- **Topic types**: Publisher and subscriber must use matching message types
- **Dependencies**: Install required packages before running examples
:::

## Teaching Notes (For Instructors)

### Week-by-Week Breakdown

- **Week 1**: Chapters 1.1-1.2 (Introduction, Nodes, Topics)
- **Week 2**: Chapters 1.3-1.4 (Services, Actions, Parameters, Launch)
- **Week 3**: Chapter 1.5 + Exercises (URDF, Practice, Assessment)

### Common Student Misconceptions

1. **"ROS 2 is an operating system"** - It's a middleware framework, not an OS
2. **"Topics are synchronous"** - Topics use asynchronous pub/sub, services are synchronous
3. **"One node per robot"** - Modern systems use many nodes for modularity

### Assessment Suggestions

- **Quiz**: ROS 2 concepts, node communication patterns (Week 2)
- **Lab**: Implement multi-node system with topics and services (Week 3)
- **Project**: Custom robot description with URDF (Week 3)

## Next Steps

After completing Module 1, you'll be ready for:
- [Module 2: Gazebo & Unity](../module-2-simulation/index.md) - Simulate robots in virtual environments
- Physical robot integration (with real hardware)

## Additional Resources

- **Official Documentation**: [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- **Glossary**: See [technical terms](../resources/glossary.md) for definitions
- **Community**: ROS Discourse, ROS Answers for questions

---

**Ready to start?** Begin with [Chapter 1.1: Introduction to ROS 2](./1-1-ros2-intro.md) 🚀
