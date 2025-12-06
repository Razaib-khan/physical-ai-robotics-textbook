---
id: module-1-exercises
title: "Module 1 Exercises"
sidebar_label: "Exercises"
sidebar_position: 6
description: Hands-on exercises for ROS 2 fundamentals with solutions and validation criteria
keywords:
  - ros2
  - exercises
  - practice
  - hands-on
difficulty: beginner
estimated_time: "4-6 hours"
---

# Module 1 Exercises

Practice ROS 2 fundamentals with these hands-on exercises. Complete them in order for progressive skill building.

## Exercise 1: Publisher and Subscriber

**Objective**: Create a simple publisher/subscriber system for robot sensor data.

### Requirements

1. Create a publisher node that publishes random temperature readings (18.0-30.0°C) to `/sensor/temperature` topic every 1 second
2. Create a subscriber node that receives temperature data and prints warnings when temperature > 25.0°C
3. Use `std_msgs/Float32` message type

### Starter Code

[Code templates to be added]

### Validation Criteria

- [ ] Publisher publishes at 1 Hz rate
- [ ] Subscriber receives all messages
- [ ] Warning printed when threshold exceeded
- [ ] Both nodes have unique names

<details>
<summary>Solution</summary>

[Solution code to be added]

</details>

---

## Exercise 2: Service for Robot Control

**Objective**: Implement a service for querying and resetting robot position.

### Requirements

1. Create a custom service definition `GetPosition.srv` with:
   - Request: empty
   - Response: `float64 x`, `float64 y`, `float64 theta`
2. Create a service server that maintains robot position state and responds to queries
3. Create a service client that calls the service and prints the position

### Starter Code

[Code templates to be added]

### Validation Criteria

- [ ] Service definition compiles correctly
- [ ] Server responds with valid position data
- [ ] Client successfully calls service
- [ ] Position data is maintained across calls

<details>
<summary>Solution</summary>

[Solution code to be added]

</details>

---

## Exercise 3: Action for Navigation Goal

**Objective**: Create an action server for simulated robot navigation with progress feedback.

### Requirements

1. Use the `action_tutorials_interfaces/Fibonacci` action (or create custom navigation action)
2. Create an action server that:
   - Accepts goal position (x, y)
   - Sends feedback every 0.5 seconds with current position
   - Returns result when goal is reached
3. Create an action client that sends a goal and prints feedback

### Starter Code

[Code templates to be added]

### Validation Criteria

- [ ] Action server accepts goals
- [ ] Feedback published periodically
- [ ] Result sent when goal complete
- [ ] Client handles goal status correctly

<details>
<summary>Solution</summary>

[Solution code to be added]

</details>

---

## Exercise 4: Parameter Configuration System

**Objective**: Build a configurable node using parameters and launch files.

### Requirements

1. Create a node with these parameters:
   - `update_rate` (double, default: 10.0 Hz)
   - `max_speed` (double, default: 1.0 m/s)
   - `robot_name` (string, default: "robot_1")
2. Implement parameter callbacks to validate changes
3. Create a YAML parameter file
4. Create a launch file that loads parameters and starts the node

### Starter Code

[Code templates to be added]

### Validation Criteria

- [ ] Parameters declared with defaults
- [ ] Parameter callbacks validate ranges
- [ ] YAML file loads correctly
- [ ] Launch file configures node properly

<details>
<summary>Solution</summary>

[Solution code and files to be added]

</details>

---

## Exercise 5: URDF Robot Description

**Objective**: Create a complete URDF description for a simple mobile robot.

### Requirements

1. Design a differential drive robot with:
   - Base link (rectangular body)
   - Two wheel links (cylinders)
   - One caster link (sphere)
   - Camera link (small box)
2. Define appropriate joints (continuous for wheels, fixed for caster and camera)
3. Add visual and collision properties
4. Visualize in RViz with joint_state_publisher

### Starter Code

[URDF template to be added]

### Validation Criteria

- [ ] URDF validates with `check_urdf`
- [ ] All links connect to base_link
- [ ] Joints have correct types
- [ ] Robot visualizes correctly in RViz
- [ ] Joint state publisher controls wheels

<details>
<summary>Solution</summary>

[Complete URDF and launch files to be added]

</details>

---

## Challenge Exercise (Optional)

**Objective**: Integrate all Module 1 concepts into a multi-node robot system.

### Requirements

1. Create a system with:
   - Sensor publisher (temperature, distance)
   - Processing node (subscribes to sensors, publishes processed data)
   - Control service (start/stop data collection)
   - Action server (simulated movement to waypoints)
   - Parameter-based configuration
2. Create launch file that starts all nodes
3. Create URDF for the robot

### Validation Criteria

- [ ] All nodes communicate correctly
- [ ] Services and actions functional
- [ ] Parameters configure system
- [ ] Launch file starts complete system
- [ ] URDF matches system description

<details>
<summary>Solution Approach</summary>

[Architecture diagram and implementation notes to be added]

</details>

---

## Testing Your Solutions

### Prerequisites

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create workspace (if not already created)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Build and Run

```bash
# Build workspace
cd ~/ros2_ws
colcon build

# Source workspace
source install/setup.bash

# Run examples (commands to be added per exercise)
```

### Debugging Tips

:::tip Common Issues
1. **"Package not found"**: Did you source the workspace? `source install/setup.bash`
2. **"Topic not publishing"**: Check topic name with `ros2 topic list`
3. **"Node already running"**: Use unique node names or namespaces
4. **"URDF parsing error"**: Validate with `check_urdf robot.urdf`
:::

---

**Next Steps**: Continue to [Module 2: Simulation](../../module-2-simulation/index.md) →
