---
id: capstone-troubleshooting
title: "Capstone Project: Troubleshooting"
sidebar_label: "Troubleshooting"
sidebar_position: 4
description: "A guide to common issues and their solutions encountered during the Capstone Project, helping you debug and overcome challenges in integrating complex robotics systems."
keywords: ['capstone', 'troubleshooting', 'debugging', 'ros2', 'simulation', 'vla']
prerequisites:
  - "capstone-evaluation"
---

## Troubleshooting Your Autonomous Humanoid

Developing a complex system like an autonomous humanoid involves integrating many different components, and it's inevitable that you'll encounter challenges. This section provides a guide to common issues you might face during your Capstone Project and offers strategies for troubleshooting them.

### General Debugging Principles

-   **Isolate the Problem**: When something goes wrong, try to narrow down where the error is occurring. Start by checking the individual components.
-   **Check ROS 2 Graph**: Use `rqt_graph` to visualize your ROS 2 nodes and their connections. Ensure all expected nodes are running and topics/services are correctly connected.
-   **Log Messages**: Pay close attention to `ros2 log` messages. They often provide valuable clues about what's going wrong.
-   **Incremental Development**: Build and test your system piece by piece. Don't try to implement everything at once.
-   **Version Control**: Regularly commit your changes and use branches to experiment.

### Common Issues and Solutions

#### 1. ROS 2 Communication Problems

-   **Issue**: Nodes not communicating, topics not visible, or services not responding.
    -   **Solution**:
        -   **Check `ros2 daemon status`**: Ensure the ROS 2 daemon is running. If not, `ros2 daemon start`.
        -   **Verify Node Names and Remappings**: Ensure node names are unique and topic/service names match exactly across publishers/subscribers or clients/servers. Check for typos.
        -   **`rqt_graph`**: Use this tool to visually inspect your ROS 2 computational graph. Are all nodes connected as expected? Are there unexpected nodes?
        -   **`ros2 topic info <topic_name>` / `ros2 service info <service_name>`**: Verify that publishers/subscribers or service clients/servers are registered for the correct interfaces.
        -   **Firewall**: Ensure no firewall is blocking ROS 2 communication (ports 11311/8888, and dynamic UDP ports).

#### 2. Simulation Environment Issues

-   **Issue**: Robot not spawning, objects behaving strangely, or simulation crashing.
    -   **Solution**:
        -   **URDF/SDF Errors**: Check your robot's URDF/SDF files for syntax errors or incorrect joint/link definitions. Use `check_urdf` to validate URDF.
        -   **Model Paths**: Ensure Gazebo/Isaac Sim can find your models. Check `GAZEBO_MODEL_PATH` or Isaac Sim asset paths.
        -   **Physics Properties**: Incorrect mass, inertia, or friction settings can lead to unstable physics. Start with simple values and gradually increase complexity.
        -   **Collision Meshes**: Ensure collision meshes are simplified and correctly aligned with visual meshes. Complex collision meshes can lead to performance issues and inaccurate physics.
        -   **Simulation Performance**: If the simulation is running very slowly, reduce the complexity of the world or the number of simulated sensors.

#### 3. Vision System Problems

-   **Issue**: Object detection failing, incorrect 3D poses, or no image stream.
    -   **Solution**:
        -   **Camera Topic**: Verify the camera is publishing images using `ros2 topic echo /camera/image_raw`.
        -   **TF Tree**: Use `ros2 run tf2_ros tf2_echo <source_frame> <target_frame>` to check your TF tree. Ensure the camera frame is correctly linked to the robot's base frame.
        -   **Model Loading**: Ensure your YOLO or other detection model is loading correctly and can identify objects. Test the model independently on static images.
        -   **Depth Data**: If using depth, ensure it's accurate. Calibration errors or incorrect sensor settings can lead to wrong 3D estimations.
        -   **Lighting Conditions**: In simulation, poor lighting can affect vision algorithms. Adjust ambient light or add spotlights.

#### 4. Language Interface Challenges

-   **Issue**: LLM parsing commands incorrectly, or failing to extract entities.
    -   **Solution**:
        -   **Prompt Engineering**: Refine your LLM prompt. Provide more examples of desired input/output pairs. Explicitly instruct the LLM on expected JSON format and what to do with ambiguous or unmentioned entities.
        -   **API Key/Connectivity**: Ensure your LLM API key is valid and you have an active internet connection (if using a cloud LLM).
        -   **Rate Limits**: Check if you're hitting API rate limits if commands are failing intermittently. Implement retry mechanisms.
        -   **Context**: For complex commands, consider providing the LLM with additional context about the robot's current state or capabilities.

#### 5. Robot Action Execution Failures

-   **Issue**: Robot not moving as expected, manipulation failing, or jerky movements.
    -   **Solution**:
        -   **Joint Controllers**: Verify that your robot's joint controllers are correctly loaded and active.
        -   **Command Velocity**: Check if your navigation controller is publishing valid `geometry_msgs/Twist` commands to `/cmd_vel` using `ros2 topic echo /cmd_vel`.
        -   **Inverse Kinematics (IK)**: If using IK, ensure the solver is correctly configured for your robot's kinematics. IK failures are common for unreachable poses.
        -   **Collision Avoidance**: Ensure your motion planner is correctly configured for collision avoidance. Visualizing planned paths in RViz can help identify issues.
        -   **Grasping Parameters**: Fine-tune grasping parameters (e.g., gripper force, closing distance) in simulation.

### Cross-Module Integration Patterns

Many issues in a Capstone project stem from incorrect integration between modules.

-   **Perception to Planning**: Ensure your object detection and pose estimation outputs (e.g., `PoseStamped` messages) are in the correct coordinate frame and are consumed correctly by your planning module.
-   **Planning to Action**: Verify that your action plans (e.g., sequences of target poses or joint commands) are correctly formatted for your robot's control interfaces.
-   **Feedback Loops**: Implement clear feedback from action execution (e.g., success/failure of a grasp) back to the planning and decision-making modules.

By systematically approaching these troubleshooting steps, you will be well-equipped to diagnose and resolve the inevitable challenges that arise in building your autonomous humanoid robot. Good luck!
