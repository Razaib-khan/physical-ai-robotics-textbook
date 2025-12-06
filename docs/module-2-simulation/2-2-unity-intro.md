---
id: 2-2-unity-intro
title: "2.2: Introduction to Unity"
sidebar_label: "Unity Introduction"
sidebar_position: 2
description: "Explore Unity as a powerful, visually-rich simulation environment for robotics, and learn how to connect it with ROS 2."
keywords: ['unity', 'ros2', 'simulation', 'robotics', 'ros-tcp-connector']
difficulty: beginner
estimated_time: "60-75 minutes"
learning_outcomes:
  - "Understand the advantages of using Unity for robotics simulation."
  - "Set up the ROS-TCP-Connector to link Unity with ROS 2."
  - "Create a basic scene in Unity for a robot."
  - "Publish data from Unity to a ROS 2 topic."
prerequisites:
  - "module-1-ros2/1-2-nodes-topics"
hardware_required: true
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Introduction to Unity for Robotics

While Gazebo is a robotics-native simulator, game engines like Unity are increasingly popular for robotics simulation. Unity offers state-of-the-art rendering capabilities, a massive asset store, and an intuitive graphical editor, making it an excellent choice for creating photorealistic and complex interactive environments.

### Unity vs. Gazebo

<Tabs>
  <TabItem value="gazebo" label="Gazebo" default>
    **Focus:** Robotics-first simulator.
    **Strengths:**
    - Deep integration with ROS.
    - Strong physics simulation for robotics (e.g., contact points, friction).
    - Lightweight and fast for non-graphical simulation.
    - Large collection of open-source robot models.
    **Weaknesses:**
    - Less visually impressive than modern game engines.
    - Steeper learning curve for creating custom worlds.
  </TabItem>
  <TabItem value="unity" label="Unity">
    **Focus:** General-purpose game engine.
    **Strengths:**
    - Photorealistic rendering.
    - Intuitive graphical editor and large asset store.
    - Excellent for human-robot interaction and synthetic data generation.
    - C# scripting is powerful and widely used.
    **Weaknesses:**
    - ROS integration requires a dedicated plugin (ROS-TCP-Connector).
    - Physics engine is optimized for games, may need tuning for robotic accuracy.
  </TabItem>
</Tabs>

### Connecting Unity and ROS 2: The ROS-TCP-Connector

To communicate between Unity and ROS 2, we use the `ROS-TCP-Connector`. This package consists of two parts:
1.  A **Unity package** that you import into your Unity project.
2.  A **ROS 2 package** (`ros_tcp_endpoint`) that runs on the ROS side.

These two parts establish a TCP connection, forwarding messages between ROS 2 topics/services and your Unity scene.

### Setting Up a Unity Scene

1.  **Create a new 3D project in Unity Hub.**
2.  **Install the ROS-TCP-Connector** from the Unity Asset Store or via the Package Manager.
3.  **Add a ROSConnection** prefab to your scene. This object manages the connection to your ROS 2 network. Configure its IP address to point to your machine running ROS.
4.  **Create a simple environment,** for example, a "Plane" for the ground and a "Cube" to represent a robot.

### Publishing from Unity to ROS 2

Let's create a simple C# script to publish the position of a cube in Unity to a ROS 2 topic.

1.  Create a C# script named `PositionPublisher.cs`.
2.  Attach it to the Cube object in your scene.
3.  Add the following code:

```csharp title="PositionPublisher.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class PositionPublisher : MonoBehaviour
{
    // ROS Connector
    ROSConnection ros;
    // ROS Topic
    public string topicName = "cube_position";
    // ROS Message
    private PointMsg cubePos;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        // Register the publisher
        ros.RegisterPublisher<PointMsg>(topicName);
        // Initialize the message
        cubePos = new PointMsg();
    }

    void Update()
    {
        // Get the cube's position
        Vector3 position = transform.position;
        // Update the message
        cubePos.x = position.x;
        cubePos.y = position.y;
        cubePos.z = position.z;

        // Publish the message
        ros.Publish(topicName, cubePos);
    }
}
```
This script gets the position of the object it's attached to every frame and publishes it as a `geometry_msgs/Point` message to the `/cube_position` topic. You can then use `ros2 topic echo /cube_position` in your terminal to see the data stream from Unity.

:::warning Hardware Note
Unity is a graphically intensive application. A dedicated graphics card (NVIDIA RTX series recommended) is highly advised for a smooth experience, especially when building complex scenes. Check the official Unity system requirements for your operating system. A free Personal license is sufficient for this textbook.
:::

In the next chapters, we will explore how to add more sophisticated sensors to our simulated robots in both Gazebo and Unity.
