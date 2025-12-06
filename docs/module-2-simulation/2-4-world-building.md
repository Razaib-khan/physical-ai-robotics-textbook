---
id: 2-4-world-building
title: "2.4: World Building"
sidebar_label: "World Building"
sidebar_position: 4
description: "Learn to create rich and complex simulation environments in Gazebo and Unity, including terrains, obstacles, and lighting."
keywords: ['world building', 'gazebo', 'unity', 'simulation', 'sdf']
difficulty: intermediate
estimated_time: "50-60 minutes"
learning_outcomes:
  - "Build a custom world in Gazebo with multiple objects."
  - "Understand how to set physics properties for objects."
  - "Structure a complex scene in Unity."
  - "Appreciate the trade-offs between simulation complexity and performance."
prerequisites:
  - "2-1-gazebo-intro"
  - "2-2-unity-intro"
hardware_required: false
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Crafting Your Robot's Universe

The world your robot inhabits is just as important as the robot itself. A well-designed world allows you to test specific scenarios, from navigating a simple maze to manipulating objects on a cluttered table. This chapter covers the basics of world building in both Gazebo and Unity.

### World Building in Gazebo

In Gazebo, the world is defined in an SDF file. You can build a world by combining models from Gazebo's database or by creating your own models.

Here is an example of a world file with a few simple shapes, creating a small obstacle course.

```xml title="obstacle_course.world"
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="box_obstacle">
      <pose>2.0 2.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="sphere_obstacle">
      <pose>-1.5 1.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere><radius>0.5</radius></sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.5</radius></sphere>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```
This world includes a box and a sphere. Each model has a `<pose>` tag to define its position and orientation, and `<collision>` and `<visual>` tags. The `<collision>` geometry is used by the physics engine, while the `<visual>` geometry is for rendering. They can be different to simplify physics calculations.

### Scene Building in Unity

In Unity, world building (or scene building) is a more graphical process. You can drag and drop objects from the Asset Store or create your own using Unity's built-in tools.

A typical scene for a robotics simulation might have a hierarchy like this:

```
- Scene Root
  - Environment
    - Ground_Plane
    - Walls
      - Wall_North
      - Wall_South
    - Obstacles
      - Cube_1
      - Sphere_1
      - Cylinder_1
  - Robots
    - My_Awesome_Robot
      - base_link
      - camera_link
      - lidar_link
  - Lights
    - Directional_Light
  - ROS
    - ROSConnection
```
This hierarchical structure helps keep your scene organized, especially as it becomes more complex. You can create empty `GameObject`s (like `Environment`, `Robots`, `Lights`) to act as folders for other objects.

### Physics Properties

The realism of your simulation depends heavily on the physics properties of the objects. You can define properties like mass, friction, and restitution (bounciness).

<Tabs>
  <TabItem value="gazebo" label="Gazebo" default>
    In SDF, you can add an `<inertial>` tag to define mass and inertia, and a `<surface>` tag to define friction.
    ```xml
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.083</iyy>
          <iyz>0.0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      ...
    </link>
    ```
  </TabItem>
  <TabItem value="unity" label="Unity">
    In Unity, you add a `Rigidbody` component to an object to give it physics properties like mass and drag. For friction and restitution, you create a `Physic Material` and apply it to the object's `Collider`.
  </TabItem>
</Tabs>

:::warning Hardware Note
The complexity of your simulation world directly impacts performance. High-polygon models, complex lighting, and a large number of objects with physics can significantly slow down your simulation. Always consider the trade-off between realism and real-time performance. It is often a good practice to use simplified "collision meshes" for physics calculations that are much simpler than the detailed "visual meshes".
:::
