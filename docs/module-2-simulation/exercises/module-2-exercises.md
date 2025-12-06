---
id: module-2-exercises
title: "Module 2: Exercises"
sidebar_label: "Exercises"
description: "Practice your simulation skills with these hands-on exercises for Gazebo and Unity."
keywords: ['exercises', 'simulation', 'gazebo', 'unity', 'ros2']
difficulty: 'mixed'
estimated_time: "3-4 hours"
---

## Exercise 1: Gazebo Obstacle Course

**Difficulty**: Beginner
**Estimated Time**: 30-40 minutes
**Objective**: Create a simple Gazebo world with a few obstacles and spawn a robot into it.

### Instructions

1.  Create a new world file named `my_obstacle_course.world`.
2.  Inside the world, include the `sun` and `ground_plane` models.
3.  Add at least three static obstacles using simple shapes (e.g., box, cylinder, sphere).
4.  Position the obstacles to create a simple course for a robot to navigate.
5.  Launch your world in Gazebo.
6.  Use the `ros2 run ros_gz_sim create` command to spawn a robot (from a URDF file) into the world.

### Validation

Your simulation should show the robot and the obstacles you created. The robot should fall to the ground plane and not through it. The obstacles should be static and not move.

<details>
<summary>Show Solution</summary>

**`my_obstacle_course.world`**:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <model name="wall_1">
      <pose>5 0 0.5 0 0 1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><box><size>7.5 0.2 1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>7.5 0.2 1</size></box></geometry></visual>
      </link>
    </model>

    <model name="cylinder_1">
      <pose>2 3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>0.5</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.5</radius><length>1.0</length></cylinder></geometry></visual>
      </link>
    </model>
  </world>
</sdf>
```

**Commands**:
```bash
# Terminal 1
gazebo my_obstacle_course.world

# Terminal 2
ros2 run ros_gz_sim create -file your_robot.urdf -name my_robot -x 0 -y -2
```

**Explanation**: We've created a world with a wall and a cylinder. The `<static>true</static>` tag ensures they don't move. We then spawn the robot at a specific starting position using the `-x` and `-y` arguments.

</details>

---

## Exercise 2: Adding a Camera

**Difficulty**: Intermediate
**Estimated Time**: 35-45 minutes
**Objective**: Add a camera sensor to your robot's URDF and visualize the images in ROS 2.

### Instructions

1.  Modify your robot's URDF file to include a camera sensor plugin.
2.  Position the camera on a `camera_link` on the front of your robot.
3.  Configure the camera to publish images to a ROS 2 topic.
4.  Launch your Gazebo simulation with the updated robot.
5.  Use `ros2 topic list` and `ros2 topic echo` to verify that the camera data is being published.
6.  Use `rviz2` to visualize the `sensor_msgs/Image` messages.

### Validation

You should be able to see the live camera feed from your simulated robot in RViz2.

<details>
<summary>Show Solution</summary>

**URDF Snippet**:
Add this to your URDF file, making sure `camera_link` is properly defined and connected to your robot's base.
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=camera/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Verification**:
```bash
# Check topic
ros2 topic list | grep /camera/image_raw

# Visualize in RViz2
rviz2
```
In RViz2, add a display by topic and select `/camera/image_raw`.

**Explanation**: The `libgazebo_ros_camera.so` plugin creates a ROS 2 publisher for the camera data. We remap the output to a clean topic name. RViz2 can subscribe to this topic and render the images.

</details>

---

## Exercise 3: LiDAR-based Wall Follower

**Difficulty**: Intermediate
**Estimated Time**: 40-50 minutes
**Objective**: Create a Python node that uses LiDAR data to make a robot follow a wall.

### Instructions

1.  Add a LiDAR sensor to your robot's URDF.
2.  Create a Python node that subscribes to the `/scan` topic.
3.  In your node, process the `LaserScan` message to find the distance to the wall on the robot's right side.
4.  Implement a simple proportional controller:
    - If the robot is too far from the wall, turn slightly towards it.
    - If the robot is too close to the wall, turn slightly away from it.
    - Otherwise, drive straight.
5.  Your node should publish `geometry_msgs/Twist` messages to `/cmd_vel` to control the robot. (You'll need a differential drive plugin in your URDF for this to work).

### Validation

When you place the robot near a wall in the simulation and run your node, it should drive alongside the wall, maintaining a relatively constant distance.

<details>
<summary>Show Solution</summary>

**Python Node Snippet**:
```python
# ... (imports and class definition)
def scan_callback(self, msg):
    # Assuming the right side of the robot corresponds to the last 90 degrees of the scan
    right_side_ranges = msg.ranges[0:90]
    avg_dist_to_wall = sum(right_side_ranges) / len(right_side_ranges)

    twist_msg = Twist()
    target_dist = 1.0  # meters
    error = target_dist - avg_dist_to_wall

    if abs(error) > 0.1:
        # Proportional controller for turning
        twist_msg.angular.z = 0.5 * error
    
    # Always move forward
    twist_msg.linear.x = 0.2

    self.publisher_.publish(twist_msg)
```

**Explanation**: This is a very simple wall-following logic. It takes the average of the LiDAR ranges on the right side and uses a proportional controller to adjust the robot's angle. A real wall-follower would be more robust, but this demonstrates the core concept of using sensor data to generate control commands. You will need to add a publisher for `/cmd_vel` to the node.

</details>

---

## Exercise 4: Unity and ROS 2 Integration

**Difficulty**: Advanced
**Estimated Time**: 60-75 minutes
**Objective**: Create a Unity scene where you can control a cube's position via ROS 2 messages.

### Instructions

1.  Set up a new Unity project with the `ROS-TCP-Connector`.
2.  Create a simple scene with a Plane and a Cube.
3.  On the ROS 2 side, make sure the `ros_tcp_endpoint` is running.
4.  Create a C# script in Unity that:
    - Subscribes to a ROS 2 topic (e.g., `/cube_goal`) of type `geometry_msgs/Point`.
    - When a message is received, smoothly moves the Cube to the position specified in the message.
5.  From your ROS 2 terminal, use `ros2 topic pub` to send goal positions to the cube.

### Validation

When you publish a `Point` message, the cube in your Unity scene should move from its current position to the new goal position.

<details>
<summary>Show Solution</summary>

**C# Script Snippet**:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class GoalSubscriber : MonoBehaviour
{
    public float speed = 2.0f;
    private Vector3 goalPosition;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PointMsg>("cube_goal", GoalCallback);
        goalPosition = transform.position;
    }

    void GoalCallback(PointMsg goal)
    {
        goalPosition = new Vector3((float)goal.x, (float)goal.y, (float)goal.z);
    }

    void Update()
    {
        transform.position = Vector3.MoveTowards(transform.position, goalPosition, speed * Time.deltaTime);
    }
}
```

**ROS 2 Command**:
```bash
ros2 topic pub /cube_goal geometry_msgs/Point "{x: 5.0, y: 0.0, z: 1.0}" -1
```

**Explanation**: The C# script subscribes to the `/cube_goal` topic. The `GoalCallback` function updates the `goalPosition` variable whenever a new message arrives. The `Update` function then uses `Vector3.MoveTowards` to smoothly interpolate the cube's position towards the goal each frame.

</details>