---
id: 2-3-sensors
title: "2.3: Simulating Sensors"
sidebar_label: "Simulating Sensors"
sidebar_position: 3
description: "Learn how to add and configure simulated sensors like cameras, LiDAR, and IMUs to your robot in Gazebo and Unity."
keywords: ['sensors', 'lidar', 'camera', 'imu', 'gazebo', 'unity', 'simulation']
difficulty: intermediate
estimated_time: "60-75 minutes"
learning_outcomes:
  - "Add camera, LiDAR, and IMU sensors to a simulated robot."
  - "Understand and configure sensor noise models."
  - "Subscribe to and process sensor data in ROS 2."
  - "Visualize sensor data using ROS 2 tools like RViz."
prerequisites:
  - "2-1-gazebo-intro"
  - "module-1-ros2/1-2-nodes-topics"
hardware_required: false
---

## Simulating the Robot's Senses

A robot is only as good as its perception of the world. In simulation, we can equip our robots with a variety of virtual sensors that mimic their real-world counterparts. This chapter will cover how to add three common sensors: cameras, LiDAR, and IMUs.

:::info Diagram Placeholder: Robot with Sensors
**Description**: A 3D rendering of a simple differential drive robot.
- A camera is mounted on the front, with its field of view (frustum) visualized as a transparent pyramid.
- A LiDAR is mounted on top, with its 360-degree scan visualized as a flat red disk.
- An IMU is shown as a small box at the center of the robot's base.
**Suggested Tool**: Blender or Inkscape
**Dimensions**: 1200x800px
**Alt Text**: "A simulated robot showing the placement and field of view of its camera, LiDAR, and IMU sensors."
:::

### Adding Sensors in Gazebo

In Gazebo, sensors are added to your robot's URDF model as plugins. These plugins are part of the `gazebo_ros_pkgs` package.

#### Camera

A camera sensor simulates a standard video camera, publishing images to a ROS 2 topic.

```xml title="camera_sensor.urdf.xacro"
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.396263</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>image_raw:=color/image_raw</remapping>
        <remapping>camera_info:=color/camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

This snippet defines a camera with a 30Hz update rate, 800x600 resolution, and publishes to `/my_robot/color/image_raw`.

#### LiDAR

A LiDAR sensor simulates a laser scanner, publishing `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2` messages.

```xml title="lidar_sensor.urdf.xacro"
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
This defines a 360-degree LiDAR with a 12-meter range, publishing to `/my_robot/scan`.

<details>
<summary>Advanced Topic: Sensor Noise</summary>

Real-world sensors are noisy. To make your simulation more realistic, you should add noise models. Gazebo allows you to add Gaussian noise to most sensors.

For the camera:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.007</stddev>
</noise>
```

For the LiDAR:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```
Adding noise is crucial for developing robust perception algorithms that will work on the physical robot.

</details>

### Subscribing to Sensor Data

Once your sensors are running in the simulation, you can subscribe to their topics in ROS 2 just like you would with a real robot.

Here is a Python script that subscribes to the LiDAR data and finds the distance to the closest obstacle.

```python title="obstacle_avoider.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import min

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.subscription = self.create_subscription(
            LaserScan,
            '/my_robot/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        min_dist = min(msg.ranges)
        self.get_logger().info(f'Closest obstacle is {min_dist:.2f} meters away.')

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()
    rclpy.spin(obstacle_avoider)
    obstacle_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

You can run this node and see the stream of distance data from the simulated LiDAR. Visualizing the data in RViz is also highly recommended to get a better understanding of what the robot "sees".
