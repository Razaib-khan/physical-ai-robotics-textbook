---
id: 1-4-parameters
title: "Parameters and Launch Files"
sidebar_label: "1.4 Parameters & Launch"
sidebar_position: 4
description: Learn dynamic configuration with parameters and multi-node orchestration with launch files in ROS 2
keywords:
  - ros2
  - parameters
  - launch files
  - configuration
  - multi-node systems
difficulty: beginner
estimated_time: "60-75 minutes"
learning_outcomes:
  - "Declare and access parameters in ROS 2 nodes"
  - "Implement parameter callbacks for dynamic updates"
  - "Create launch files for multi-node systems"
  - "Configure nodes using YAML parameter files"
prerequisites:
  - "1-3-services-actions"
hardware_required: false
---

# Parameters and Launch Files

## Overview

Real-world robots need flexible configuration without code changes. This chapter covers:

- **Parameters**: Runtime configuration values that nodes can read and update
- **Launch files**: Scripts to start multiple nodes with their configurations

## What are ROS 2 Parameters?

Parameters are configuration values stored in nodes. They allow you to:

- Configure node behavior without recompiling
- Tune algorithms at runtime
- Share settings across multiple launches

### Parameter Types

| Type | Python Type | Example |
|------|-------------|---------|
| `bool` | `bool` | `True`, `False` |
| `int` | `int` | `42`, `-10` |
| `double` | `float` | `3.14159` |
| `string` | `str` | `"robot_1"` |
| `byte_array` | `bytes` | `b'\x01\x02'` |
| `bool_array` | `List[bool]` | `[True, False]` |
| `int_array` | `List[int]` | `[1, 2, 3]` |
| `double_array` | `List[float]` | `[1.0, 2.0]` |
| `string_array` | `List[str]` | `["a", "b"]` |

## Declaring and Using Parameters

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="python" label="Python" default>

```python title="parameter_node.py"
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class ParameterNode(Node):
    """Demonstrates parameter declaration and usage."""

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults and descriptions
        self.declare_parameter('robot_name', 'robot_1',
            ParameterDescriptor(description='Name of the robot'))

        self.declare_parameter('max_speed', 1.0,
            ParameterDescriptor(description='Maximum speed in m/s'))

        self.declare_parameter('enable_logging', True,
            ParameterDescriptor(description='Enable debug logging'))

        self.declare_parameter('waypoints', [0.0, 0.0, 1.0, 1.0],
            ParameterDescriptor(description='List of waypoint coordinates'))

        # Read parameter values
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value
        enable_logging = self.get_parameter('enable_logging').value
        waypoints = self.get_parameter('waypoints').value

        self.get_logger().info(f'Robot name: {robot_name}')
        self.get_logger().info(f'Max speed: {max_speed} m/s')
        self.get_logger().info(f'Logging enabled: {enable_logging}')
        self.get_logger().info(f'Waypoints: {waypoints}')

        # Create a timer to periodically check parameters
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        """Read parameters periodically (they may have changed)."""
        max_speed = self.get_parameter('max_speed').value
        self.get_logger().info(f'Current max_speed: {max_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

  </TabItem>
  <TabItem value="cpp" label="C++">

```cpp title="parameter_node.cpp"
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode()
    : Node("parameter_node")
    {
        // Declare parameters with defaults and descriptions
        rcl_interfaces::msg::ParameterDescriptor desc;

        desc.description = "Name of the robot";
        this->declare_parameter("robot_name", "robot_1", desc);

        desc.description = "Maximum speed in m/s";
        this->declare_parameter("max_speed", 1.0, desc);

        desc.description = "Enable debug logging";
        this->declare_parameter("enable_logging", true, desc);

        // Read parameter values
        std::string robot_name = this->get_parameter("robot_name").as_string();
        double max_speed = this->get_parameter("max_speed").as_double();
        bool enable_logging = this->get_parameter("enable_logging").as_bool();

        RCLCPP_INFO(this->get_logger(), "Robot name: %s", robot_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Max speed: %.2f m/s", max_speed);
        RCLCPP_INFO(this->get_logger(), "Logging enabled: %s",
            enable_logging ? "true" : "false");

        // Timer to check parameters
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ParameterNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        double max_speed = this->get_parameter("max_speed").as_double();
        RCLCPP_INFO(this->get_logger(), "Current max_speed: %.2f", max_speed);
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterNode>());
    rclcpp::shutdown();
    return 0;
}
```

  </TabItem>
</Tabs>

### Running with Command-Line Parameters

```bash title="Override parameters at startup"
ros2 run my_package parameter_node --ros-args \
    -p robot_name:=my_robot \
    -p max_speed:=2.5 \
    -p enable_logging:=false
```

## Parameter Callbacks

React to parameter changes at runtime with callbacks:

```python title="parameter_callback_node.py"
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class DynamicParameterNode(Node):
    """Node with dynamic parameter updates."""

    def __init__(self):
        super().__init__('dynamic_parameter_node')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('enabled', True)

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Use current rate for timer
        rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('Dynamic parameter node started')

    def parameter_callback(self, params):
        """Called when parameters are changed."""
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')

            # Validate parameter values
            if param.name == 'update_rate':
                if param.value <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason='update_rate must be positive')

                # Recreate timer with new rate
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / param.value, self.timer_callback)

        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Do work if enabled."""
        if self.get_parameter('enabled').value:
            self.get_logger().info('Working...')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Changing Parameters at Runtime

```bash title="Set parameter from CLI"
ros2 param set /dynamic_parameter_node update_rate 5.0
```

```bash title="Get current parameter value"
ros2 param get /dynamic_parameter_node update_rate
```

```bash title="List all parameters"
ros2 param list /dynamic_parameter_node
```

## YAML Parameter Files

Store parameters in YAML files for easy management:

```yaml title="config/robot_params.yaml"
# Parameters for the robot controller
parameter_node:
  ros__parameters:
    robot_name: "atlas"
    max_speed: 2.0
    enable_logging: true
    waypoints: [0.0, 0.0, 5.0, 5.0, 10.0, 0.0]

# Parameters for another node
sensor_node:
  ros__parameters:
    update_rate: 30.0
    sensor_topic: "/camera/image_raw"
```

### Loading Parameters from YAML

```bash title="Load parameters from file"
ros2 run my_package parameter_node --ros-args \
    --params-file config/robot_params.yaml
```

## What are Launch Files?

Launch files automate starting multiple nodes with their configurations. They can:

- Start multiple nodes simultaneously
- Load parameter files
- Remap topic names
- Set namespace prefixes
- Include other launch files

### Launch File Formats

ROS 2 supports three launch file formats:

| Format | Extension | Use Case |
|--------|-----------|----------|
| Python | `.launch.py` | Most flexible, conditional logic |
| XML | `.launch.xml` | Simple, readable |
| YAML | `.launch.yaml` | Compact, easy to read |

## Creating Launch Files

<Tabs>
  <TabItem value="python" label="Python" default>

```python title="launch/robot_launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_package')

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_1',
        description='Name of the robot'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='1.0',
        description='Maximum robot speed'
    )

    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

    # Define nodes
    controller_node = Node(
        package='my_package',
        executable='controller_node',
        name='robot_controller',
        output='screen',
        parameters=[
            params_file,
            {'robot_name': LaunchConfiguration('robot_name')},
            {'max_speed': LaunchConfiguration('max_speed')}
        ],
        remappings=[
            ('/cmd_vel', '/robot1/cmd_vel'),
            ('/odom', '/robot1/odom')
        ]
    )

    sensor_node = Node(
        package='my_package',
        executable='sensor_node',
        name='sensor_processor',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        robot_name_arg,
        max_speed_arg,
        controller_node,
        sensor_node
    ])
```

  </TabItem>
  <TabItem value="xml" label="XML">

```xml title="launch/robot_launch.xml"
<launch>
    <!-- Declare arguments -->
    <arg name="robot_name" default="robot_1" description="Name of the robot"/>
    <arg name="max_speed" default="1.0" description="Maximum robot speed"/>

    <!-- Controller node -->
    <node pkg="my_package" exec="controller_node" name="robot_controller" output="screen">
        <param from="$(find-pkg-share my_package)/config/robot_params.yaml"/>
        <param name="robot_name" value="$(var robot_name)"/>
        <param name="max_speed" value="$(var max_speed)"/>
        <remap from="/cmd_vel" to="/robot1/cmd_vel"/>
        <remap from="/odom" to="/robot1/odom"/>
    </node>

    <!-- Sensor node -->
    <node pkg="my_package" exec="sensor_node" name="sensor_processor" output="screen">
        <param from="$(find-pkg-share my_package)/config/robot_params.yaml"/>
    </node>
</launch>
```

  </TabItem>
</Tabs>

### Running Launch Files

```bash title="Run a launch file"
ros2 launch my_package robot_launch.py
```

```bash title="Override launch arguments"
ros2 launch my_package robot_launch.py robot_name:=atlas max_speed:=2.5
```

```bash title="Show launch arguments"
ros2 launch my_package robot_launch.py --show-args
```

## Advanced Launch Features

### Conditional Logic

```python title="launch/conditional_launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim = DeclareLaunchArgument('use_sim', default_value='true')

    # Only start simulation node if use_sim is true
    sim_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    # Start hardware driver unless using simulation
    hardware_node = Node(
        package='my_package',
        executable='hardware_driver',
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_sim,
        sim_node,
        hardware_node
    ])
```

### Including Other Launch Files

```python title="launch/full_system_launch.py"
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_package')

    # Include another launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'robot_name': 'main_robot',
            'max_speed': '1.5'
        }.items()
    )

    return LaunchDescription([robot_launch])
```

### Namespaces

```python title="Using namespaces for multi-robot"
robot1 = Node(
    package='my_package',
    executable='controller',
    namespace='robot1',  # All topics/services prefixed with /robot1/
    name='controller'
)

robot2 = Node(
    package='my_package',
    executable='controller',
    namespace='robot2',  # All topics/services prefixed with /robot2/
    name='controller'
)
```

## Parameter CLI Reference

| Command | Description |
|---------|-------------|
| `ros2 param list <node>` | List all parameters |
| `ros2 param get <node> <param>` | Get parameter value |
| `ros2 param set <node> <param> <value>` | Set parameter value |
| `ros2 param describe <node> <param>` | Show parameter description |
| `ros2 param dump <node>` | Dump all parameters to YAML |
| `ros2 param load <node> <file>` | Load parameters from YAML |

## Best Practices

:::tip Parameter Design
1. **Declare all parameters**: Always declare parameters with defaults
2. **Use descriptive names**: `max_linear_velocity` not `mlv`
3. **Document parameters**: Add descriptions for all parameters
4. **Validate values**: Use callbacks to reject invalid values
5. **Group related parameters**: Use YAML files for organization
:::

:::tip Launch File Design
1. **Use arguments**: Make launch files configurable
2. **Modular design**: Split large systems into multiple launch files
3. **Include files**: Reuse common configurations
4. **Use namespaces**: Enable multi-robot deployments
5. **Document**: Add comments explaining each section
:::

:::warning Common Pitfalls
- **Undeclared parameters**: Parameters must be declared before use
- **Type mismatches**: Setting wrong type causes errors
- **Missing files**: Check paths to YAML and included launch files
- **Circular includes**: Launch files including each other
:::

## Summary

In this chapter, you learned:

1. **Parameters** enable runtime configuration of nodes
2. **Callbacks** allow dynamic response to parameter changes
3. **YAML files** organize parameters for easy management
4. **Launch files** automate multi-node system startup
5. **Advanced features** like conditions and namespaces enable complex deployments

## Exercises

See [Module 1 Exercises](./exercises/module-1-exercises.md) for hands-on practice with parameters and launch files.

---

**Next**: [1.5 URDF Robot Description](./1-5-urdf.md) - Learn to describe robot kinematics and visualization
