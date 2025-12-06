---
id: module-4-exercises
title: "Module 4: Exercises"
sidebar_label: "Exercises"
description: "Test your understanding of Vision-Language-Action (VLA) models and integrate vision, language, and robotic control."
keywords: ['exercises', 'vla', 'vision', 'language', 'robotics', 'ai']
difficulty: 'mixed'
estimated_time: "4-5 hours"
---

## Exercise 1: Object Detection Pipeline

**Difficulty**: Intermediate
**Estimated Time**: 50-60 minutes
**Objective**: Build a ROS 2 node that performs object detection on a simulated camera feed and publishes the detected object's pose.

### Instructions

1.  Start a Gazebo simulation with a robot equipped with a camera (as in Module 2).
2.  Create a ROS 2 Python node that subscribes to the camera's image topic (`/camera/image_raw`).
3.  Integrate a pre-trained object detection model (e.g., YOLO from `ultralytics`) to identify common objects (e.g., "cup", "bottle", "book") in the camera feed.
4.  For each detected object, calculate its approximate 3D position relative to the camera (you can assume a known distance for simplicity, or use depth data if available from your simulated camera).
5.  Publish the `geometry_msgs/PoseStamped` of the *closest* detected object of a specific class (e.g., "cup") to a new ROS 2 topic (e.g., `/detected_object_pose`).

### Validation

When a "cup" (or another target object) is within the camera's view in the simulation, your node should publish its pose to `/detected_object_pose`. You can verify this using `ros2 topic echo /detected_object_pose` and visualizing the pose in RViz2.

<details>
<summary>Show Solution</summary>

**Python Node Snippet**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectPoseDetector(Node):
    def __init__(self):
        super().__init__('object_pose_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') # Load pre-trained YOLOv8 nano model
        self.target_class_id = self.get_target_class_id("cup") # Example: get ID for 'cup'

    def get_target_class_id(self, class_name):
        # This is a simplified way to get class ID. In a real scenario, you'd map class names to model's class IDs.
        # Check model.names for available classes.
        for idx, name in self.model.names.items():
            if name == class_name:
                return idx
        self.get_logger().error(f"Class '{class_name}' not found in YOLO model.")
        return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        results = self.model(cv_image, verbose=False)
        detections = results[0].boxes

        closest_object_pose = None
        min_distance = float('inf')

        for det in detections:
            if det.cls == self.target_class_id:
                # Get bounding box (x_center, y_center, width, height)
                x_c, y_c, w, h = det.xywh[0].cpu().numpy()
                
                # --- Simplified 3D position estimation (replace with actual depth if available) ---
                # Assume a fixed camera height and estimate distance based on object size or camera intrinsics
                # For simplicity, let's just assume a fixed distance for now for any detected cup
                estimated_depth = 1.0 # meters (this needs to be smarter with actual depth data)
                
                # Calculate approximate 3D position relative to camera frame
                # This is a placeholder; requires camera intrinsics and image to 3D projection
                # For a real implementation, you'd use depth image or camera matrix.
                x_cam = (x_c - cv_image.shape[1] / 2) / (cv_image.shape[1] / 2) * estimated_depth * 0.5 # rough estimate
                y_cam = (y_c - cv_image.shape[0] / 2) / (cv_image.shape[0] / 2) * estimated_depth * 0.5 # rough estimate
                z_cam = estimated_depth # assuming Z-axis points forward from camera
                
                # Further refine by finding the 'closest' based on a heuristic, e.g., smallest bbox or smallest depth
                current_distance = estimated_depth # Use actual depth if available
                if current_distance < min_distance:
                    min_distance = current_distance
                    
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "camera_link" # Or your robot's camera frame
                    pose_msg.pose.position.x = x_cam
                    pose_msg.pose.position.y = y_cam
                    pose_msg.pose.position.z = z_cam
                    # Orientation can be set to identity for now
                    pose_msg.pose.orientation.w = 1.0
                    closest_object_pose = pose_msg
        
        if closest_object_pose:
            self.pose_publisher.publish(closest_object_pose)
            self.get_logger().info(f"Published pose for {self.model.names[self.target_class_id]}: x={closest_object_pose.pose.position.x:.2f}, y={closest_object_pose.pose.position.y:.2f}, z={closest_object_pose.pose.position.z:.2f}")
        
        annotated_frame = results[0].plot()
        cv2.imshow("Object Detections", annotated_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    object_pose_detector = ObjectPoseDetector()
    rclpy.spin(object_pose_detector)
    object_pose_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Explanation**: This solution provides a basic framework. The critical part of obtaining the 3D pose (`x_cam`, `y_cam`, `z_cam`) from a 2D bounding box requires knowledge of camera intrinsics (focal length, principal point) and, ideally, a depth image from the camera. Without actual depth, the `estimated_depth` is a placeholder. For a more accurate solution, you would use a depth camera and project the 2D pixel coordinates of the detected object into 3D space using the depth value.

</details>

---

## Exercise 2: Natural Language Command Parser

**Difficulty**: Intermediate
**Estimated Time**: 45-55 minutes
**Objective**: Create a Python script that uses a Large Language Model (LLM) to parse natural language commands into a robot-executable JSON format.

### Instructions

1.  Choose an LLM API (e.g., OpenAI, Google Gemini, Hugging Face). You will need an API key if using a cloud-based service.
2.  Write a Python function `parse_command(command_string)` that takes a natural language command as input.
3.  Craft a suitable prompt for your chosen LLM to instruct it to extract:
    -   An `action` (e.g., "pick", "place", "move").
    -   An `object` (e.g., "red block", "mug").
    -   An optional `location` (e.g., "table", "shelf").
4.  The LLM's response should be a JSON object containing these extracted entities.
5.  Test your function with various commands like:
    -   "Robot, grab the green cube."
    -   "Move to the kitchen."
    -   "Put the book on the chair."

### Validation

Your `parse_command` function should consistently return correctly structured JSON objects, extracting the relevant `action`, `object`, and `location` from the input natural language commands.

<details>
<summary>Show Solution</summary>

**Python Script (using OpenAI API)**:
```python
import openai
import json
import os # For API key

# Replace with your actual OpenAI API key (or set as environment variable)
# openai.api_key = os.getenv("OPENAI_API_KEY")

def parse_command(command_string):
    prompt_messages = [
        {"role": "system", "content": """
        You are a helpful assistant designed to parse natural language commands for a robot.
        Extract the 'action', 'object', and 'location' from the user's command into a JSON object.
        If an entity is not explicitly mentioned, omit it.
        Example 1: \"Pick up the red block.\" -> {\"action\": \"pick up\", \"object\": \"red block\"}
        Example 2: \"Go to the charging station.\" -> {\"action\": \"go to\", \"location\": \"charging station\"}
        Example 3: \"Place the item on the table.\" -> {\"action\": \"place\", \"object\": \"item\", \"location\": \"table\"}
        """},
        {"role": "user", "content": command_string}
    ]

    try:
        response = openai.chat.completions.create(
            model="gpt-4o-mini", # Or gpt-3.5-turbo, or gpt-4
            messages=prompt_messages,
            response_format={"type": "json_object"}
        )
        return json.loads(response.choices[0].message.content)
    except Exception as e:
        print(f"Error parsing command with LLM: {e}")
        return {"error": str(e)}

# Test cases
commands = [
    "Robot, grab the green cube.",
    "Move to the kitchen.",
    "Put the book on the chair.",
    "Inspect the broken sensor.",
    "What is my battery level?" # Example of a command it might not handle well yet
]

for cmd in commands:
    print(f"Command: '{cmd}'")
    parsed = parse_command(cmd)
    print(f"Parsed: {json.dumps(parsed, indent=2)}
")

```
**Explanation**: This solution uses OpenAI's API. The key is the `system` message in the `prompt_messages` list, which guides the LLM on how to parse the command and specifies the desired JSON output format. Ensure you have the `openai` Python package installed and your API key configured. Similar approaches can be used with other LLMs.

</details>

---

## Exercise 3: VLA Pick-and-Place System

**Difficulty**: Advanced
**Estimated Time**: 90-120 minutes
**Objective**: Combine vision and language modules to create a complete VLA system that performs a simple pick-and-place task in simulation.

### Instructions

1.  Set up a Gazebo or Isaac Sim environment with a robot arm, a few distinct objects (e.g., colored blocks), and a "target" location (e.g., a bin or another table).
2.  Use your object detection node (from Exercise 1) to identify the objects and their 3D poses.
3.  Use your natural language parser (from Exercise 2) to interpret a command like "Pick up the [color] block and place it in the [target] area."
4.  Write a central VLA coordinator node (Python ROS 2 node) that:
    -   Subscribes to the `/detected_object_pose` topic.
    -   Receives natural language commands (e.g., via a ROS 2 service or a simple terminal input).
    -   Uses the parsed command and detected object poses to formulate a robot action (e.g., "pick up red block at pose X,Y,Z").
    -   Sends control commands to your robot arm in the simulation (e.g., through a ROS 2 action/service for `move_to_pose`, `grasp`, `release`). This might involve a simple MoveIt! setup or direct joint control.

### Validation

When you issue a command like "Pick up the blue cube and place it in the red bin," your simulated robot arm should:
1.  Visually identify the blue cube.
2.  Move to the blue cube's location.
3.  Perform a grasping motion.
4.  Move to the red bin's location.
5.  Perform a releasing motion.

<details>
<summary>Show Solution</summary>

**Conceptual VLA Coordinator Node (Python ROS 2)**:
This is a high-level conceptual solution. Implementing the full robot control (`move_to_pose`, `grasp`, `release`) would involve advanced topics like inverse kinematics, motion planning, and potentially a `MoveIt!` setup, which are beyond the scope of a single exercise. The focus here is on integrating the vision and language components.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger # Example for NL input
# Import your parse_command function and object detector
from your_llm_parser import parse_command
# from your_object_detector import ObjectPoseDetector # Not directly used here, but its output is

# Define custom interfaces for robot arm control (example)
# from robot_interfaces.srv import PickPlace # Custom service like in 4.3

class VLACoordinator(Node):
    def __init__(self):
        super().__init__('vla_coordinator')
        self.object_poses = {} # Store latest detected object poses by label
        self.object_pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose', # Assuming your object detector publishes this
            self.object_pose_callback,
            10)
        self.nl_command_service = self.create_service(
            Trigger,
            '/send_nl_command',
            self.nl_command_callback)
        
        # Example robot arm control client (replace with your actual robot API)
        self.robot_control_client = self.create_client(PickPlace, 'pick_place_service')
        while not self.robot_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Robot control service not available, waiting...')

        self.get_logger().info("VLA Coordinator Ready.")

    def object_pose_callback(self, msg):
        # Assuming you can extract a label from the PoseStamped (e.g., in a child frame_id)
        # For simplicity, let's assume objects are identified by their 'label' and pose_msg.header.frame_id contains it
        object_label = msg.header.frame_id # This would likely be a custom message for object detection
        self.object_poses[object_label] = msg
        self.get_logger().info(f"Received pose for {object_label} at {msg.pose.position}")

    def nl_command_callback(self, request, response):
        nl_command = request.message # Assuming trigger message contains the NL command
        self.get_logger().info(f"Received NL command: '{nl_command}'")
        parsed_cmd = parse_command(nl_command)
        self.get_logger().info(f"Parsed command: {parsed_cmd}")

        action = parsed_cmd.get("action")
        obj_name = parsed_cmd.get("object")
        location_name = parsed_cmd.get("location")

        if action == "pick up" and obj_name:
            target_object_pose = self.object_poses.get(obj_name)
            if target_object_pose:
                # Call robot control service to pick up
                req = PickPlace.Request()
                req.action_type = PickPlace.Request.PICK
                req.target_object_pose = target_object_pose
                future = self.robot_control_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                if future.result().success:
                    self.get_logger().info(f"Successfully executed pick up: {obj_name}")
                    response.success = True
                    response.message = f"Successfully picked up {obj_name}."
                else:
                    self.get_logger().error(f"Failed to pick up {obj_name}.")
                    response.success = False
                    response.message = f"Failed to pick up {obj_name}."
            else:
                self.get_logger().warn(f"Object '{obj_name}' not detected.")
                response.success = False
                response.message = f"Object '{obj_name}' not detected."
        elif action == "place" and location_name:
            # For simplicity, assume a predefined pose for 'red bin' or 'table'
            target_location_pose = PoseStamped() # Populate with known bin/table pose
            target_location_pose.header.frame_id = location_name # Placeholder
            target_location_pose.pose.position.x = 0.8
            target_location_pose.pose.position.y = 0.2
            target_location_pose.pose.position.z = 0.1
            
            # Call robot control service to place
            req = PickPlace.Request()
            req.action_type = PickPlace.Request.PLACE
            req.target_location_pose = target_location_pose
            future = self.robot_control_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info(f"Successfully executed place at {location_name}.")
                response.success = True
                response.message = f"Successfully placed object at {location_name}."
            else:
                self.get_logger().error(f"Failed to place at {location_name}.")
                response.success = False
                response.message = f"Failed to place object at {location_name}."
        else:
            self.get_logger().warn(f"Action '{action}' with object '{obj_name}' and location '{location_name}' not supported yet.")
            response.success = False
            response.message = "Command not understood or supported."
        return response

def main(args=None):
    rclpy.init(args=args)
    vla_coordinator = VLACoordinator()
    rclpy.spin(vla_coordinator)
    vla_coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Explanation**: This conceptual node orchestrates the VLA process. It subscribes to object detection results and exposes a service to receive natural language commands. Upon receiving a command, it parses it, finds the relevant object's pose, and then calls a robot control service (`PickPlace`) to execute the physical actions. This exercise ties together vision, language, and action components into a functional VLA system, albeit with simplified robot control. You would need to define the `PickPlace.srv` custom service and implement a corresponding service server on your robot control side.

</details>