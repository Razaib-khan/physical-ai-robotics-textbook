---
id: module-3-exercises
title: "Module 3: Exercises"
sidebar_label: "Exercises"
description: "Apply your knowledge of NVIDIA Isaac Sim and Reinforcement Learning with these hands-on exercises."
keywords: ['exercises', 'isaac sim', 'reinforcement learning', 'rl', 'robotics']
difficulty: 'mixed'
estimated_time: "3-4 hours"
---

## Exercise 1: Your First Isaac Sim Scene

**Difficulty**: Beginner
**Estimated Time**: 45-60 minutes
**Objective**: Write a Python script to create a basic Isaac Sim scene with a robot and a few objects.

### Instructions

1.  Create a new Python script.
2.  Import the necessary Isaac Sim libraries (`SimulationApp`, `World`, `Robot`).
3.  Initialize the `SimulationApp`.
4.  Create a `World` instance.
5.  Add a default ground plane to the world.
6.  Add a robot to the world from a URDF file. You can use a simple URDF from Module 1 or one of the examples provided with Isaac Sim.
7.  Add at least two simple objects (e.g., a cube and a sphere) to the scene at different positions. You can use `world.scene.add(Cube(...))` and `world.scene.add(Sphere(...))`.
8.  Run the simulation loop.

### Validation

When you run the script, an Isaac Sim window should open showing your robot, a cube, and a sphere on a ground plane.

<details>
<summary>Show Solution</summary>

**Python Script**:
```python
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import Cube, Sphere
from omni.isaac.core.robots import Robot

world = World()
world.scene.add_default_ground_plane()

# Add a robot
robot_asset_path = "path/to/your/robot.urdf"
world.scene.add(
    Robot(
        prim_path="/world/robot",
        name="my_robot",
        position=[0, 0, 0.5],
        urdf_path=robot_asset_path
    )
)

# Add a cube
world.scene.add(
    Cube(
        prim_path="/world/my_cube",
        name="my_cube",
        position=[2, 2, 0.5],
        scale=[0.5, 0.5, 0.5],
        color=[1.0, 0, 0],
    )
)

# Add a sphere
world.scene.add(
    Sphere(
        prim_path="/world/my_sphere",
        name="my_sphere",
        position=[-2, 2, 0.5],
        radius=0.5,
        color=[0, 0, 1.0],
    )
)

world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

**Explanation**: This script follows the basic structure for creating any Isaac Sim application. We initialize the simulation, create a world, and then add objects to it. The `world.scene.add()` method is a versatile way to add robots, simple shapes, and other objects to the simulation.

</details>

---

## Exercise 2: Simple Navigation RL Scenario

**Difficulty**: Intermediate
**Estimated Time**: 60-75 minutes
**Objective**: Set up a basic reinforcement learning environment where a robot learns to navigate to a target.

### Instructions

1.  Start with one of the Isaac Gym example environments (e.g., the Cartpole example).
2.  Modify the environment to use a simple differential drive robot instead of the cartpole.
3.  The **observation space** should include the robot's distance and angle to the goal.
4.  The **action space** should be the linear and angular velocity of the robot.
5.  The **reward function** should be based on the negative distance to the goal.
6.  At the end of each episode, reset the robot to a random position and the goal to a new random position.
7.  Run the training and observe the `reward` values in the console output.

### Validation

As the training progresses, you should see the average reward increase over time, indicating that the robot is getting better at reaching the goal. When you visualize the simulation, you should see the robots moving more directly towards their targets in later stages of training.

<details>
<summary>Show Solution</summary>

**Conceptual Solution Snippets**:

**Observation Calculation**:
```python
# Part of the simulation step
goal_pos = self.goal_pos[env_ids]
robot_pos = self.robot_pos[env_ids]

# Vector to goal
vec_to_goal = goal_pos - robot_pos
dist_to_goal = torch.norm(vec_to_goal, p=2, dim=-1)
angle_to_goal = torch.atan2(vec_to_goal[..., 1], vec_to_goal[..., 0])

# Populate the observation buffer
self.obs_buf[env_ids, 0] = dist_to_goal
self.obs_buf[env_ids, 1] = angle_to_goal
```

**Reward Calculation**:
```python
# Part of the simulation step
dist_to_goal = self.obs_buf[env_ids, 0]
self.rew_buf[env_ids] = -dist_to_goal
```

**Episode Reset**:
```python
# Part of the reset function
# Random robot and goal positions
self.robot_pos[env_ids, 0] = torch_rand_float(-5, 5, (num_resets, 1), device=self.device).squeeze()
self.goal_pos[env_ids, 0] = torch_rand_float(-5, 5, (num_resets, 1), device=self.device).squeeze()
# ... (similar for y coordinate)
```

**Explanation**: This is a non-trivial exercise that requires modifying one of the existing Isaac Gym examples. The key is to correctly define the observation space, action space, and reward function for your specific task. The snippets above show the core logic for a navigation task. The RL algorithm (e.g., PPO) will then use these components to learn a successful policy.

</details>

---

## Exercise 3: Custom Reward Function

**Difficulty**: Advanced
**Estimated Time**: 75-90 minutes
**Objective**: Implement a more sophisticated reward function for a manipulation task (e.g., reaching).

### Instructions

1.  Start with an Isaac Gym example for a robot arm, like the Franka reach task.
2.  Identify the existing reward function in the environment's Python script.
3.  Modify the reward function to include the following components:
    -   A dense reward based on the negative distance between the end-effector and the target.
    -   A large bonus reward for successfully reaching the target (i.e., distance < a small threshold).
    -   A small penalty for large joint velocities to encourage smooth movements.
    -   A penalty for being near a joint limit.
4.  Train the policy with your new reward function.
5.  Compare the performance of the policy trained with your custom reward function to the one trained with the original reward function.

### Validation

The policy trained with your custom reward function should exhibit more desirable behavior, such as smoother movements and avoiding joint limits, while still successfully completing the task. You might see a more stable reward curve during training.

<details>
<summary>Show Solution</summary>

**Reward Function Snippet**:
```python
# Inside the simulation step

# Distance-based reward
dist_to_target = torch.norm(target_pos - end_effector_pos, dim=-1)
dist_reward = -dist_to_target

# Success bonus
success_bonus = torch.where(dist_to_target < 0.02, torch.ones_like(dist_to_target) * 10.0, torch.zeros_like(dist_to_target))

# Joint velocity penalty
joint_vels = self.dof_vel[:, :]
action_penalty = torch.sum(torch.square(joint_vels), dim=1) * -0.001

# Joint limit penalty
joint_limits_penalty = ... # (Requires checking joint positions against limits)

# Total reward
self.rew_buf[:] = dist_reward + success_bonus + action_penalty # + joint_limits_penalty
```

**Explanation**: Reward shaping is a crucial aspect of RL. By combining different reward components, you can guide the learning process towards a policy that not only solves the task but does so in a way that meets your specific criteria (e.g., efficiency, safety, smoothness). This exercise demonstrates how to translate high-level goals into a mathematical reward function.

</details>