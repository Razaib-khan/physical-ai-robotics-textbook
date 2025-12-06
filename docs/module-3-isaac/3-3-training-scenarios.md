---
id: 3-3-training-scenarios
title: "3.3: Training Scenarios and Sim-to-Real"
sidebar_label: "Training Scenarios"
sidebar_position: 3
description: "Learn how to design effective training scenarios in Isaac Sim and bridge the 'reality gap' using domain randomization for successful sim-to-real transfer."
keywords: ['sim-to-real', 'domain randomization', 'isaac sim', 'training', 'robotics']
difficulty: advanced
estimated_time: "60-75 minutes"
learning_outcomes:
  - "Design and implement custom training scenarios for RL."
  - "Understand the 'reality gap' and its challenges."
  - "Apply domain randomization to improve sim-to-real transfer."
prerequisites:
  - "3-2-reinforcement-learning"
hardware_required: true
---

## From Simulation to Reality: Bridging the Gap

After training a robot in simulation, the ultimate goal is to transfer that learned policy to a physical robot. However, there is often a "reality gap" where a policy that works perfectly in simulation fails on the real hardware. This is because no simulation is a perfect one-to-one replica of the real world.

This chapter explores how to design effective training scenarios and use a technique called **domain randomization** to bridge this reality gap.

### Designing Training Scenarios

A good training scenario should be more than just a single, static setup. To ensure the learned policy is robust and can generalize, you should introduce variability into the training process.

In Isaac Gym, you can reset your environments to new configurations at the end of each episode. This is where you can programmatically change the scenario.

```python title="scenario_reset.py"
# Inside your Isaac Gym environment class

def _reset_envs(self, env_ids):
    # Get the number of environments to reset
    num_resets = len(env_ids)

    # Randomize the goal position for each environment
    self.goal_pos[env_ids, 0] = torch.rand(num_resets) * 2.0 - 1.0
    self.goal_pos[env_ids, 1] = torch.rand(num_resets) * 2.0 - 1.0
    self.goal_pos[env_ids, 2] = torch.rand(num_resets) * 0.5 + 0.5

    # ... (reset the robot's joint positions and velocities)
```
In this example, every time an environment is reset, the goal is moved to a new random position. By training across thousands of these randomized scenarios in parallel, the robot learns a much more general reaching policy that isn't tied to a single target location.

### The Reality Gap and Domain Randomization

Domain Randomization (DR) is a powerful technique to bridge the sim-to-real gap. The core idea is that if you expose the robot to a wide enough range of variations in simulation, the real world will just look like another one of those variations.

Instead of trying to make your simulation perfectly match reality, you make it *so varied* that the real world is just another point in the distribution.

You can randomize:
-   **Visuals:** Lighting conditions, textures, camera position.
-   **Physics:** Object masses, friction coefficients, motor torques.
-   **Dynamics:** Delays in sensor data or motor commands.

Here's how you might implement domain randomization for physics properties in Isaac Gym:

```python title="domain_randomization.py"
# During environment setup
for env in self.envs:
    # Get the handle to the robot actor
    robot_handle = gym.get_actor_handle(env, 0)

    # Get the robot's rigid body properties
    props = gym.get_actor_rigid_body_properties(env, robot_handle)

    # For each rigid body in the robot
    for i in range(len(props)):
        # Randomize the mass
        props[i].mass *= (1.0 + (torch.rand(1) * 0.4 - 0.2)) # +/- 20%

    # Apply the new, randomized properties
    gym.set_actor_rigid_body_properties(env, robot_handle, props)
```
This script iterates through all the rigid bodies of the robot in each environment and randomizes their mass by up to 20%. By doing this for friction, lighting, and other parameters, you force the RL algorithm to learn a policy that is robust to these variations, and therefore more likely to succeed in the real world.

:::warning Hardware Note
Training with domain randomization can be computationally expensive, as it often requires more complex scenes and physics calculations. A powerful GPU with ample VRAM (e.g., NVIDIA RTX 3080 or better) is highly recommended for effective domain randomization, especially with a large number of parallel environments. Training times can range from a few hours to a few days depending on the task complexity.
:::

With a well-designed training scenario and effective use of domain randomization, the policies you train in Isaac Sim have a much higher chance of achieving successful sim-to-real transfer, bringing your AI-driven robot one step closer to reality.
