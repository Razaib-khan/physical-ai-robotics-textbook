---
id: hardware-specs
title: "Hardware Specifications"
sidebar_label: "Hardware Specs"
description: Recommended hardware specifications for robotics development, simulation, and physical robots
keywords:
  - hardware
  - specifications
  - requirements
  - gpu
  - robots
---

# Hardware Specifications

Hardware requirements for robotics development and deployment.

## Development Workstation

### Minimum Specifications (Modules 1-2)

For ROS 2 basics and Gazebo/Unity simulation:

- **CPU**: Intel i5 / AMD Ryzen 5 (4 cores)
- **RAM**: 8GB DDR4
- **GPU**: Integrated graphics (Intel UHD, AMD Radeon)
- **Disk**: 256GB SSD
- **OS**: Ubuntu 22.04 LTS

**Use Cases**: Module 1 (ROS 2), Module 2 (Gazebo basic), coding exercises

### Recommended Specifications (Modules 1-3)

For GPU-accelerated simulation and RL training:

- **CPU**: Intel i7 / AMD Ryzen 7 (8 cores)
- **RAM**: 32GB DDR4
- **GPU**: NVIDIA RTX 3060 (12GB VRAM) or better
- **Disk**: 512GB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

**Use Cases**: All modules, Isaac Sim, Isaac Gym RL training, VLA model inference

### High-Performance Specifications (Module 3-4 + Capstone)

For large-scale RL training and VLA model fine-tuning:

- **CPU**: Intel i9 / AMD Ryzen 9 (12+ cores)
- **RAM**: 64GB DDR5
- **GPU**: NVIDIA RTX 4080 / RTX 4090 (16GB+ VRAM)
- **Disk**: 1TB+ NVMe SSD
- **OS**: Ubuntu 22.04 LTS

**Use Cases**: Parallel RL training, VLA fine-tuning, multi-robot simulation

## GPU Requirements by Module

| Module | Task | Minimum GPU | Recommended GPU | VRAM |
|--------|------|-------------|-----------------|------|
| 1 | ROS 2 | None (CPU) | None | N/A |
| 2 | Gazebo | Integrated | GTX 1650 | 4GB |
| 2 | Unity | GTX 1060 | RTX 2060 | 6GB |
| 3 | Isaac Gym | GTX 1060 | RTX 3060 | 12GB |
| 3 | Isaac Sim | RTX 2060 | RTX 3080 | 12GB |
| 4 | VLA Inference | GTX 1660 | RTX 3060 | 8GB |
| Capstone | Full Integration | RTX 3060 | RTX 4070 | 12GB |

:::tip No GPU?
- **Modules 1-2**: Fully functional on CPU
- **Module 3**: Use cloud compute (AWS EC2 G5, Google Cloud GPU instances)
- **Module 4**: Use quantized models or cloud APIs (OpenAI, Anthropic)
:::

## Cloud Computing Alternatives

### AWS EC2 Instances

| Instance Type | GPU | vCPU | RAM | Use Case | Cost (approx) |
|---------------|-----|------|-----|----------|---------------|
| g4dn.xlarge | T4 (16GB) | 4 | 16GB | Module 3 basic | $0.50/hr |
| g5.xlarge | A10G (24GB) | 4 | 16GB | Module 3 + 4 | $1.00/hr |
| g5.2xlarge | A10G (24GB) | 8 | 32GB | Capstone | $1.20/hr |

### Google Cloud Platform

| Instance Type | GPU | vCPU | RAM | Use Case | Cost (approx) |
|---------------|-----|------|-----|----------|---------------|
| n1-standard-4 + T4 | T4 (16GB) | 4 | 15GB | Module 3 basic | $0.45/hr |
| n1-standard-8 + V100 | V100 (16GB) | 8 | 30GB | Module 3 + 4 | $2.50/hr |

### NVIDIA NGC Containers

Pre-configured Docker containers with Isaac Sim, PyTorch, CUDA:
- Free tier available for learning
- [NVIDIA NGC Catalog](https://catalog.ngc.nvidia.com/)

## Physical Robot Hardware (Optional)

### Mobile Robot Platforms

For students wanting to deploy to real hardware:

#### TurtleBot 4 (Entry-Level)

- **Platform**: Create 3 Base + Raspberry Pi 4
- **Sensors**: LiDAR, IMU, Camera
- **ROS 2**: Native support
- **Cost**: ~$1,200
- **Use Cases**: Module 1-2 deployment, navigation

#### Unitree Go1 (Quadruped)

- **Platform**: Quadruped robot
- **Sensors**: Cameras, IMU, foot force sensors
- **ROS 2**: Community support
- **Cost**: ~$2,700
- **Use Cases**: Locomotion RL deployment

### Manipulator Arms

#### WidowX 250 Robot Arm

- **DOF**: 6 (5 arm + 1 gripper)
- **Reach**: 650mm
- **Payload**: 250g
- **ROS 2**: Native support
- **Cost**: ~$1,400
- **Use Cases**: Manipulation RL deployment

#### Franka Emika Panda (Research-Grade)

- **DOF**: 7 + 2 finger gripper
- **Reach**: 855mm
- **Payload**: 3kg
- **ROS 2**: Official support
- **Cost**: ~$25,000
- **Use Cases**: Advanced manipulation research

### Humanoid Robots

#### Unitree H1 (Humanoid)

- **Height**: 180cm
- **Weight**: 47kg
- **DOF**: 25+ joints
- **Sensors**: Cameras, LiDAR, IMU
- **Cost**: Contact manufacturer
- **Use Cases**: Capstone deployment (advanced)

:::warning Physical Robots Not Required
All textbook content can be completed in simulation. Physical hardware is optional for students with advanced interests and budgets.
:::

## Compute Budgets

### Cost Estimates for Cloud Training

**Module 3 RL Training** (Isaac Gym):
- Training time: 10-20 hours per policy
- Instance: AWS g5.xlarge ($1.00/hr)
- **Estimated cost per module**: $20-40

**Module 4 VLA Fine-Tuning**:
- Training time: 20-40 hours
- Instance: AWS g5.2xlarge ($1.20/hr)
- **Estimated cost per module**: $25-50

**Capstone Project**:
- Total compute time: 50-100 hours
- Mixed instances (g4dn + g5)
- **Estimated cost**: $50-100

**Total Cloud Cost (Modules 3-4 + Capstone)**: $100-200

## Storage Requirements

| Component | Size | Notes |
|-----------|------|-------|
| Ubuntu 22.04 | 10GB | Base OS |
| ROS 2 Humble | 5GB | Full desktop install |
| Gazebo + models | 5GB | Simulator and assets |
| Unity Editor | 10GB | Game engine |
| Isaac Sim | 30GB | Omniverse + Isaac |
| PyTorch + CUDA | 5GB | Deep learning libraries |
| Pre-trained models | 20GB | VLA, vision models |
| Datasets | 10-50GB | Optional training data |
| **Total** | **95-135GB** | Full stack |

:::tip Storage Management
Use external SSD or cloud storage for datasets. Only install Isaac Sim if pursuing Module 3.
:::

## Network Requirements

- **Internet Speed**: 10 Mbps minimum for package downloads
- **Bandwidth**: 1GB+ for initial ROS 2, Gazebo, Unity installations
- **Cloud Training**: Upload datasets (1-10GB) before training

## Peripheral Hardware

### Recommended Accessories

- **Monitor**: 1080p minimum, 1440p+ recommended for Unity/Isaac Sim
- **Input**: Mouse + keyboard (required for simulation)
- **Camera**: Webcam if testing vision algorithms with real data
- **Microphone**: For voice command experiments (Module 4)

---

**Next**: [Software Setup Guide](./software-setup.md) for installation instructions

[Return to Homepage](/)
