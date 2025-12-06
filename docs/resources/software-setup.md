---
id: software-setup
title: "Software Setup Guide"
sidebar_label: "Software Setup"
description: Step-by-step installation guide for ROS 2, simulators, and development tools
keywords:
  - installation
  - setup
  - ros2
  - gazebo
  - unity
  - isaac sim
---

# Software Setup Guide

Complete installation instructions for all textbook modules.

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **User Privileges**: sudo access for package installation
- **Internet Connection**: Stable connection for downloads
- **Disk Space**: See [Hardware Specifications](./hardware-specs.md) for requirements

:::warning Windows/macOS Users
ROS 2 works best on Ubuntu Linux. Windows users should use WSL2 or dual-boot. macOS support is experimental. We strongly recommend native Ubuntu 22.04.
:::

## Module 1: ROS 2 Setup

### 1.1 Install ROS 2 Humble

**Set up sources:**

```bash title="Add ROS 2 apt repository"
# Ensure UTF-8 encoding
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Install ROS 2 packages:**

```bash title="Install ROS 2 Humble Desktop"
# Update package index
sudo apt update

# Install ROS 2 Humble Desktop (full installation)
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

**Environment setup:**

```bash title="Add to ~/.bashrc for automatic sourcing"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify installation:**

```bash title="Check ROS 2 version"
ros2 --version
# Expected: ros2 cli version: 0.18.5 or later
```

### 1.2 Install Python Dependencies

```bash title="Install Python packages for ROS 2 development"
sudo apt install python3-pip
pip3 install --upgrade pip

# ROS 2 Python libraries
pip3 install transforms3d pyyaml

# Development tools
pip3 install pytest black flake8
```

---

## Module 2: Simulation Setup

### 2.1 Install Gazebo Harmonic

**Add Gazebo repository:**

```bash title="Setup Gazebo apt repository"
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

**Install Gazebo:**

```bash title="Install Gazebo Harmonic"
sudo apt update
sudo apt install gz-harmonic

# ROS 2 - Gazebo bridge
sudo apt install ros-humble-ros-gz
```

**Verify installation:**

```bash title="Check Gazebo version"
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

### 2.2 Install Unity (Optional)

**Download Unity Hub:**

```bash title="Install Unity Hub for Linux"
# Download Unity Hub AppImage
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

**Install Unity Editor via Unity Hub:**

1. Open Unity Hub
2. Go to Installs → Add
3. Select Unity 2022.3 LTS (recommended)
4. Add modules: Linux Build Support

**Install ROS-TCP connector:**

Follow instructions at [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

---

## Module 3: NVIDIA Isaac Setup

:::warning GPU Required
Isaac Sim and Isaac Gym require NVIDIA GPU with CUDA support. Minimum: GTX 1060. Recommended: RTX 3060 or better.
:::

### 3.1 Install CUDA Toolkit

**Check GPU compatibility:**

```bash title="Verify NVIDIA GPU"
lspci | grep -i nvidia
# Should list NVIDIA GPU
```

**Install CUDA 11.8:**

```bash title="Install CUDA Toolkit"
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt update
sudo apt install cuda-11-8

# Add to PATH
echo 'export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc
```

**Verify CUDA:**

```bash title="Check CUDA version"
nvcc --version
# Expected: release 11.8
```

### 3.2 Install PyTorch with CUDA

```bash title="Install PyTorch 2.0 with CUDA 11.8"
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

**Verify GPU access:**

```python title="Test PyTorch CUDA"
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0)}")
```

### 3.3 Install Isaac Sim

**Download Omniverse Launcher:**

1. Go to [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Download Omniverse Launcher for Linux
3. Install and run Omniverse Launcher

**Install Isaac Sim via Launcher:**

1. Open Omniverse Launcher
2. Go to Exchange tab
3. Search for "Isaac Sim"
4. Click Install (version 2023.1.0 or later)
5. Wait for installation (~30GB download)

**Verify Isaac Sim:**

Launch Isaac Sim from Omniverse Launcher to verify installation.

### 3.4 Install Isaac Gym (Alternative)

**Clone Isaac Gym repository:**

```bash title="Install Isaac Gym Preview 4"
# Download from NVIDIA (requires developer account)
# Extract to ~/IsaacGym

cd ~/IsaacGym/python
pip3 install -e .
```

**Test installation:**

```bash title="Run Isaac Gym example"
cd ~/IsaacGym/python/examples
python3 1080_balls_of_solitude.py
# Should open physics simulation window
```

---

## Module 4: VLA and AI Setup

### 4.1 Install Hugging Face Transformers

```bash title="Install Transformers and dependencies"
pip3 install transformers[torch]
pip3 install accelerate safetensors
```

### 4.2 Install Computer Vision Libraries

```bash title="Install OpenCV and vision tools"
pip3 install opencv-python opencv-contrib-python
pip3 install pillow matplotlib
```

### 4.3 Install Model Serving Tools (Optional)

```bash title="Install ONNX Runtime for optimized inference"
pip3 install onnxruntime-gpu  # GPU version
# OR
pip3 install onnxruntime  # CPU version
```

---

## Development Tools

### VS Code with ROS 2 Extensions

**Install VS Code:**

```bash title="Install Visual Studio Code"
sudo snap install code --classic
```

**Recommended extensions:**

- ROS (Microsoft)
- Python (Microsoft)
- C/C++ (Microsoft)
- CMake Tools

### Git Configuration

```bash title="Setup Git for version control"
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

---

## Verification Checklist

After installation, verify all components:

```bash title="Run verification script"
# ROS 2
ros2 doctor

# Gazebo
gz sim --version

# Python packages
python3 -c "import rclpy, torch, transformers; print('All imports successful')"

# CUDA (if GPU)
nvidia-smi
```

:::tip Troubleshooting
If verification fails:
1. Verify all dependencies installed
2. Restart terminal to reload environment variables
3. Consult module-specific documentation
4. Check ROS 2 and NVIDIA forums for common issues
:::

---

## Next Steps

- **Module 1**: Continue to [1.1 ROS 2 Introduction](../module-1-ros2/1-1-ros2-intro.md)
- **Hardware**: See [Hardware Specifications](./hardware-specs.md) for compute requirements

[Return to Homepage](/)
