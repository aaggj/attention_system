
# Attention System

Welcome to the Attention System repository. This system is designed to manage a list of attention points, moving the robot's head from one point to another while keeping a record of how often each point has been visited. This ensures a dynamic interaction environment, focusing on areas of interest based on predefined criteria or external inputs.

## Prerequisites

- **Git:** Required for cloning the repository. If not installed, download it from [https://git-scm.com/](https://git-scm.com/).
- **ROS2 Jazzy:** Ensure ROS2 Jazzy is installed on your system to run the attention system. For installation details, refer to the [ROS2  Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation.html).

## Installation

### Step 1: Prepare the Workspace

Create a new workspace or use an existing one. To create a new workspace, run the following commands in a terminal:

```bash
mkdir -p ~/attention_system_ws/src
cd ~/attention_system_ws/src
```

### Step 2: Clone the Repository

Clone the attention system repository with the following command:

```bash
git clone https://github.com/aaggj/attention_system
```

### Step 3: Build

Before running the system, ensure to compile the workspace with ROS2. Navigate to your workspace's root directory and execute:

```bash
cd ~/attention_system_ws
colcon build --symlink-install 
source install/setup.bash
```

## Usage

To run the attention system, use the following ROS2 command:

```bash
ros2 run attention_system attention_server
```

This command will start a lifecycle cascade node that will be configured and activated automatically, listening on the `/attention/attention_points` topic. This topic expects messages of type `attention_system_msgs/msg/AttentionPoints`, which are used to manage the attention points within the system.

## `AttentionPoints` Message

The `AttentionPoints` message is designed to contain specific information about the attention points. Be sure to review the message definition in the repository to understand the structure and required fields.

## Contributing

If you are interested in contributing to the project, we encourage you to do so. You can start by reviewing the open issues in the repository, discussing new features, or reporting bugs. For major contributions, please make sure to open a new issue to discuss your ideas before making a pull request.


<!-- [![Build Status](https://travis-ci.com/aaggj/attention_system.svg?branch=humble)](https://travis-ci.com/aaggj/attention_system) -->
