# Robotic Arm Manipulation Demo

This project demonstrates the use of [RAI](https://github.com/RobotecAI/rai) framework interfacing with a robotic arm in a simulation. In particular, it shows the communication between Generative AI models and the [Panda Franka](https://www.franka.de/) manipulator, which is a popular choice in AI and manipulation research in the robotic community. 

The simulation uses [O3DE](https://www.o3de.org/) game engine paired with [ROS 2 Gem](https://docs.o3de.org/docs/user-guide/interactivity/robotics/).

![Screenshot](docs/images/manipulation.png)

## Demo description

The demonstration features a simulation of a robotic arm equipped with a gripper, which interacts with a multitude of objects that can be grasped, sorted, or manipulated based on instructions issued in the natural language. The RAI framework captures contextual information such as camera images and available command sets, subsequently translating these commands into ROS 2 messages to execute the arm's movements resolved by the Generative AI models.

# Building the demo

## Prerequisites
- Install [ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Install all ROS2 packages [required by O3DE](https://docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/)
- Install the MoveIt package:
```bash
sudo apt install "ros-jazzy-moveit*"
```
- Install `git-lfs` for pulling binary files:
```bash
sudo apt-get install git-lfs
```
- Clone and build [RAI](https://github.com/RobotecAI/rai?tab=readme-ov-file#setup) (follow the instructions there).

## O3DE Setup

Assuming a common base folder `$DEMO_BASE` (absolute path):
1. Clone O3DE and register the engine:
```bash
cd $DEMO_BASE
git clone https://github.com/o3de/o3de.git -b main
cd $DEMO_BASE/o3de
git lfs install
git lfs pull
python/get_python.sh
scripts/o3de.sh register --this-engine
```

2. Clone o3de-extras and register the gems
```bash
cd $DEMO_BASE
git clone https://github.com/o3de/o3de-extras.git -b main
$DEMO_BASE/o3de/scripts/o3de.sh register -agp $DEMO_BASE/o3de-extras/Gems
```

## Project setup

1. Clone the repository and build the O3DE project.

```bash
cd $DEMO_BASE
git clone https://github.com/RobotecAI/rai-manipulation-demo
cd $DEMO_BASE/rai-manipulation-demo/Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile --target RAIManipulationDemo.Assets RAIManipulationDemo.GameLauncher Editor
```

2. Build the ROS2 workspace.

> ****NOTE****: This step requires to have [RAI](https://github.com/RobotecAI/rai) packages built and sourced! Make sure to run `source install/setup.sh` in the RAI directory before continuing.

```bash
cd $DEMO_BASE/rai-manipulation-demo/ros2_ws
colcon build
```

## Running the demo

1. Run the Game Launcher.
```bash
cd $DEMO_BASE/rai-manipulation-demo/Project
./build/linux/bin/profile/RAIManipulationDemo.GameLauncher
```

2. Launch MoveIt:

```bash
cd $DEMO_BASE/rai-manipulation-demo/Project/Examples
ros2 launch panda_moveit_config_demo.launch.py
```

3. Run the manipulation interface node.

```bash
cd $DEMO_BASE/rai-manipulation-demo
source ros2_ws/install/setup.bash
ros2 run robotic_manipulation robotic_manipulation
```

The package exposes one service:

- `/manipulator_move_to` ([rai_interfaces/srv/ManipulatorMoveTo](https://github.com/RobotecAI/rai/blob/development/src/rai_interfaces/srv/ManipulatorMoveTo.srv)) -
Moves the manipulator to the specified pose, with two intermediate poses above the current position and the target pose, allowing safe manipulation of the objects on the scene. Allows specifying gripper states before and after the movement.
