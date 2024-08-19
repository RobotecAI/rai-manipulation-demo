# Robotic Arm Manipulation Demo

This project demonstrates the use of [RAI](https://github.com/RobotecAI/rai) framework interfacing with a robotic arm in a simulation. In particular, it shows the communication between Generative AI models and the [Panda Franka](https://www.franka.de/) manipulator, which is a popular choice in AI and manipulation research in the robotic community. 

The simulation uses [O3DE](https://www.o3de.org/) game engine paired with [ROS 2 Gem](https://docs.o3de.org/docs/user-guide/interactivity/robotics/).

> **_NOTE:_**  This repository is not yet functional, as the demo is undergoing an internal review and development. The code will be fully usable and documented before [ROSCon 2024](https://roscon.ros.org/2024/).

## Demo description

The demonstration features a simulation of a robotic arm equipped with a gripper, which interacts with a multitude of objects that can be grasped, sorted, or manipulated based on instructions issued in the natural language. The RAI framework captures contextual information such as camera images and available command sets, subsequently translating these commands into ROS 2 messages to execute the arm's movements resolved by the Generative AI models.

## Prerequisites

1. Download and register o3de
2. Download o3de-extras
3. Register o3de-extras gems

## Project setup

1. Clone the repository.

    ```bash
    git clone https://github.com/RobotecAI/rai-manipulation-demo
    cd rai-manipulation-demo
    ```

2. Build the O3DE project.

    ```bash
    cd Project
    cmake -B build/linux -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
    cmake --build build/linux --config profile --target RAIManipulationDemo.Assets RAIManipulationDemo.GameLauncher
    ```

3. Build the ROS2 packages.

    ```bash
    cd ros2_ws
    colcon build
    ```

## Running the demo

1. Run the editor and load the Robotic Manipulation level.
2. Press play.
3. Run the MoveIt nodes.

    ```bash
    cd Project/Examples
    ros2 launch panda_moveit_config_demo.launch.py
    ```

4. In another terminal, run the OpenVLA inference node.
    - Source the ROS 2 workspace.

    ```bash
    source ros2_ws/install/setup.bash
    ```

    - Run the OpenVLA inference node

    ```bash
    ros2 run run_vla vla
    ```

    - In yet another terminal, run the interface node

    ```bash
    ros2 run robotic_manipulation robotic_manipulation 1 100 vla
    ```
