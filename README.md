# Robot-Dog
# Spot Mini Workspace Setup

This guide shows the steps to set up a ROS 2 workspace for the Unitree B2 robot, including building the workspace, launching the robot model in Mujoco and RViz, and making the robot stand.

## 1. Create and Build the ROS 2 Workspace

Follow these steps to create and build your ROS 2 workspace:

1.  **Create the workspace directory:**

    ```bash
    mkdir -p bostondynamics_spot_ws/src
    cd bostondynamics_spot_ws/src
    ```

2.  **Clone the robot's repository:**

    ```bash
    git clone https://github.com/krantiprakash/Robot-Dog/tree/main/bostondynamics_spot_ws
    ```



3.  **Navigate to the workspace root and build:**

    ```bash
    cd ..
    colcon build && source install/setup.bash
    ```

## 2. Launch the Robot Model in Mujoco and RViz with ROS 2 Effort Controller

To visualize and control the Unitree B2 robot model, launch the following ROS 2 launch file:

```bash
ros2 launch spot_description spot_effort.launch.py
 ```

## 3. Make the Robot Stand
```bash
ros2 run my_package my_node
```


### [Demonstration Video](https://youtu.be/bBCZpLf8pjU)

### [Documentation File](https://drive.google.com/file/d/1f-2_3iCpwe2D4ktlGrKjPAVEsN041Xwp/view?usp=sharing) 
