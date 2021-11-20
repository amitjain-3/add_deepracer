# Set up the message interface package

For further understanding check: https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html

## Creating a custom msg package
- [x] Add a `ObjVelocityMsg.msg` file
- [x] Define its content (two `float32` for horizontal and vertical velocity)
- [x] Setting this change in the `CMakeLists.txt` file
- [] Setting the `package.xml` file
- [] Build the interface package with following command line:
    - [] `cd ~/deepracer_ws/add_deepracer/ && colcon build --packages-select deepracer_interfaces_pkg`
    - [] Follow the tutorial! (https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)