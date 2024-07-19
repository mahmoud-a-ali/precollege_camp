# Description

Introduction to ROS and the Gazebo Robot Simulator for students. 
We will start with a 10-minute introduction, and then the students will be running a gazebo simulator and commanding a turtlebot3 robot to do different types of motion.

## Learning Outcomes
Learn how to:
- Run Gazebo simulator with turtlebot3
- Visualize sensor data using rviz2
- Control a mobile robot in a simulator 
- Detect obstacles using lidar.


### First tutorial: run the simulator, visualize data, and control robot using keyboard:
1. Open a terminal window by pressing `Ctrl+Alt+t`
2. Clone this repository into your PC
   ```bash
   git clone https://github.com/mahmoud-a-ali/precollege_camp.git
   ```
3. Go to the cloned directory
   ```bash
   cd precollege_camp
   ```
5. Make the bash script files executables
   ```bash
   chmod +x create_turtlebot_sim.sh source_turtlebot_sim.sh
   ```
6. Run `create_turtlebot_sim.sh` to create the simulation workspace `turtlebot_sim`
   ```bash
   source create_turtlebot_sim.sh
   ```
   - This script does the following: create a `turtlebot_sim` workspace, download required packages, compile the workspace, source the environment, and run the empty `gazebo` world with a `burger` turtlebots.
  
7. Open new terminal tab by pressing `Ctrl+Shift+t`, we will call this tab the `control_tab`.
8. Source the simulation environment
   ```bash
   source source_turtlebot_sim.sh
   ```
   - This is equivalent to
     ```bash
     source /opt/ros/humble/setup.bash
     export TURTLEBOT3_MODEL=burger
     cd turtlebot_sim
     source install/setup.bash
     ```
9. Run the `teleop_key` node to run the
  ```bash
  ros2 run turtlebot3_teleop 
  ```
10. Open a new terminal tab by pressing `Ctrl+Shift+t` and in it run `rqt_graph` to visualize robot nodes and topics (`rqt_tab`).

11. Open a new terminal tab by pressing `Ctrl+Shift+t` and in it run `rviz2` to visualize the robot, the robot's path, and the laser scanner data(`rviz_tab`).

12. In the `control_tab`, where `teleop_key` is running you can control the robot by pressing `w`, `a`, `d`, `x`, and `s` letters.
13. Check robot motion and robot path in `rviz2`.


Place different obstacles in the environment
Use the mouse to drop few boxes, cylinders, or spheres as obstacles

Detect Obstacle position using Lidar
