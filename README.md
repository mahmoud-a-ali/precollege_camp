# Description

Introduction to ROS2 and the Gazebo Robot Simulator for students. 
We will start with a 10-minute introduction to ROS2 packages, nodes, and topics. Then, the students will be running a gazebo simulator and commanding a turtlebot3 robot to do different types of motion.

## Learning Outcomes
Learn how to:
- Run Gazebo simulator with turtlebot3
- Visualize sensor data using rviz2
- Control a mobile robot in a simulator 
- Detect obstacles using lidar.


### First tutorial: run the simulator, check ros2 nodes and topics visualize data, and control robot using keyboard:
1. Open a terminal window by pressing `Ctrl+Alt+t`
2. remove previous folder
    ```bash
   rm -rf precollege_camp
   ```
    ```bash
   rm -rf turtlebot_sim2
   ```
4. Clone this repository into your PC
   ```bash
   git clone https://github.com/mahmoud-a-ali/precollege_camp.git
   ```
5. Go to the cloned directory
   ```bash
   cd precollege_camp
   ```
6. Make the bash script files executables
   ```bash
   chmod +x create_turtlebot_sim.sh source_turtlebot_sim.sh
   ```
7. Run `create_turtlebot_sim.sh` to create the simulation workspace `turtlebot_sim`
   ```bash
   source create_turtlebot_sim.sh
   ```
   - This script does the following: create a `turtlebot_sim` workspace, download required packages, compile the workspace, source the environment, and run the empty `gazebo` world with a `burger` turtlebots.
   - If the robot doesn't show up, close the script using `ctrl+c`, and then run the simulator using:
     ```bash
     ros2 launch turtlebot3_gazebo empty_world.launch.py
     ```
  
8. Open a new terminal tab by pressing `Ctrl+Shift+t`, source the workspace, and run `rqt_graph` to visualize robot nodes and topics
 ```bash
 cd
 cd precollege_camp
 source source_turtlebot_sim.sh
 rqt_graph
 ```
7. Open a new terminal tab by pressing `Ctrl+Shift+t`, source the workspace, and run `rviz2` to visualize the robot, the robot's path, and the laser scanner data
  ```bash
  cd
  cd precollege_camp
  source source_turtlebot_sim.sh
  rviz2
  ```
### Second tutorial: control the robot using the keyboard
1. Open a new terminal tab by pressing `Ctrl+Shift+t`, we will call this tab the `keyboard_tab`.
2. Source the simulation environment
   ```bash
   cd
   cd precollege_camp
   source source_turtlebot_sim.sh
   ```
   - This is equivalent to: "Don't Run the following command!"
     ```bash
     source /opt/ros/humble/setup.bash
     export TURTLEBOT3_MODEL=burger
     cd turtlebot_sim
     source install/setup.bash
     ```
9. Run the `teleop_key` node to run the
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard 
  ```
14. In the `keyboard_tab`, where `teleop_key` is running you can control the robot by pressing `w`, `a`, `d`, `x`, and `s` letters.
15. Check the robot's motion and its path in `rviz2`.
16. Use `s` to stop the robot before starting the next tutorial.
    
### Third tutorial: command the robot to go to a specific XY position
1. Open a new terminal tab by pressing `Ctrl+Shift+t`, source the workspace, and run `position_control` to command the robot to go to a specific XY position (`velocity_tab`)
   ```bash
   cd
   cd precollege_camp
   source source_turtlebot_sim.sh
   ros2 run turtlebot3_example turtlebot3_absolute_move
   ```
   - enter the following values to test it:
   ```bash
   Input x: 2
   Input y: 0
   Input theta: 0
   ```
2. Command the robot to go to different XY positions in the environment.
3. Close the `position_control` node by pressing `ctrl+c`
### Fourth tutorial: control the robot using specific linear and angular velocities
1. Open new terminal Tab, and source the workspace
   ```bash
   cd
    cd precollege_camp
    source source_turtlebot_sim.sh
    ```
1. add the two Python scripts `send_cmd_vel.py` and `obstacle_direction.py` to the `turtlebot3_teleop` package in the script folder `/turtlebot_sim/src/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script`
2. Add the two lines in `setup_update.txt` to the 'setup.py' file of the `turtlebot3_teleop` package under the line containing `console_scripts`
   ```
   'console_scripts': [
        'teleop_keyboard = turtlebot3_teleop.script.teleop_keyboard:main',
       'send_cmd_vel = turtlebot3_teleop.script.send_cmd_vel:main',
       'obstacle_direction = turtlebot3_teleop.script.obstacle_direction:main'
   ],
   ```
3. compile and source the workspace again
   ```bash
   colcon build
    ```
   ```bash
   source install/setup.bash
   ```
5. Run the new `send_cmd_vel` node
   ```bash
   ros2 run turtlebot3_teleop send_cmd_vel
   ```
   - enter the following values to test it:
   ```bash
   Input Linear: 0.5
   Input Angular: 0.5
   ```
6. Close the `send_cmd_vel` node using `ctrl+c`
7. Rerun the same node again, this time try different combinations of linear and angular velocities to move the robot in a straight line, or a circle in clockwise and counter-clockwise directions.
   ```bash
   ros2 run turtlebot3_teleop send_cmd_vel
   ```
8. Close the `send_cmd_vel` node using `ctrl+c`
### Fifth tutorial: detect obstacle direction using a laser scanner
1. Add obstacles to the environment such as boxes, cylinders, or spheres.
2. In the `velocity_tab`, run the `obstacle_direction` node
   ```bash
   ros2 run turtlebot3_teleop obstacle_direction
   ```
### Try new environments:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
or
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

3. Check the code of the `obstacle_direction` node and try to print the `distance` to the nearest obstacle from the robot.


