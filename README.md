# Run  
roscore  
roslaunch turtlebot3_bringup turtlebot3_robot.launch  
roslaunch open_manipulator_moveit open_manipulator_demo.launch use_gazebo:=false  

# Test  
//rosrun rosserial_python serial_node.py /dev/ttyUSB0  
//rosrun rosserial_python serial_node.py /dev/ttyACM0  
