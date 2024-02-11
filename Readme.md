# Assignment 2b :TurtleBot3 open-loop control Analysis in square and circle
**Authors:** Rahul Chamarthi, Ravi Varikuti, and Benjamin I Johnson  

## Description:
The ROS package [assignment2b_turtlebot3](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/tree/master/assignment2b_turtlebot3) for this assignment contains the following Python  scripts:

- [circle.py](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/blob/master/assignment2b_turtlebot3/src/scripts/circle.py): This script allows the Turtlebot3 burger to move in a circle with a constant twist for a specified distance with varying speeds in the gazebo.
- [square.py](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/blob/master/assignment2b_turtlebot3/src/scripts/square.py): This script allows the Turtlebot3 burger to move in a square with a constant twist for a specified distance with varying speeds in the gazebo.

The ROS package [assignment2b_turtlebot3](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/tree/master/assignment2b_turtlebot3) for this assignment also contains the launch file to execute the scripts:

- [move.launch](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/blob/master/assignment2b_turtlebot3/src/launch/move.launch): launches the circle .py or square .py depending the argument given in an empty world in Gazebo with Turtlebot3 burger executing the tasks.

## Results:

The [videos](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/tree/master/assignment2b_turtlebot3/videos) folder contains the videos of implementation and results

1.Turtlebot3 burger in a circle:
 It is found that, for low velocities, the turtlebot3 burger can make a circle but as the speed increases the the turtle bot  deviates from its circular trajectory. It can be seen in the [circle.mp4](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/blob/master/assignment2b_turtlebot3/videos/circle.mp4)

2.Turtlebot3 burger in a Square: It is found that, for low velocities, the turtlebot3 burger can make a square but as the speed increases the the turtle bot  deviates from its square path. It is demonstrated in [square.mp4](https://github.com/rahulchamarthi/AuE8230Spring24_Group1/blob/master/assignment2b_turtlebot3/videos/square.mp4)





