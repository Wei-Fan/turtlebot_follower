turtlebot_apps
==============

A group of simple demos and examples to run on your TurtleBot to help you get started and having fun with ROS and TurtleBot.

roslaunch turtlebot_bringup minimal.launch 
roslaunch turtlebot_follower follower.launch 

To record data: rosbag record /turtlebot_follower/marker /follower_velocity_smoother/raw_cmd_vel
To play back: rosbag play XXXXX.bag
To show bag: rqt_bag XXXXX.bag
