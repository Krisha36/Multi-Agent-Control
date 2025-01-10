#!/bin/bash
sleep $1
rosrun rrt_star_path_planning_turtlebot rrt_demo "-1.5,0.0" "1.0,0.0" "/tb3_0/move_base_simple/goal" "/tb3_0/odom" "0" &  
rosrun rrt_star_path_planning_turtlebot rrt_demo "-0.5,0.0" "0.0,1.0" "/tb3_1/move_base_simple/goal" "/tb3_1/odom" "1" &  
rosrun rrt_star_path_planning_turtlebot rrt_demo "0.5,0.0" "-1.0,0.0" "/tb3_2/move_base_simple/goal" "/tb3_2/odom" "2" &  
rosrun rrt_star_path_planning_turtlebot rrt_demo "1.5,0.0" "-0.0,-1.0" "/tb3_3/move_base_simple/goal" "/tb3_3/odom" "3" 
