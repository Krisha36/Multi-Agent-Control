#!/usr/bin/python3
import rospy
import os
import rospkg
from pydantic import BaseModel
from spawn_formation import *
from nav_msgs.msg import Odometry
import time

class LaunchParams(BaseModel):
    """
    Represents the launch parameters for the robots.

    Attributes:
        num_of_robots (int): The number of robots.
        robot_model (str): The model of the robot.
        distribution (str): The distribution method.
        distribution_list (dict): The distribution list.
    """
    num_of_robots: int
    robot_model: str
    distribution_type: str
    distribution_params: dict

class LaunchFileGenerator:
    """
    A class to generate a launch file for spawning robots in Gazebo.

    Attributes
    ----------
    params : LaunchParams
        An instance of the LaunchParams class that contains the parameters for the launch file.
    num_robots : int
        The number of robots to be spawned.
    model : str
        The model of the robots to be spawned.
    start_formation_type : str
        The type of formation in which the robots should be spawned.
    distribution : dict
        The parameters for the distribution of the robots.
    output_file_name : str
        The path to the output launch file.

    Methods
    -------
    __init__(self, params: LaunchParams):
        Initializes the LaunchFileGenerator with parameters from a LaunchParams instance.

    load_params(config: dict) -> LaunchParams:
        Reads parameters from a dictionary and returns a LaunchParams instance.

    generate_launch_file(self):
        Generates a launch file for spawning robots in Gazebo.

    run(self):
        Runs the LaunchFileGenerator.
    """
   
    def __init__(self, params: LaunchParams):
        """Initializes the LaunchFileGenerator with parameters from a YAML file."""
        self.params = params
        self.num_robots = self.params.num_of_robots
        self.model = self.params.robot_model
        self.start_formation_type = self.params.distribution_type
        self.distribution = self.params.distribution_params
        rospack = rospkg.RosPack()
        self.startx = None
        self.starty = None
        package_path = rospack.get_path('gazebo_multi_robot_spawn')  # Replace with your package name
        self.output_file_name = os.path.join(os.path.join(package_path, 'launch'), 'goal.sh')

    def odom_callback(self, odom_msg):
        # Retrieve odometry data and store it for path planning
        self.startx = odom_msg.pose.pose.position.x
        self.starty = odom_msg.pose.pose.position.y

    def get_position(self):
        # Return the current x and y coordinates
        return self.startx, self.starty

    @staticmethod
    def load_params(config: dict) -> LaunchParams:
        """Reads parameters from a dictionary and returns a LaunchParams instance."""
        return LaunchParams(**config)

    def generate_launch_file(self):
        """Generates a launch file for spawning robots in Gazebo."""
        with open(self.output_file_name, "w") as f:
            f.write('#!/bin/bash\n')
            f.write('sleep $1\n')
            #startx = [None] * self.num_robots
            #starty = [None] * self.num_robots
            
                


            for k in range(self.num_robots):
                if self.start_formation_type == 'circle':
                    formation_class = Circle(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'line':
                    formation_class = Line(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'two_lines':
                    formation_class = TwoLines(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'three_lines':
                    formation_class = ThreeLines(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'rectangle':
                    formation_class = Rectangle(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                
                topic = f"/tb3_{k}/odom"
                
                odom_msg = rospy.wait_for_message(topic, Odometry, timeout=10)

                f.write(f'rosrun rrt_star_path_planning_turtlebot rrt_demo "{round(odom_msg.pose.pose.position.x,2)},{round(odom_msg.pose.pose.position.y,2)}" "{round(x,2)},{round(y,2)}" "/tb3_{k}/move_base_simple/goal" "/tb3_{k}/odom" "{k}"')
                if(k<(self.num_robots-1)):
                    f.write(" &  \n")
                else: 
                    f.write(" \n")


            

    def run(self):
        """Runs the LaunchFileGenerator."""
        rospy.init_node('launch_file_generator', anonymous=True)
        print('running')
        self.generate_launch_file()
        rospy.loginfo("Launch file generated at: %s", self.output_file_name)
        rospy.signal_shutdown("Work Completed")


if __name__ == "__main__":
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('gazebo_multi_robot_spawn')  # Replace with new package name if altered. 
        config = {
            "num_of_robots": rospy.get_param('num_of_robots'),
            "robot_model": rospy.get_param('robot_model'),
            "distribution_type": rospy.get_param('distribution_type'),
            "distribution_params": rospy.get_param('distribution_params')
        }
        #print("helloo")
        params = LaunchFileGenerator.load_params(config)
        generator = LaunchFileGenerator(params)
        generator.run()
    except rospy.ROSInterruptException:
        pass
