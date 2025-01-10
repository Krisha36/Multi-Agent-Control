#!/usr/bin/python3
import rospy
import os
import rospkg
from pydantic import BaseModel
from spawn_formation import *

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
    A class to generate launch files for spawning robots in Gazebo.

    Attributes
    ----------
    params : LaunchParams
        An instance of the LaunchParams class that contains the parameters for the launch files.
    num_robots : int
        The number of robots to be spawned.
    model : str
        The model of the robots to be spawned.
    start_formation_type : str
        The type of formation in which the robots should be spawned.
    distribution : dict
        The parameters for the distribution of the robots.

    Methods
    -------
    __init__(self, params: LaunchParams):
        Initializes the LaunchFileGenerator with parameters from a LaunchParams instance.

    load_params(config: dict) -> LaunchParams:
        Reads parameters from a dictionary and returns a LaunchParams instance.

    generate_launch_files(self):
        Generates a launch file for each robot to be spawned in Gazebo.

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
        package_path = rospack.get_path('gazebo_multi_robot_spawn')  # Replace with your package name
        self.output_dir = os.path.join(package_path, 'launch')

    @staticmethod
    def load_params(config: dict) -> LaunchParams:
        """Reads parameters from a dictionary and returns a LaunchParams instance."""
        return LaunchParams(**config)

    def generate_launch_files(self):
        """Generates a launch file for each robot to be spawned in Gazebo."""
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
            
            output_file_name = os.path.join(self.output_dir, f'nav_tb3_{k}.launch')
            with open(output_file_name, "w") as f:
                f.write('<?xml version="1.0"?>\n')
                f.write('<launch>\n')
                f.write('  <!-- Arguments -->\n')
                f.write('  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>\n')
                f.write('  <arg name="map_file" default="$(find turtlebot3_navigation)/maps/map.yaml"/>\n')
                f.write('  <arg name="open_rviz" default="false"/>\n')
                f.write('  <arg name="move_forward_only" default="false"/>\n')
                #f.write(f'  <arg name="first_tb3" default="tb3_{k}" />\n')
                f.write(f'  <group ns="tb3_{k}">\n')
                f.write('  <!-- Map server -->\n')
                f.write('  <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/>\n')
                f.write('  <!-- AMCL -->\n')
                f.write(' <node pkg="amcl" type="amcl" name="amcl">\n')
                f.write('   <param name="min_particles" value="500"/>\n')
                f.write('   <param name="max_particles" value="3000"/>\n')
                f.write('   <param name="kld_err" value="0.02"/>\n')
                f.write('   <param name="update_min_d" value="0.20"/>\n')
                f.write('   <param name="update_min_a" value="0.20"/>\n')
                f.write('   <param name="resample_interval" value="1"/>\n')
                f.write('   <param name="transform_tolerance" value="0.5"/>\n')
                f.write('   <param name="recovery_alpha_slow" value="0.00"/>\n')
                f.write('   <param name="recovery_alpha_fast" value="0.00"/>\n')
                f.write(f'   <param name="initial_pose_x" value="{x}"/>\n')
                f.write(f'   <param name="initial_pose_y" value="{y}"/>\n')
                f.write('   <param name="initial_pose_a" value="0.0"/>\n')
                f.write('   <param name="gui_publish_rate" value="50.0"/>\n')
                f.write('   <remap from="scan" to="scan"/>\n')
                f.write('   <param name="laser_max_range" value="3.5"/>\n')
                f.write('   <param name="laser_max_beams" value="180"/>\n')
                f.write('   <param name="laser_z_hit" value="0.5"/>\n')
                f.write('   <param name="laser_z_short" value="0.05"/>\n')
                f.write('   <param name="laser_z_max" value="0.05"/>\n')
                f.write('   <param name="laser_z_rand" value="0.5"/>\n')
                f.write('   <param name="laser_sigma_hit" value="0.2"/>\n')
                f.write('   <param name="laser_lambda_short" value="0.1"/>\n')
                f.write('   <param name="laser_likelihood_max_dist" value="2.0"/>\n')
                f.write('   <param name="laser_model_type" value="likelihood_field"/>\n')
                f.write('   <param name="odom_model_type" value="diff"/>\n')
                f.write('   <param name="odom_alpha1" value="0.1"/>\n')
                f.write('   <param name="odom_alpha2" value="0.1"/>\n')
                f.write('   <param name="odom_alpha3" value="0.1"/>\n')
                f.write('   <param name="odom_alpha4" value="0.1"/>\n')
                f.write(f'   <param name="odom_frame_id" value="/tb3_{k}/odom"/>\n')
                f.write(f'   <param name="base_frame_id" value="/tb3_{k}/base_footprint"/>\n')
                f.write(' </node>\n')
                f.write('  <!-- move_base -->\n')
                f.write('  <!-- Arguments -->\n')
                f.write('  <arg name="cmd_vel_topic" default="cmd_vel" />\n')
                f.write('  <arg name="odom_topic" default="odom" />\n')
                f.write('  <!-- move_base -->\n')
                f.write('  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">\n')
                f.write('    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />\n')
                f.write(f'    <rosparam file="/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/multiparam/multi_param{k}/local_costmap_params.yaml" command="load" />\n')
                f.write(f'    <rosparam file="/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/multiparam/multi_param{k}/global_costmap_params.yaml" command="load" />\n')
                f.write(f'    <rosparam file="/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/multiparam/multi_param{k}/move_base_params.yaml" command="load" />\n')
                f.write(f'    <rosparam file="/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/multiparam/multi_param{k}/dwa_local_planner_params_$(arg model).yaml" command="load" />\n')
                f.write('    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>\n')
                f.write('    <remap from="odom" to="$(arg odom_topic)"/>\n')
                f.write('    <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)" />\n')
                f.write('  </node>\n')
                f.write('  </group>\n')
                f.write('  <!-- rviz -->\n')
                f.write('  <group if="$(arg open_rviz)">\n')
                f.write('    <node pkg="rviz" type="rviz" name="rviz" required="true"\n')
                f.write('          args="-d $(find turtlebot3_navigation)/rviz/turtlebot3_navigation.rviz"/>\n')
                f.write('  </group>\n')
                f.write('</launch>\n')

            rospy.loginfo("Launch file generated at: %s", output_file_name)

    def run(self):
        """Runs the LaunchFileGenerator."""
        rospy.init_node('launch_file_generator', anonymous=True)
        print('running')
        self.generate_launch_files()
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
        params = LaunchFileGenerator.load_params(config)
        generator = LaunchFileGenerator(params)
        generator.run()
    except rospy.ROSInterruptException:
        pass
