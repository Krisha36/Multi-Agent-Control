import os
import yaml
from pydantic import BaseModel
from spawn_formation import *

def load_config(config_file):
    """Loads the configuration from a YAML file."""
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config

config = load_config('/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/goal_params.yaml')

num_robots = config['num_of_robots']
distribution_type = config['distribution_type']
distribution_params = config['distribution_params']


output_dir = '/home/krisha/catkin_ws/src/turtlebot3/RRTstar_path_planning_with_turtlebot/src'
    
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

file_name = os.path.join(output_dir, "multi_initx.cpp")
with open(file_name, "w") as f:
    # Include necessary headers
    f.write('#include <tf/transform_listener.h>\n')
    f.write('#include <geometry_msgs/PoseWithCovarianceStamped.h>\n')
    f.write('#include <nav_msgs/Odometry.h>\n')
    f.write('#include <ros/ros.h>\n')
    f.write('#include <tf/transform_datatypes.h>\n\n')

    # Declare publisher array
    f.write(f'ros::Publisher pub[{num_robots}];\n\n')

    # Define callbacks
    for i in range(num_robots):
        f.write(f'void Callback{i}(const nav_msgs::Odometry::ConstPtr& msg)\n')
        f.write('{\n')
        f.write('    double x, y, z;\n')
        f.write('    x = msg->pose.pose.position.x;\n')
        f.write('    y = msg->pose.pose.position.y;\n')
        f.write('    z = msg->pose.pose.position.z;\n')
        f.write('    std::string fixed_frame = "map";\n')
        f.write('    geometry_msgs::PoseWithCovarianceStamped pose;\n')
        f.write('    pose.header.frame_id = fixed_frame;\n')
        f.write('    pose.header.stamp = ros::Time::now();\n\n')
        f.write('    pose.pose.pose.position.x = x;\n')
        f.write('    pose.pose.pose.position.y = y;\n')
        f.write('    pose.pose.pose.position.z = z;\n\n')
        f.write('    tf::Quaternion quat;\n')
        f.write('    quat.setRPY(0.0, 0.0, 0);\n')
        f.write('    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);\n')
        f.write('    pose.pose.covariance[6*0+0] = 0.5 * 0.5;\n')
        f.write('    pose.pose.covariance[6*1+1] = 0.5 * 0.5;\n')
        f.write('    pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;\n\n')
        f.write(f'    pub[{i}].publish(pose);\n')
        f.write('}\n\n')

    # Define main function
    f.write('int main(int argc, char** argv)\n')
    f.write('{\n')
    f.write('    ros::init(argc, argv, "set_pose_node");\n')
    f.write('    ros::NodeHandle nh_;\n\n')
    for i in range(num_robots):
        f.write(f'    pub[{i}] = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/tb3_{i}/initialpose", 1);\n')
        f.write(f'    ros::Subscriber sub{i} = nh_.subscribe("/tb3_{i}/odom", 1000, Callback{i});\n')
    f.write('    ros::Rate loop_rate(10);\n')
    f.write('    int i = 0;\n\n')
    f.write('    while (i <= 10) {\n')
    f.write('        ros::spinOnce();\n')
    f.write('        i = i + 1;\n')
    f.write('        loop_rate.sleep();\n')
    f.write('    }\n')
    f.write('    return 0;\n')
    f.write('}\n')

with open(os.path.join(output_dir, "multi_goalx.cpp"), "w") as f:
    
    f.write('#include <tf/transform_listener.h>\n')
    f.write('#include <geometry_msgs/PoseStamped.h>\n')
    f.write('#include <nav_msgs/Odometry.h>\n')
    f.write('\n')
    #f.write('double pose(int flag);\n')
    f.write('\n')
    f.write(f'ros::Publisher goal[{num_robots}];\n\n')

    #f.write('ros::Publisher ' + ','.join([f'goal{i}' for i in range(number_of_robots)]) + ';\n')
    f.write('\n')
    f.write('int main(int argc, char** argv)\n')
    f.write('{\n')
    f.write('    ros::init(argc, argv, "set_goal_node");\n')
    f.write('    ros::NodeHandle nh_;\n')
    f.write('\n')

    for i in range(num_robots):
        f.write(f'    goal[{i}] = nh_.advertise<geometry_msgs::PoseStamped> ("/tb3_{i}/move_base_simple/goal", 1);\n')

    f.write('\n')
    f.write('    std::string fixed_frame = "map";\n')

    for i in range(num_robots):
        f.write(f'    geometry_msgs::PoseStamped pose{i};\n')
        f.write(f'    pose{i}.header.frame_id = fixed_frame;\n')
        f.write(f'    pose{i}.header.stamp = ros::Time::now();\n')
        f.write(f'    pose{i}.pose.position.z = 0.0;\n')

    f.write('\n')
    
    f.write('\n')
    f.write('    tf::Quaternion quat;\n')
    f.write('    quat.setRPY(0.0, 0.0, 0);\n')

    for i in range(num_robots):
        f.write(f'    tf::quaternionTFToMsg(quat, pose{i}.pose.orientation);\n')
        if distribution_type == 'circle':
            formation_class = Circle(**distribution_params)
            x, y = formation_class.distribute(i, num_robots)
        elif distribution_type == 'line':
            formation_class = Line(**distribution_params)
            x, y = formation_class.distribute(i, num_robots)
        elif distribution_type == 'two_lines':
            formation_class = TwoLines(**distribution_params)
            x, y = formation_class.distribute(i, num_robots)
        elif distribution_type == 'three_lines':
            formation_class = ThreeLines(**distribution_params)
            x, y = formation_class.distribute(i, num_robots)
        elif distribution_type == 'rectangle':
            formation_class = Rectangle(**distribution_params)
            x, y = formation_class.distribute(i, num_robots)

        f.write(f'    pose{i}.pose.position.x = {round(x,2)};\n')
        f.write(f'    pose{i}.pose.position.y = {round(y,2)};\n')

    f.write('\n')
    f.write('    ros::Rate loop_rate(10);\n')
    f.write('    int i = 0;\n')
    f.write('\n')
    f.write('    while (i<=10){\n')

    for k in range(num_robots):
        
        f.write(f'        goal[{k}].publish(pose{k});\n')

    f.write('        i=i+1;\n')
    f.write('        loop_rate.sleep();\n')
    f.write('    }\n')
    f.write('    return 0;\n')
    f.write('}\n')

print(f"C++ file generated at: {file_name}")

