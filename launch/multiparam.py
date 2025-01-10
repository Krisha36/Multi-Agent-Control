import os
import shutil
import yaml

class MultiParamFolderGenerator:
    """
    A class to generate multi_param folders for each robot with specific files.

    Attributes
    ----------
    num_robots : int
        The number of robots to create folders for.
    template_dir : str
        The directory containing the template files.
    output_dir : str
        The directory where the multi_param folders will be created.

    Methods
    -------
    generate_folders(self):
        Generates the multi_param folders and populates them with modified files.
    modify_file_content(self, content: str, robot_index: int) -> str:
        Modifies the content of the file based on the robot index.
    """

    def __init__(self, num_robots, template_dir, output_dir, launch_dir):
        self.num_robots = num_robots
        self.template_dir = template_dir
        self.output_dir = output_dir
        self.launch_dir = launch_dir

    def generate_folders(self):
        """Generates the multi_param folders and populates them with modified files."""

        multi_nav_launch_path = os.path.join(self.launch_dir, 'multi_nav.launch')
        with open(multi_nav_launch_path, 'w') as f:
            f.write('<?xml version="1.0"?>\n')
            f.write('<launch>\n')
            f.write('  <!-- Arguments -->\n')
            f.write('  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>\n')
            f.write('  <arg name="map_file" default="$(find turtlebot3_navigation)/maps/blank_map.yaml"/>\n')
            
            for k in range(self.num_robots):
                f.write(f'  <include file="/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/launch/nav_tb3_{k}.launch">\n')
                f.write('    <arg name="model" value="$(arg model)" />\n')
                f.write('    <arg name="map_file" value="$(arg map_file)" />\n')
                f.write('  </include>\n')

            f.write('  <!-- rviz -->\n')
            f.write('  <node pkg="rviz" type="rviz" name="rviz" required="false"\n')
            f.write('        args="-d $(find turtlebot3_navigation)/rviz/multi_turtlebot3_navigation_own.rviz"/>\n')
            f.write('</launch>\n')

        for k in range(self.num_robots):
            folder_name = f'multi_param{k}'
            folder_path = os.path.join(self.output_dir, folder_name)

            # Create the multi_param{k} folder
            os.makedirs(folder_path, exist_ok=True)

            # Copy and modify template files
            for filename in os.listdir(self.template_dir):
                template_file_path = os.path.join(self.template_dir, filename)
                new_file_path = os.path.join(folder_path, filename)

                with open(template_file_path, 'r') as template_file:
                    content = template_file.read()

                # Modify the file content based on the robot index
                modified_content = self.modify_file_content(content, k)

                with open(new_file_path, 'w') as new_file:
                    new_file.write(modified_content)

    def modify_file_content(self, content, robot_index):
        """Modifies the content of the file based on the robot index."""
        # Example modification: Add the robot index to some placeholder in the file
        modified_content = content.replace('<robot_index>', str(robot_index))
        return modified_content

def load_config(config_file):
    """Loads the configuration from a YAML file."""
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    return config

if __name__ == "__main__":

    config = load_config('/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/params.yaml')

    num_robots = config['num_of_robots']
    template_dir = '/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/param'  # Directory with template files
    output_dir = '/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/multiparam'  # Directory to create multi_param folders
    launch_dir = '/home/krisha/catkin_ws/src/gazebo_multi_robot_spawn_ow/launch'
    generator = MultiParamFolderGenerator(num_robots, template_dir, output_dir, launch_dir)
    generator.generate_folders()
