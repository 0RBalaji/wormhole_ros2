import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    pkg_path = get_package_share_directory('assesment')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_yaml_file = os.path.join(pkg_path, 'maps', 'map_a.yaml')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value = map_yaml_file,
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': 'base_footprint'
        # 'robot_base_frame': 'base_link'
    }
    
    configured_params = RewrittenYaml(
        source_file=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': configured_params
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': configured_params
        }.items()
    )    

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(localization)
    ld.add_action(navigation)

    return ld