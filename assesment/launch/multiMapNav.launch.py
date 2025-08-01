import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from xacro import process_file

def generate_launch_description():
    # Get the launch directory

    pkg_path = get_package_share_directory('assesment')

    tb3_description_dir = get_package_share_directory('turtlebot3_gazebo')

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = PathJoinSubstitution([pkg_path, 'worlds', 'model.world'])

    map_yaml_file = os.path.join(pkg_path, 'maps', 'map_1.yaml')

    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle_pi.urdf')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value = map_yaml_file,
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_tb3_model_cmd = DeclareLaunchArgument(
    'turtlebot3_model',
    default_value='waffle_pi',  # or 'waffle', 'waffle_pi'
    description='Model type [burger, waffle, waffle_pi]')
    
    declare_pose_cmds = [
        DeclareLaunchArgument('x_pose', default_value='1.47', description='X position'),
        DeclareLaunchArgument('y_pose', default_value='6.46', description='Y position'),
        DeclareLaunchArgument('z_pose', default_value='0.05', description='Z position'),
        DeclareLaunchArgument('roll', default_value='0.00', description='Roll'),
        DeclareLaunchArgument('pitch', default_value='0.00', description='Pitch'),
        DeclareLaunchArgument('yaw', default_value='-1.57', description='Yaw')
    ]
    
    pose = {
        'x': LaunchConfiguration('x_pose', default='1.47'),
        'y': LaunchConfiguration('y_pose', default='6.46'),
        'z': LaunchConfiguration('z_pose', default='0.05'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='-1.57')
    }

    # -9, -1, 0.05, 0.00, 0.00, 0.0
    # -1, 6.1, 0.05, 0, 0, -3.142
    # -5.5, 6, 0.05, 0, 0, 0
    # 2.3, -0.8, ......
    
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_description_dir, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_description_dir, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': pose['x'],
            'y_pose': pose['y']
        }.items()
    )

    rviz_config_file = os.path.join(pkg_path, 'config', 'default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'bringup.launch.py')),
        # launch_arguments={
        #     # 'map': map_yaml_file,
        #     'use_sim_time': use_sim_time,
        # }.items()
    )

    # # Add the wormhole DB and action server also
    wormhole_db = Node(
        package='assesment',
        executable='database_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    wormhole_action_server = Node(
        package='assesment',
        executable='multi_map_action_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    for cmd in declare_pose_cmds:
        ld.add_action(cmd)

    ld.add_action(gazebo)
    ld.add_action(robot)

    ld.add_action(spawn)
    ld.add_action(rviz_node)

    ld.add_action(bringup_node)

    # ld.add_action(wormhole_db)
    ld.add_action(wormhole_action_server)

    return ld
