import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

urdf_file_name = 'anymal.urdf'
package_name = 'anymal'
robot_name_in_model = 'anymal'

def loadModel():
    urdf = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name,
    )

    with open(urdf, 'r') as infp:
        return urdf, infp.read()
    
def load_simulation_config():
    return os.path.join(
        get_package_share_directory(package_name),
        'config',
        'urdf.rviz',
    )

def generate_launch_description():

    # Configuration
    gui_conf = LaunchConfiguration('gui')
    use_sim_time_conf = LaunchConfiguration('use_sim_time') 

    
    # Arguments 
    gui_arg = DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='True',description='Use simulation (Gazebo) clock if true')


    # Nodes
    urdf, robot = loadModel()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot, 'use_sim_time': use_sim_time_conf}],
        arguments=[urdf]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time_conf}],
        condition=UnlessCondition(gui_conf),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time_conf}],
        condition=IfCondition(gui_conf),
    )


    # RVIZ
    rviz_conf = load_simulation_config()
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_conf}],
        arguments=['-d', rviz_conf]
    )


    # GAZEBO
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                    "-entity", robot_name_in_model,
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '0.05',
                    "-Y", '0.0']
    )

    gazebo_node = ExecuteProcess(
        cmd=[
            'gazebo', 
            '--verbose', '-s', 
            'libgazebo_ros_factory.so', 
            '-s', 'libgazebo_ros_init.so'
            ], 
            output='screen',
        )

    # Launch
    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_node,
        rviz_node,
        gazebo_node,
    ])
