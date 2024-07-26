import os
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

urdf_file_name = 'model.urdf'
package_name = 'basic'

def loadModel():
    urdf = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name,
    )

    with open(urdf, 'r') as infp:
        return urdf, infp.read()
    
def loadSimulationConfiguration():
    rviz = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'urdf.rviz',
    )

    return rviz

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time') 
    robot_name = 'physics'

    urdf, robot = loadModel()

    robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot, 'use_sim_time': use_sim_time}],
        arguments=[urdf],
    )

    rviz_configuration = loadSimulationConfiguration()

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_configuration],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                    "-entity", robot_name,
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '-5.0',
                    "-Y", '1.5']
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
        '-s', 'libgazebo_ros_init.so'], output='screen',
        )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rviz2,
        spawn,
        robot,
        gazebo,
    ])