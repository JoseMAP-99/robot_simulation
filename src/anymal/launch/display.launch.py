import os
import launch
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

urdf_file_name = 'anymal.urdf'
package_name = 'anymal'

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

    urdf, robot = loadModel()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot}],
        arguments=[urdf]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_conf = loadSimulationConfiguration()
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[{'-d', rviz_conf}]
    )

    gui = DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui')

    return LaunchDescription([
        gui,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
