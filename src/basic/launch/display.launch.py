import os
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    urdf, robot = loadModel()

    robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot}],
        arguments=[urdf]
    )

    rviz_configuration = loadSimulationConfiguration()

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_configuration],
        output='screen',
    )

    gui = DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui')

    return LaunchDescription([
        gui,
        robot,
        rviz2,
    ])