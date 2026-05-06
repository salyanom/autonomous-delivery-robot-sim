import launch


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

                                                                     
from launch.actions import SetEnvironmentVariable
import os


def generate_launch_description():
              
    gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH', '')
    new_model_path = os.path.expanduser('~/ros2_ws/src/RDSim/rdsim_gazebo/models')
    combined_gazebo_model_path = f"{gazebo_model_path}:{new_model_path}" if gazebo_model_path else new_model_path

                                
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=combined_gazebo_model_path
    )

                          
    world_dir= PathJoinSubstitution(
        [
            FindPackageShare('rdsim_gazebo'),
            'worlds',
            'small_city.world'
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation'),
            
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'),

                          
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_dir],
            output='screen'),

    ])
