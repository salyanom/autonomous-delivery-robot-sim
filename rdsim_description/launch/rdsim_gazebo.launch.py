import launch


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

                                                                     
from launch.actions import SetEnvironmentVariable
from launch_ros.descriptions import ParameterValue

                                                                     
from launch.actions import SetEnvironmentVariable
import os

from scripts import GazeboRosPaths


def generate_launch_description():
              
    model, plugin, media = GazeboRosPaths.get_paths()

    gazebo_model_path = os.getenv("GAZEBO_MODEL_PATH", "")
    rdsim_gazebo_model_path = os.path.expanduser(
        "~/ros2_ws/src/RDSim/rdsim_gazebo/models"
    )
    if gazebo_model_path:
        rdsim_gazebo_model_path = f"{gazebo_model_path}:{rdsim_gazebo_model_path}"
    combined_gazebo_model_path = f"{model}:{rdsim_gazebo_model_path}"

    gazebo_resource_path = os.getenv("GAZEBO_RESOURCE_PATH", "")
    combined_gazebo_resource_path = (
        f"{gazebo_resource_path}:{media}" if gazebo_resource_path else media
    )

                                
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=combined_gazebo_model_path
    )
                                 
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=plugin
    )
                                
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=combined_gazebo_resource_path
    )

    start_rviz = LaunchConfiguration("start_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui", default="True")
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

                               
    default_model_dir = PathJoinSubstitution(
        [FindPackageShare("rdsim_description"), "urdf", "rdsim.urdf.xacro"]
    )

                         
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rdsim_description"), "rviz", "display.rviz"]
    )
                          
    world_dir = PathJoinSubstitution(
        [FindPackageShare("rdsim_gazebo"), "worlds", "small_city.world"]
    )

                                            

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model")]), value_type=str
                ),
            }
        ],
    )

                                            
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
    )

                           
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
                         
        condition=IfCondition(start_rviz),
    )

                             
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "rdsim",
            "-topic",
            "robot_description",
            "-x",
            "0.5",
            "-y",
            "0.5",
            "-z",
            "0.01",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            set_gazebo_model_path,
            set_gazebo_plugin_path,
            set_gazebo_resource_path,
                                    
            DeclareLaunchArgument(
                "start_rviz", default_value="true", description="Whether execute rviz2"
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value="true",
                description="Start robot in Gazebo simulation",
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="False",
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_dir,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=rviz_config_file,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
                            
            launch.actions.ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    world_dir,
                ],
                output="screen",
                condition=IfCondition(use_gazebo_gui),
            ),
                            
            launch.actions.ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    world_dir,
                ],
                output="screen",
                condition=UnlessCondition(use_gazebo_gui),
            ),
                                 
                               
                                         
            robot_state_publisher_node,
                                             
                                 
            spawn_entity,
                        
        ]
    )
