import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

#For Gazebo Sim
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database Flag"
    )

    warehouse_ros_config = {"warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection","warehouse_host": "/home/varun/ros2_ws/src/binbot/sqlite/warehouse_db.sqlite"}
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="ign_ros2_control/IgnitionSystem",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac, ign_ros2_control/IgnitionSystem]",
    )

    urdf_path = os.path.join(get_package_share_directory("panda_moveit_config"),"config","panda.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name ="panda_moveit_config")
        .robot_description(
            file_path= urdf_path,
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path=os.path.join(get_package_share_directory("panda_moveit_config"),"config","panda.srdf"))
        .robot_description_kinematics(file_path=os.path.join(get_package_share_directory("panda_moveit_config"),"config", "kinematics.yaml"))
        .trajectory_execution(file_path=os.path.join(get_package_share_directory("panda_moveit_config"),"config", "gripper_moveit_controllers.yaml"))
        .planning_scene_monitor(warehouse_ros_config)
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),warehouse_ros_config,{"use_sim_time":True}],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('binbot'), 'rviz', 'config.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            warehouse_ros_config,
            {"use_sim_time":True},
        ],
    )

    #Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch","gz_sim.launch.py")),
            launch_arguments={'gz_args': '-r /home/varun/ros2_ws/src/binbot/world/franka.world'}.items(),
    )
    
    #Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )
    
    #Publish TF
    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,{"use_sim_time": True}],
    )

    #Gazebo Spawn Entity
    spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-name", "panda",
        "-topic", "robot_description",
        "-x", "0.1",
        "-y", "0.1",
        "-z", "1.0"
    ],
    output="screen"
    )
    
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_state_publisher_spawner = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}]
        )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )
    
    #Warehouse mongodb server
    # db_config = LaunchConfiguration("db")
    # mongodb_server_node = Node(
    #     package="warehouse_ros_mongo",
    #     executable="mongo_wrapper_ros.py",
    #     parameters=[
    #         {"warehouse_port": 33829},
    #         {"warehouse_host": "localhost"},
    #         {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
    #     ],
    #     output="screen",
    #     condition=IfCondition(db_config),
    # )

    # mongodb_server_node = Node(
    #    package="warehouse_ros_mongo",
    #    executable="mongo_wrapper_ros.py",
    #    parameters=[
    #        warehouse_ros_config,
    #    ],
    #    output="screen",
    # )
    
    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            db_arg,
            ros2_control_hardware_type,
            rviz_node,
            gazebo_launch,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            spawn_entity,
            move_group_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner
        ]
    )