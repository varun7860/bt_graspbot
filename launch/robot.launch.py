import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database Flag"
    )

    warehouse_ros_config = {"warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection","warehouse_host": "/home/varun/ros2_ws/src/binbot/sqlite/warehouse_db.sqlite"}
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
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
        parameters=[moveit_config.to_dict(),warehouse_ros_config],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    #Rviz
    #rviz_base = get_package_share_directory('artibot'),"rviz"
    #rviz_full_config = os.path.join(rviz_base,"config.rviz")
    
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
            warehouse_ros_config
            #{"use_sim_time": True}
        ],
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
        parameters=[moveit_config.robot_description],
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
            db_arg,
            ros2_control_hardware_type,
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            joint_state_publisher_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner
        ]
    )

# def generate_launch_description():
    
#     ros2_control_hardware_type = DeclareLaunchArgument(
#         "ros2_control_hardware_type",
#         default_value="mock_components",
#         description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
#     )
      
#     moveit_config = (
#         MoveItConfigsBuilder("moveit_resources_fanuc")
#         .robot_description(
#             file_path="config/fanuc.urdf.xacro",
#             mappings={
#                 "ros2_control_hardware_type": LaunchConfiguration(
#                 "ros2_control_hardware_type"
#                 )    
#             },
#           )
#         .robot_description_semantic(file_path="config/fanuc.srdf")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_pipelines(
#             pipelines= ["ompl","chomp","pilz_industrial_motion_planner"]
#             )
#         .to_moveit_configs()
#     )
    
#     ros2_controllers_path = os.path.join(
#         get_package_share_directory("moveit_resources_fanuc_moveit_config"),
#         "config",
#         "ros2_controllers.yaml",
#         )
    
#     print("ros2 controllers path:", ros2_controllers_path)
    
#     return LaunchDescription([
#         # ExecuteProcess(
#         #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
#         #     output='screen'),
    
#         # Node(
#         #      package="gazebo_ros",
#         #      executable="spawn_entity.py",
#         #      arguments=["-topic","robot_description","-entity","fanuc"],
#         #      output = "screen"
#         #      ),
#         ros2_control_hardware_type,
        
#         Node(
#              package="moveit_ros_move_group",
#              executable="move_group",
#              output="screen",
#              parameters=[moveit_config.to_dict()],     
#         ),
        
#         Node(
#             package='controller_manager',
#             executable='ros2_control_node',
#             parameters=[ros2_controllers_path],
#             remappings=[
#             ("/controller_manager/robot_description", "/robot_description"),
#             ],
#             output="screen",
#         ),
        
#         Node(
#             package="controller_manager",
#             executable="spawner",
#             arguments=[
#                 "joint_state_broadcaster",
#                 "--controller-manager",
#                 "/controller_manager",
#             ],
#         ),
        
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name="robot_state_publisher",
#             output="both",
#             parameters=[moveit_config.robot_description]
#             ),
        
#         Node(
#             package='joint_state_publisher',
#             executable='joint_state_publisher',
#             name='joint_state_publisher',
#             output="screen",
#             ),
        
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', os.path.join(get_package_share_directory('artibot'), 'rviz', 'config.rviz')],
#             parameters=[
#                 moveit_config.robot_description,
#                 moveit_config.robot_description_semantic,
#                 moveit_config.planning_pipelines,
#                 moveit_config.robot_description_kinematics,
#                 ],
#             ),
        
#         #Warehouse mangodb server
#         db_config
        
#         # Warehouse mongodb server
#         db_config = LaunchConfiguration("db")
#         mongodb_server_node = Node(
#             package="warehouse_ros_mongo",
#             executable="mongo_wrapper_ros.py",
#             parameters=[
#                 {"warehouse_port": 33829},
#                 {"warehouse_host": "localhost"},
#                 {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
#             ],
#             output="screen",
#             condition=IfCondition(db_config),
#         ),
        
#     ])
