from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    uryt_share     = get_package_share_directory("ur_yt_sim")
    robotiq_share  = get_package_share_directory("robotiq_description")
    ur_share       = get_package_share_directory("ur_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(
        get_package_share_directory('ur_yt_sim'), 'worlds', 'world2.world'
    )
    # --- ENV Gazebo (minimi, nessun assoluto hard-coded) ---
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(["/usr/share/gazebo-11", uryt_share, robotiq_share, ur_share])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join([os.path.join(uryt_share,"models"),
                        os.path.join(robotiq_share,"models"),
                        os.path.expanduser("~/.gazebo/models")])
    ))
    # Plugin ROS di Gazebo
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join([
            "/opt/ros/humble/lib",
            os.path.normpath(os.path.join(uryt_share, "..", "..", "lib")),  # ~/…/install/lib
        ])
    ))

    # --- argomenti ---
    with_rviz = DeclareLaunchArgument("with_rviz", default_value="true")
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    ld.add_action(with_rviz); ld.add_action(x_arg); ld.add_action(y_arg); ld.add_action(z_arg)

    # --- MoveIt config -> genera robot_description (xacro + mappings) ---
    joint_controllers_file = os.path.join(uryt_share, "config", "ur5_controllers_gripper.yaml")
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur5",
                "sim_gazebo": "true",
                "sim_ignition": "false",
                "use_fake_hardware": "false",
                "simulation_controllers": joint_controllers_file,
                "initial_positions_file": os.path.join(uryt_share, "config", "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # --- Gazebo (in pausa) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"use_sim_time":"true", "gui":"true", "paused":"true", 'world' : world_file}.items()
    )
    ld.add_action(gazebo)

    # --- RSP (usa il robot_description generato) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn dal topic /robot_description ---
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity","cobot",
            "-topic","robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn]))  # dai tempo a Gazebo

    # --- Spawner controller (dopo lo spawn) ---
    jsb  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"], output="screen")
    arm  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_trajectory_controller","--controller-manager","/controller_manager"], output="screen")
    grip = Node(package="controller_manager", executable="spawner",
                arguments=["gripper_position_controller","--controller-manager","/controller_manager"], output="screen")

    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=spawn, on_start=[
            TimerAction(period=2.0, actions=[jsb]),
            TimerAction(period=3.0, actions=[arm, grip]),
        ])
    ))

    # --- RViz (opzionale) ---
    rviz_cfg = os.path.join(get_package_share_directory("ur5_camera_gripper_moveit_config"),
                            "config", "moveit.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz"))
    )
    ld.add_action(rviz)

    # --- MoveGroup (se usi MoveIt in questa sessione) ---
    mg_params = moveit_config.to_dict(); mg_params.update({"use_sim_time": True})
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", parameters=[mg_params],
        arguments=["--ros-args","--log-level","info"],
    )
    ld.add_action(move_group)

    return ld