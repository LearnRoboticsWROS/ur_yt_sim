#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_interface");

    // Logger
    auto logger = rclcpp::get_logger("move_group_interface");

    // Spinner with more thread for avoiding blocks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // Wait initialization
    rclcpp::sleep_for(std::chrono::seconds(2));

    // MoveIt2 interface
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur5_manipulator");
    move_group.setPoseReferenceFrame("base_link");
    move_group.setPlanningTime(10.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    RCLCPP_INFO(logger, "Pose reference frame set to: %s", move_group.getPoseReferenceFrame().c_str());

    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = -0.6;
    target_pose.position.y = -0.5;
    target_pose.position.z = 0.3;

    move_group.setPoseTarget(target_pose, "tool0");

    RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning
    MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");

    // Execution
    if (success)
    {
        move_group.move();
        RCLCPP_INFO(logger, "Motion execution completed.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Motion planning failed!");
    }

    // stop the spinner
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}