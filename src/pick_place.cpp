#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

const double tau = 2 * M_PI;


class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "ur5_manipulator"), //
          gripper(node, "robotiq_gripper"), //gripper
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace")),

          // link attacher
          node_(node)
    {
        move_group.setPoseReferenceFrame("base_link");
        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.2);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
        gripper.move();
    }
    void pick()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);  
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.03); 
       
        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.14, 0, -1.57);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = 0.48;
        pick_pose.position.y = 0;
        pick_pose.position.z = 0.4;
        move_group.setPoseTarget(pick_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Pick motion execution completed.");

            close_gripper();
            rclcpp::sleep_for(std::chrono::seconds(1));
            
            attachObject();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }
    }

    void place()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);  
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.03); 

        // Creazione della posa target per il pick
        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-2.964, 0, -1.57);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = 0.426;
        place_pose.position.y = 0.383;
        place_pose.position.z = 0.452;

        move_group.setPoseTarget(place_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";  // Nome del robot in Gazebo
        request->link1_name = "wrist_3_link"; // Nome del link del gripper
        request->model2_name = "cube_pick"; // Nome dell'oggetto da afferrare
        request->link2_name = "link_1";    // Nome del link dell'oggetto

        while (!attach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";  // Nome del robot in Gazebo
        request->link1_name = "wrist_3_link"; // Nome del link del gripper
        request->model2_name = "cube_pick"; // Nome dell'oggetto da afferrare
        request->link2_name = "link_1";    // Nome del link dell'oggetto

        while (!detach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }
    }




    void addCollisionObjects()
    {

        collision_objects.resize(4);

        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "world";
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[0].primitives[0].dimensions = {0.608, 2.0, 1.0};
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.576;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = 0.5;
        collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

        collision_objects[1].id = "wall1";
        collision_objects[1].header.frame_id = "world";
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[1].primitives[0].dimensions = {0.5, 0.05, 0.5};
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0.5;
        collision_objects[1].primitive_poses[0].position.y = 0.2;
        collision_objects[1].primitive_poses[0].position.z = 1.25;
        collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

        collision_objects[2].id = "wall2";
        collision_objects[2].header.frame_id = "world";
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[2].primitives[0].dimensions = {0.5, 0.05, 0.5};
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 0.5;
        collision_objects[2].primitive_poses[0].position.y = 0.8;
        collision_objects[2].primitive_poses[0].position.z = 1.25;
        collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

        collision_objects[3].id = "basement";
        collision_objects[3].header.frame_id = "world";
        collision_objects[3].primitives.resize(1);
        collision_objects[3].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        collision_objects[3].primitives[0].dimensions = {0.8, 0.2}; 
        collision_objects[3].primitive_poses.resize(1);
        collision_objects[3].primitive_poses[0].position.x = 0.0;
        collision_objects[3].primitive_poses[0].position.y = 0.0;
        collision_objects[3].primitive_poses[0].position.z = 0.4;
        collision_objects[3].operation = moveit_msgs::msg::CollisionObject::ADD;


        // Add objects to the scene
        planning_scene_interface.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(logger, "Collision objects added to the planning scene.");

    }

    // void attachCollisionObject()
    // {
    //     //moveit_msgs::AttachedCollisionObject attached_object;
    //     attached_object.link_name = "wrist_3_link";
    //     attached_object.object = collision_objects[3];  

    //     attached_object.object.operation = attached_object.object.ADD;

    //     std::vector<std::string> touch_links;

    //     touch_links.push_back("robotiq_85_right_finger_tip_link");
    //     touch_links.push_back("robotiq_85_left_finger_tip_link");
    //     touch_links.push_back("wrist_3_link");

    //     attached_object.touch_links = touch_links;

    //     planning_scene_interface.applyAttachedCollisionObject(attached_object);
    // }

    // void detachCollisionObject()
    // {
    //     // Specify the link to which the object is currently attached
    //     attached_object.link_name = "wrist_3_link";
    //     // Define the operation as removing the attachment
    //     attached_object.object.operation = attached_object.object.REMOVE;
    //     // Apply the detachment operation to the planning scene
    //     planning_scene_interface.applyAttachedCollisionObject(attached_object);
    // }


private:

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    rclcpp::Logger logger;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;

};



int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_node");

    PickAndPlace pick_and_place(node);

    // Add Collision object
    pick_and_place.addCollisionObjects();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Pick Execution
    pick_and_place.pick();

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Attach collision object
    // pick_and_place.attachCollisionObject();
    // rclcpp::sleep_for(std::chrono::seconds(1));

    // Place Execution
    pick_and_place.place();
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Open gripper
    pick_and_place.open_gripper();
    // rclcpp::sleep_for(std::chrono::seconds(1));
    // pick_and_place.detachCollisionObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    pick_and_place.detachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));


    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}