#include<behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "binbot/panda_robot_interface.hpp"
#include "binbot/move_to_pose_node.hpp"
#include "binbot/move_home_node.hpp"
#include "binbot/open_gripper_node.hpp"
#include "binbot/close_gripper_node.hpp"
#include "binbot/attach_object_node.hpp"
#include "binbot/detach_object_node.hpp"
#include "binbot/add_collision_object_node.hpp"
#include "binbot/wait_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<PandaRobot>();

    //Set the Static Robot Pointer in all your custom nodes
    MoveToPoseNode::setRobot(robot.get());
    MoveHomeNode::setRobot(robot.get());
    OpenGripperNode::setRobot(robot.get());
    CloseGripperNode::setRobot(robot.get());
    AttachObjectNode::setRobot(robot.get());
    DetachObjectNode::setRobot(robot.get());
    AddCollisionObjectNode::setRobot(robot.get());

    //Create the behavior tree factory
    BT::BehaviorTreeFactory factory;

    //Register Custom Nodes
    factory.registerNodeType<MoveToPoseNode>("MoveToPose");
    factory.registerNodeType<MoveHomeNode>("MoveHome");
    factory.registerNodeType<OpenGripperNode>("OpenGripper");
    factory.registerNodeType<CloseGripperNode>("CloseGripper");
    factory.registerNodeType<AttachObjectNode>("AttachObject");
    factory.registerNodeType<DetachObjectNode>("DetachObject");
    factory.registerNodeType<AddCollisionObjectNode>("AddCollisionObject");
    factory.registerNodeType<WaitNode>("wait");

    //Load the Tree from XML
    auto tree = factory.createTreeFromFile("/home/varun/ros2_ws/src/binbot/bt/bt_graspbot.xml");

    //Execute the tree
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while(rclcpp::ok() && status != BT::NodeStatus::SUCCESS)
    {
        status = tree.rootNode()->executeTick();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}