#pragma once

#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <streambuf>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.hpp>
#include <boost/variant.hpp>

class PandaRobot
{
public:
    PandaRobot();
    ~PandaRobot();

    geometry_msgs::msg::Pose createPose(double x=0, double y=0, double z=0,
                                        double ox=0, double oy=0, double oz=0, double w=1);

    geometry_msgs::msg::Pose get_current_pose();
    std::vector<double> get_joint_values();
    geometry_msgs::msg::Pose getPredefinedPose(const std::string& name);

    bool planAndExecute(const geometry_msgs::msg::Pose& pose);
    bool moveLinear(const geometry_msgs::msg::Pose& target_pose);
    bool moveJoints(const std::vector<double>& joint_values);

    void gripperOpen(double j1=0.035, double j2=0.035);
    void gripperClose();
    void attachObject(const std::string& id, const std::string& link_name);
    void detachObject(const std::string& id);
    void addCollisionObjects(const std::string& type = "default", const std::string& id = "box",
                             double x=0.4, double y=0.0, double z=0.3,
                             double ox=0.0, double oy=0.0, double oz=0.0, double w=1.0);

private:
    bool loadSRDF(const std::string& path);
    bool loadKinematics(const std::string& path);

    //rclcpp::Logger logger_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
    std::thread spinner_;
};
