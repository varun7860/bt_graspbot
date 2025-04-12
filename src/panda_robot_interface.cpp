#include "binbot/panda_robot_interface.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

PandaRobot::PandaRobot()
{
    // Initialize ROS before any node or logger creation
    node_ = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    //logger_ = node_->get_logger();  

    executor_.add_node(node_);
    spinner_ = std::thread([this]() { executor_.spin(); });

    if (!loadSRDF("/home/varun/ros2_ws/src/panda_moveit_config/config/panda.srdf"))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load SRDF. Exiting...");
        std::exit(1);
    }

    if (!loadKinematics("/home/varun/ros2_ws/src/panda_moveit_config/config/kinematics.yaml"))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load kinematics. Exiting...");
        std::exit(1);
    }

    move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");
    planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_,"hand");
    rclcpp::sleep_for(std::chrono::seconds(2));
    addCollisionObjects("default","box");
}

PandaRobot::~PandaRobot()
{
    rclcpp::shutdown();
    if (spinner_.joinable())
        spinner_.join();
}

geometry_msgs::msg::Pose PandaRobot::getPredefinedPose(const std::string& name)
{
    if (name == "approach") return createPose(0.399367,0,0.510952,0.923956,-0.3825,0,0);
    if (name == "grasp") return createPose(0.399367,0,0.417898,0.923956,-0.3825,0,0);
    if (name == "target") return createPose(0.050255,-0.301609,0.579429,-0.426951,0.904186,0.01149,-0.00536161);
    if (name == "drop") return createPose(0.050255,-0.301609,0.486498,-0.426951,0.904186,0.01149,-0.00536161);
    if (name == "initial") return get_current_pose();
    return geometry_msgs::msg::Pose();
}

bool PandaRobot::loadSRDF(const std::string &path)
{
    std::ifstream srdf_file(path);
    if (!srdf_file.is_open())
    {
        return false;
    }

    std::string srdf_content((std::istreambuf_iterator<char>(srdf_file)),
                                std::istreambuf_iterator<char>());
    srdf_file.close();

    node_->declare_parameter("robot_description_semantic", rclcpp::ParameterValue(srdf_content));
    return true;
}

bool PandaRobot::loadKinematics(const std::string &path)
{
    std::ifstream kin_file(path);
    if (!kin_file.is_open())
        return false;

    std::string kin_content((std::istreambuf_iterator<char>(kin_file)),
                            std::istreambuf_iterator<char>());
    node_->declare_parameter("robot_description_kinematics", rclcpp::ParameterValue(kin_content));
    return true;
}

geometry_msgs::msg::Pose PandaRobot::createPose(double x, double y, double z, double ox, double oy, double oz, double w)
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = w;
    pose.orientation.x = ox;
    pose.orientation.y = oy;
    pose.orientation.z = oz;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
}

geometry_msgs::msg::Pose PandaRobot::get_current_pose()
{
    //std::string ee_link = move_group_interface.getEndEffectorLink();
    geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
    
    RCLCPP_INFO(node_->get_logger(), "End Effector Pose:");
    RCLCPP_INFO(node_->get_logger(), "Position -> x: %f, y: %f, z: %f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "Orientation -> yaw: %f, pitch: %f, roll: %f, quat: %f",
                current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                current_pose.orientation.w);
    
    return current_pose;
}

std::vector<double> PandaRobot::get_joint_values()
{
    return move_group_interface_->getCurrentJointValues();
}

bool PandaRobot::planAndExecute(const geometry_msgs::msg::Pose &pose)
{
    move_group_interface_->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_interface_->plan(plan))
    {
        move_group_interface_->execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning to the target pose failed!");
        return false;
    }
}

bool PandaRobot::moveLinear(const geometry_msgs::msg::Pose &target_pose)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // step size for interpolation
    const double jump_threshold = 0.0;

    double fraction = move_group_interface_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.99)
    {
        RCLCPP_WARN(node_->get_logger(), "Only %.2f%% of the path was planned", fraction * 100.0);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    move_group_interface_->execute(plan);
    return true;
}

bool PandaRobot::moveJoints(const std::vector<double> &joint_values)
{
    move_group_interface_->setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_interface_->plan(plan))
    {
        move_group_interface_->execute(plan);
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint-space planning failed!");
        return false;
    }
}

void PandaRobot::addCollisionObjects(const std::string& type, const std::string& id, double x,double y, double z, double ox, double oy, double oz, double w)
{
    //Create an instance of object
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::CollisionObject object;
    Eigen::Vector3d scale(0.2, 0.2, 0.2);

    if(type == "default")
    {
        object.header.frame_id = move_group_interface_->getPlanningFrame();
        object.id = id;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.05,0.05,0.05};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = w;
        box_pose.orientation.x = ox;
        box_pose.orientation.y = oy;
        box_pose.orientation.z = oz;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(box_pose);
        object.operation = object.ADD;

        collision_objects.push_back(object);
    }

    else
    {
        object.header.frame_id = move_group_interface_->getPlanningFrame();
        object.id = id; 

        shapes::ShapeMsg shape_msg_variant;
        shapes::Mesh* m = shapes::createMeshFromResource("package://binbot/objects/table.dae",scale);
        shapes::constructMsgFromShape(m, shape_msg_variant);
        shape_msgs::msg::Mesh primitive = boost::get<shape_msgs::msg::Mesh>(shape_msg_variant);

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = w;
        table_pose.orientation.x = ox;
        table_pose.orientation.y = oy;
        table_pose.orientation.z = oz;
        table_pose.position.x = x;
        table_pose.position.y = y;
        table_pose.position.z = z;

        object.meshes.push_back(primitive);
        object.mesh_poses.push_back(table_pose);
        object.operation = object.ADD;

        collision_objects.push_back(object);
    }

    planning_scene_interface_-> applyCollisionObjects(collision_objects);
    RCLCPP_INFO(node_->get_logger(), "Added collison object: id=%s", id.c_str());
}

void PandaRobot::detachObject(const std::string &id)
{
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    moveit_msgs::msg::CollisionObject object;
    detach_object.object.id = id;
    detach_object.link_name = "panda_link8";
    //object.header.frame_id = move_group_interface_->getPlanningFrame();
    detach_object.object.operation = detach_object.object.REMOVE;

    planning_scene_interface_->applyAttachedCollisionObject(detach_object);

    RCLCPP_INFO(node_->get_logger(),"Detached object: %s", id.c_str());

    object.id  = id;
    object.header.frame_id = move_group_interface_->getPlanningFrame();
    object.operation = object.REMOVE;
    planning_scene_interface_->applyCollisionObject(object);

    RCLCPP_INFO(node_->get_logger(),"Removed object: %s", id.c_str());
}

void PandaRobot::attachObject(const std::string& id, const std::string& link_name)
{
    moveit_msgs::msg::AttachedCollisionObject object;
    object.link_name = link_name;
    object.object.id = id;
    object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};

    planning_scene_interface_->applyAttachedCollisionObject(object);

    RCLCPP_INFO(node_->get_logger(), "Attached object '%s' to link '%s'", id.c_str(), link_name.c_str());
}

void PandaRobot::gripperOpen(double j1, double j2)
{
    std::vector<double>open = {j1,j2};

    gripper_->setJointValueTarget(open);
    gripper_->move();
}

void PandaRobot::gripperClose()
{
    std::vector<double>closed = {0.0,0.0};

    gripper_->setJointValueTarget(closed);
    gripper_->move();
}
