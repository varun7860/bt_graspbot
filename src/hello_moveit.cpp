#include <memory>
#include <string>
#include <fstream>
#include <streambuf>
#include <vector>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.hpp>
#include <boost/variant.hpp>

class PandaRobot
{
public:
    PandaRobot()
    {
        // Initialize ROS before any node or logger creation

        node_ = std::make_shared<rclcpp::Node>(
            "hello_moveit",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

        logger_ = node_->get_logger();  

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
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    ~PandaRobot()
    {
        rclcpp::shutdown();
        if (spinner_.joinable())
            spinner_.join();
    }

    void run(std::string move_type = "default")
    {
        //Add objects in the scene
        addCollisionObjects("default","box");
        //addCollisionObjects("meshes","table");

        geometry_msgs::msg::Pose approach_pose;
        geometry_msgs::msg::Pose target_pose;
        geometry_msgs::msg::Pose current_pose;
        std::vector<double> joint_pose;
        std::vector<double> home_joints = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};

        if(move_type == "default")
        {
            joint_pose = {1.57, 0, 0, 0, 0, 1.571, 0};
            std::this_thread::sleep_for(std::chrono::seconds(1));
            moveJoints(joint_pose);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            moveJoints(home_joints);
        }

        else if(move_type == "linear")
        {
            current_pose = get_current_pose();
            target_pose = createPose(current_pose.position.x,current_pose.position.y,current_pose.position.z+0.2,
                                     current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                                     current_pose.orientation.w);
            moveLinear(target_pose);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            current_pose = get_current_pose();
            target_pose = createPose(current_pose.position.x,current_pose.position.y,current_pose.position.z-0.2,
                current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                current_pose.orientation.w);
            moveLinear(target_pose);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

private:
    bool loadSRDF(const std::string &path)
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

    bool loadKinematics(const std::string &path)
    {
        std::ifstream kin_file(path);
        if (!kin_file.is_open())
            return false;

        std::string kin_content((std::istreambuf_iterator<char>(kin_file)),
                                std::istreambuf_iterator<char>());
        node_->declare_parameter("robot_description_kinematics", rclcpp::ParameterValue(kin_content));
        return true;
    }

    geometry_msgs::msg::Pose createPose(double x=0, double y=0, double z=0, double ox = 0, double oy = 0, double oz = 0, double w = 1)
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

    geometry_msgs::msg::Pose get_current_pose()
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

    std::vector<double> get_joint_values()
    {
        return move_group_interface_->getCurrentJointValues();
    }

    bool planAndExecute(const geometry_msgs::msg::Pose &pose)
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

    bool moveLinear(const geometry_msgs::msg::Pose &target_pose)
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

    bool moveJoints(const std::vector<double> &joint_values)
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

    void addCollisionObjects(std::string type="default", std::string id="box", double x =0.4,double y=0.0, double z = 0.3, double ox=0.0, double oy=0.0, double oz=0.0, double w = 1.0)
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

    void removeCollisionObject(std::string id="box")
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = id;
        object.header.frame_id = move_group_interface_->getPlanningFrame();
        object.operation = object.REMOVE;

        planning_scene_interface_->applyCollisionObjects({object});

        RCLCPP_INFO(node_->get_logger(),"Removed collision object: %s", id.c_str());
    }

    void attachObject(std::string& id, std::string& link_name)
    {
        moveit_msgs::msg::AttachedCollisionObject object;
        object.link_name = link_name;
        object.object.id = id;
        object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        object.touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};

        planning_scene_interface_->applyAttachedCollisionObject(object);

        RCLCPP_INFO(node_->get_logger(), "Attached object '%s' to link '%s'", id.c_str(), link_name.c_str());
    }

    //rclcpp::Logger logger_;
    rclcpp::Logger logger_{rclcpp::get_logger("default_logger")};  // Temporary init
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    std::thread spinner_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    PandaRobot robot;
    robot.run("linear");

    return 0;
}

/*int main(int argc, char *argv[])
{
    //Initialize ROS and Create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    //Add executors
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    //Create ROS Logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    //Read SRDF File
    std::ifstream srdf_file("/home/varun/ros2_ws/src/panda_moveit_config/config/panda.srdf");
    if(!srdf_file.is_open())
    {
       RCLCPP_ERROR(logger, "Failed to open SRDF File");
       return 1;
    }

    std::string srdf_content((std::istreambuf_iterator<char>(srdf_file)),
                              std::istreambuf_iterator<char>());


    srdf_file.close();

    //Declare ROS Parameter
    node->declare_parameter("robot_description_semantic",rclcpp::ParameterValue(srdf_content));

    //Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node,"panda_arm");
    rclcpp::sleep_for(std::chrono::seconds(2));
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    //Get Current Pose
    std::string ee_link = move_group_interface.getEndEffectorLink();
    geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
    RCLCPP_INFO(node->get_logger(), "Using end-effector: %s", ee_link.c_str());
    RCLCPP_INFO(node->get_logger(), "End Effector Pose:");
    RCLCPP_INFO(node->get_logger(), "Position -> x: %f, y: %f, z: %f",
                 current_pose.position.x, current_pose.position.y, current_pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Orientation -> yaw: %f, pitch: %f, roll: %f, quat: %f",
                 current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                 current_pose.orientation.w);


    //Set Approach Point
    auto const approach = []
    {
        geometry_msgs::msg::Pose app;
        app.orientation.w =1.0;
        //app.orientation.x = 0.70711;
        //app.orientation.y = -3.17467e-11;
        //app.orientation.z = 0.707107;
        app.position.x = 0.28;
        app.position.y = -0.2;
        app.position.z = 0.5;
        return app;
    }();

    //Set a Target Pose
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0; //1.0
        //msg.orientation.x = 0.70706;
        //msg.orientation.y = -1.27293e-05;
        //msg.orientation.z = 0.707154;
        msg.position.x = 0.28; //0.28
        msg.position.y = -0.2; //-0.2
        msg.position.z = 0.6; // 0.5
        return msg;
    }();

    move_group_interface.setPoseTarget(approach);
    auto const [done,app_plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan app;
        auto const ok = static_cast<bool>(move_group_interface.plan(app));
        return std::make_pair(ok,app);
    }();

    if(done)
    {
        move_group_interface.execute(app_plan);
        //saveTrajectory(app_plan.trajectory_, "/home/varun/ros2_ws/src/binbot/trajectories/app_trajectory.yaml");
    }

    else
    {
        RCLCPP_ERROR(logger," Moving to approach point planning failed!");
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    move_group_interface.setPoseTarget(target_pose);

    //Create a plan to that target pose
    auto const [success,plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok,msg);
    }();

    //Execute the plan
    if(success)
    {
        move_group_interface.execute(plan);
        //saveTrajectory(plan.trajectory_,"/home/varun/ros2_ws/src/binbot/trajectories/trajectory.yaml");
    }

    else
    {
        RCLCPP_ERROR(logger," Moving to final point planning failed!");
    }

    //Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}*/
