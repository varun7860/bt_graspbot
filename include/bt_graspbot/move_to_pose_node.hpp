#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "panda_robot_interface.hpp"

// --- MoveToPose Node ---
class MoveToPoseNode : public BT::SyncActionNode 
{
    public:
      MoveToPoseNode(const std::string &name, const BT::NodeConfiguration &config)
          : BT::SyncActionNode(name, config) {}
    
      static BT::PortsList providedPorts() 
      {
        return {BT::InputPort<std::string>("pose")};
      }
    
      BT::NodeStatus tick() override
      {
        auto pose_name = getInput<std::string>("pose");
        if (!pose_name)
          return BT::NodeStatus::FAILURE;
        auto pose = robot_->getPredefinedPose(pose_name.value());
        return robot_->moveLinear(pose) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
    
      static void setRobot(PandaRobot *robot) 
      { 
        robot_ = robot; 
      }
    
    private:
      static PandaRobot *robot_;
};