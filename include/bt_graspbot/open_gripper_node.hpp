#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "panda_robot_interface.hpp"

// --- OpenGripper Node ---
class OpenGripperNode : public BT::SyncActionNode 
{
public:
  OpenGripperNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() 
  { 
    return {};
  }

  BT::NodeStatus tick() override 
  {
    robot_->gripperOpen();
    return BT::NodeStatus::SUCCESS;
  }

  static void setRobot(PandaRobot *robot) 
  { 
    robot_ = robot; 
  }

private:
    static PandaRobot *robot_;
};