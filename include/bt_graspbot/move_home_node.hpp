#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "panda_robot_interface.hpp"

class MoveHomeNode : public BT::SyncActionNode 
{
public:
  MoveHomeNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() 
  { 
    return {};
  }

  BT::NodeStatus tick() override 
  {
    std::vector<double> home_joints = {0,-0.785,0,-2.356,0,1.571,0.785};
    return robot_->moveJoints(home_joints) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static void setRobot(PandaRobot *robot) 
  { 
    robot_ = robot; 
  }

private:
    static PandaRobot *robot_;
};