#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "panda_robot_interface.hpp"

class WaitNode : public BT::SyncActionNode 
{
public:
    WaitNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() 
  { 
    return {BT::InputPort<int>("seconds")};
  }

  BT::NodeStatus tick() override 
  {
    auto seconds = getInput<int>("seconds");
    if(!seconds)
    {
        return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::seconds(seconds.value()));
    return BT::NodeStatus::SUCCESS;
  }
};