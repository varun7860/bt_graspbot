#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "panda_robot_interface.hpp"

#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "panda_robot_interface.hpp"

class DetachObjectNode: public BT::SyncActionNode
{
    public:
        DetachObjectNode(const std::string &name, const BT::NodeConfiguration &config): BT::SyncActionNode(name,config){}
        
        static BT::PortsList providedPorts() {return {};}

        BT::NodeStatus tick() override
        {
            robot_->detachObject("box");
            return BT::NodeStatus::SUCCESS;
        }

        static void setRobot(PandaRobot *robot)
        {
            robot_ = robot;
        }

    private:
        static PandaRobot *robot_;
};