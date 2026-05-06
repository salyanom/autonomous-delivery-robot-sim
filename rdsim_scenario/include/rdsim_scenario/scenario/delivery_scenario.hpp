#ifndef __RDSIM_DELIVERY_SCENARIO_H__
#define __RDSIM_DELIVERY_SCENARIO_H__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rdsim_interfaces/action/delivery.hpp"
#include "rdsim_scenario/scenario/scenario.hpp"
#include <memory>
#include <string>
#include <vector>

namespace rdsim_scenario {




class DeliveryScenario : public rdsim_scenario::Scenario<rdsim_interfaces::action::Delivery> {
public:
  using ActionT = rdsim_interfaces::action::Delivery;

  


  DeliveryScenario() : Scenario() {}

  




  bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  


  bool cleanup() override;

  



  std::string getName() override { return std::string("delivery_scenario"); }

  




  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  






  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  



  void onLoop() override;

  


  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  






  void goalCompleted(typename ActionT::Result::SharedPtr result,
                     const nav2_behavior_tree::BtStatus final_bt_status) override;

  



  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string start_goal_blackboard_id_;
  std::string end_goal_blackboard_id_;
  std::string path_blackboard_id_;
};

} 
#endif 