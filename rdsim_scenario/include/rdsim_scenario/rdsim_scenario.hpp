#ifndef __RDSIM_SCENARIO_MANAGER_H__
#define __RDSIM_SCENARIO_MANAGER_H__

#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rdsim_scenario/scenario/delivery_scenario.hpp"

namespace rdsim_scenario {
class ScenarioManager : public nav2_util::LifecycleNode {
public:
  ScenarioManager();

protected:
  







  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  




  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  




  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  




  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  




  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
  BT::Blackboard::Ptr m_blackboard;
  std::unique_ptr<rdsim_scenario::DeliveryScenario> m_delivery_scenario;
  ScenarioMuxer m_muxer;
};

} 

#endif 