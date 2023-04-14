/**
 * @file test_traj_with_mpdm.cc
 * @author HKUST Aerial Robotics Group (lzhangbz@ust.hk)
 * @brief
 * @version 0.1
 * @date 2020-09-21
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "behavior_planner/behavior_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "plan_utils/traj_server_ros.h"
#include "semantic_map_manager/visualizer.h"


DECLARE_BACKWARD;
double traj_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

plan_utils::TrajPlannerServer* p_traj_server_{nullptr};
planning::BehaviorPlannerServer* p_bp_server_{nullptr};
semantic_map_manager::Visualizer *p_smm_vis_{nullptr};

int BehaviorUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  p_smm_vis_->VisualizeDataWithStamp(ros::Time::now(),smm);
  p_smm_vis_->SendTfWithStamp(ros::Time::now(), smm);
  p_traj_server_->SetEgoState(smm.ego_vehicle().state());
  if (p_traj_server_) p_traj_server_->PushSemanticMap(smm);
  return 0;
}

int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  // ROS_WARN("SemanticMapUpdateCallback ");
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");
  //send control law by the topic "ctrl/agent_0"
  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  std::string agent_config_path;
  p_smm_vis_   = new semantic_map_manager::Visualizer(nh, ego_id);

  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param %s", agent_config_path.c_str());
    assert(false);
  }

  std::string traj_config_path;
  if (!nh.getParam("traj_config_path", traj_config_path)) {
    ROS_ERROR("Failed to get param traj_config_path %s",
              traj_config_path.c_str());
    assert(false);
  }

  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);//store the state information
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);//connect with ros
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback); //connenction with phy simulator




  
  double desired_vel;
  nh.param("desired_vel", desired_vel, 6.0);
  // Declare bp
  p_bp_server_ = new planning::BehaviorPlannerServer(nh, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
  p_bp_server_->set_autonomous_level(3);
  p_bp_server_->enable_hmi_interface();

  p_traj_server_ =  new plan_utils::TrajPlannerServer(nh, traj_planner_work_rate, ego_id);

  p_traj_server_->Init(traj_config_path);
  p_bp_server_->Init();
  smm_ros_adapter.Init();

  p_bp_server_->Start();
  p_traj_server_->Start();

  // TicToc timer;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
