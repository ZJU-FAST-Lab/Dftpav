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
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/point_types.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>


#include <plan_manage/traj_manager.h>


// int BehaviorUpdateCallback(
//     const semantic_map_manager::SemanticMapManager& smm) {
//   if (p_traj_server_) p_traj_server_->PushSemanticMap(smm);
//   return 0;
// }
ros::Publisher  odom_pub;
ros::Publisher  pcl_pub,pcl_pub2;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;


int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
 
  common::Vehicle ego;
  ego = smm.ego_vehicle();
  common::State ego_state  = ego.state();
  
  double angular_Rate = tan(ego_state.steer)*ego_state.velocity/ego.param().wheel_base();

  std::set<std::array<decimal_t, 2>> obs_grids;
  obs_grids = smm.obstacle_grids();

  cloudMap.points.clear();
 



  common::GridMapND<uint8_t, 2> grid_map;
  grid_map = smm.obstacle_map();
  for(int i = 0;i <grid_map.dims_size(0); i++){
    for(int j = 0; j<grid_map.dims_size(1); j++){
      // grid_map.
      bool isocc = false;
      grid_map.CheckIfEqualUsingCoordinate(std::array<int,2>{i,j},80,&isocc);
      if(isocc){
        std::array<decimal_t, 2> pos;
        grid_map.GetGlobalPositionUsingCoordinate(std::array<int,2>{i,j},&pos);
        pcl::PointXYZ pt;
        pt.x = pos[0];
        pt.y = pos[1];
        pt.z = 0.0;
        cloudMap.points.push_back(pt); 
      }
    }
  }
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.stamp = ros::Time::now();
  globalMap_pcd.header.frame_id = "map";
  pcl_pub2.publish(globalMap_pcd);
  //
  //next should transform these raw data to odom tf and pcl
  //tf已经发了 接下来发pcl


  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id ="/odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = ego_state.vec_position[0];
  odom_msg.pose.pose.position.y = ego_state.vec_position[1];
  odom_msg.pose.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, ego_state.angle);
  tf2::convert(q, odom_msg.pose.pose.orientation);
  odom_msg.twist.twist.linear.x = ego_state.velocity;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = angular_Rate;
  odom_pub.publish(odom_msg);
  //pcl
  //set the frame is as "map" topic:/pcl data_type: pointcloud2
  //    *(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
  //static obstacle
  cloudMap.points.clear();
  for(const auto obs : obs_grids){
    pcl::PointXYZ pt;
    pt.x = obs[0];
    pt.y = obs[1];
    pt.z = 0.0;
    cloudMap.points.push_back(pt); 
  }
  //dynamic obstacle
  

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.stamp = ros::Time::now();
  globalMap_pcd.header.frame_id = "map";
  pcl_pub.publish(globalMap_pcd);



  //publish /tf
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = odom_msg.pose.pose.position.x; 
  transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q1;
  q1.setRPY(0, 0, ego_state.angle);
  transformStamped.transform.rotation.x = q1.x();
  transformStamped.transform.rotation.y = q1.y();
  transformStamped.transform.rotation.z = q1.z();
  transformStamped.transform.rotation.w = q1.w();
  br.sendTransform(transformStamped);
  
  
  // ego_state.acceleration
  /* for (auto it = obstacle_grids_.begin(); it != obstacle_grids_.end(); ++it)
      {
        Vec2f pt((*it)[0], (*it)[1]);
        global_point_vec.push_back(pt);
      }*/

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");
  
//   //send control law by the topic "ctrl/agent_0"
  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  std::string agent_config_path;
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

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
  pcl_pub   = nh.advertise<sensor_msgs::PointCloud2>("/pcl", 1);
  pcl_pub2   = nh.advertise<sensor_msgs::PointCloud2>("/pcl2", 1);





  smm_ros_adapter.Init();


  // TicToc timer;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
