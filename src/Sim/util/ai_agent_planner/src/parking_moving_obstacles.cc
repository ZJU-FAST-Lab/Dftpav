#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>
#include <random>

#include "behavior_planner/behavior_server_ros.h"
#include "common/basics/tic_toc.h"
#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "sensor_msgs/Joy.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/encoder.h"

DECLARE_BACKWARD;
double fs_work_rate = 25.0;

int vehicle_num = -1;
 
ros::Publisher surround_traj_pub;


struct moving_vehicle
{
  Eigen::Vector2d center;
  double radius;
  double inityaw;
  double desiredV;
  double desiredOmg;
  ros::Publisher ctrl_signal_pub;
};


double pre_time;
double deltatime;
double inittime;
std::vector<moving_vehicle> vehicles;

common::State getState(double now_t, double init_t, int id){
  //
  
  double px = vehicles[id].radius * cos(vehicles[id].inityaw + (now_t- init_t)*vehicles[id].desiredOmg) + vehicles[id].center[0];
  double py = vehicles[id].radius * sin(vehicles[id].inityaw + (now_t- init_t)*vehicles[id].desiredOmg) + vehicles[id].center[1];
  double cur_yaw = vehicles[id].inityaw + (now_t- init_t)*vehicles[id].desiredOmg + M_PI_2;
  common::State state;
  state.acceleration = 0.0;
  state.angle = cur_yaw;
  state.curvature  = 1.0/vehicles[id].radius;
  state.time_stamp = ros::Time::now().toSec();
  state.vec_position << px,py;
  state.velocity = vehicles[id].desiredV;
  return state;
}

void PublishControl() {
  double cur_time = ros::Time::now().toSec();
  for(int i = 0; i < vehicle_num; i++){
    common::State state = getState(cur_time,inittime,i);
    common::VehicleControlSignal ctrl(state);
    {
      vehicle_msgs::ControlSignal ctrl_msg;
      vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
        ctrl, ros::Time::now(), std::string("map"), &ctrl_msg);
      vehicles[i].ctrl_signal_pub.publish(ctrl_msg);
    }
  }
  
  
  //publish sur trajs;
  visualization_msgs::MarkerArray surtrajs;
  for(int i = 0; i < vehicle_num; i++){
    visualization_msgs::Marker traj;
    traj.action = visualization_msgs::Marker::ADD;
    traj.id = i;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.pose.orientation.w = 1.00;
    traj.color.r = 0.00;
    traj.color.g = 0.00;
    traj.color.b = 0.00;
    traj.color.a = 1.00;
    traj.scale.x = 0.1;
    traj.scale.y = 0.1;
    traj.scale.z = 0.1;
    traj.header.frame_id = "map";
    traj.header.stamp =ros::Time().fromSec(cur_time);
    geometry_msgs::Point point1;
    for(double t = 0.0; t<=pre_time; t += deltatime){
      common::State state = getState(cur_time + t, inittime, i);
      point1.x = state.vec_position[0];
      point1.y = state.vec_position[1];
      point1.z = state.angle;
      traj.points.push_back(point1);
    }
  surtrajs.markers.push_back(traj);

  }


  




  surround_traj_pub.publish(surtrajs);

}

int main(int argc, char** argv) {
  //min turn radius = 1 / 0.35;
  ros::init(argc, argv, "moving_obs");
  ros::NodeHandle nh("~");

  nh.getParam("vehicle_num",vehicle_num);
  nh.getParam("predictiontime",pre_time);
  nh.getParam("deltatime",deltatime);
  vehicles.resize(vehicle_num);
  std::cout<<"vehicle_num : "<<vehicle_num<<std::endl;
  for(int i = 0; i < vehicle_num; i++){
    std::string id = std::string("car") +  std::to_string(i+1);
    nh.getParam(id+std::string("/centerpx"),vehicles[i].center[0]);
    nh.getParam(id+std::string("/centerpy"),vehicles[i].center[1]);
    nh.getParam(id+std::string("/desired_vel"),vehicles[i].desiredV);
    nh.getParam(id+std::string("/radius"),vehicles[i].radius);
    nh.getParam(id+std::string("/inityaw"),vehicles[i].inityaw);
    vehicles[i].desiredOmg = vehicles[i].desiredV / vehicles[i].radius;
    vehicles[i].ctrl_signal_pub = nh.advertise<vehicle_msgs::ControlSignal>(std::string("/ctrl/agent_")+std::to_string(i+1), 10);
  }
  inittime = ros::Time::now().toSec();
  surround_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis/parking_surround_trajs", 10);
  ros::Rate rate(fs_work_rate);
  while (ros::ok()) {
    ros::spinOnce();
    PublishControl();
    rate.sleep();
  }
  return 0;
}
