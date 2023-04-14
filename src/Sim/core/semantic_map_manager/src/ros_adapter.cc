#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

void RosAdapter::Init() {
  // communicate with phy simulator
  {
    // arena_info_sub_ =
    //     nh_.subscribe("arena_info", 2, &RosAdapter::ArenaInfoCallback, this,
    //                   ros::TransportHints().tcpNoDelay());
    arena_info_static_sub_ = nh_.subscribe(
        "arena_info_static", 2, &RosAdapter::ArenaInfoStaticCallback, this,
        ros::TransportHints().tcpNoDelay());
    arena_info_dynamic_sub_ = nh_.subscribe(
        "arena_info_dynamic", 2, &RosAdapter::ArenaInfoDynamicCallback, this,
        ros::TransportHints().tcpNoDelay());

    isparking = nh_.getParam("isparking",isparking);
    if(isparking){
      sur_traj_sub_ = nh_.subscribe("/vis/parking_surround_trajs", 1, &RosAdapter::DyObsCallback, this,ros::TransportHints().tcpNoDelay());
      nh_.getParam("vehicle_num",vehicle_num);
      nh_.getParam("deltatime",deltatime);
      desiredVs.resize(vehicle_num); radiuss.resize(vehicle_num);
      for(int i = 0; i < vehicle_num; i++){
        std::string id = std::string("car") +  std::to_string(i+1);
        nh_.getParam(id+std::string("/desired_vel"),desiredVs[i]);
        nh_.getParam(id+std::string("/radius"),radiuss[i]);
      } 
    }

  }
}
void RosAdapter::DyObsCallback(const visualization_msgs::MarkerArray& msg){
  std::vector<std::vector<common::State>> sur_trajs;
  int idx = 0;
  for(auto trajmsg : msg.markers){
    std::vector<common::State> traj;
    double cur_time = trajmsg.header.stamp.toSec();
    int kdx = 0;
    for(auto point : trajmsg.points){
      common::State state;
      state.vec_position[0] = point.x;
      state.vec_position[1] = point.y;
      state.angle = point.z;
      state.curvature = 1.0/radiuss[idx];
      state.velocity = desiredVs[idx];
      state.acceleration = 0.0; 
      state.time_stamp = cur_time + kdx * deltatime;
      traj.push_back(state);
      kdx++;
    }
    sur_trajs.push_back(traj);
    idx++;
  }
  p_smm_->set_sur_points(sur_trajs);
}
// ! DEPRECATED (@lu.zhang)
void RosAdapter::ArenaInfoCallback(
    const vehicle_msgs::ArenaInfo::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                           obstacle_set_);
  //feed the simulator data to the smm
  if (has_callback_binded_) {
    private_callback_fn_(*p_smm_);
  }
}

void RosAdapter::ArenaInfoStaticCallback(
    const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  get_arena_info_static_ = true;
}

void RosAdapter::ArenaInfoDynamicCallback(
    const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(
      *msg, &time_stamp, &vehicle_set_);

  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                             obstacle_set_);
    if (has_callback_binded_) {
      
      private_callback_fn_(*p_smm_);

      
    }
  }
}

void RosAdapter::BindMapUpdateCallback(
    std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);

  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager