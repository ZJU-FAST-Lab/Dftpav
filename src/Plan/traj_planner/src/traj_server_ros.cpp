/**
 * @file ssc_server.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc planner server
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

/**
 * @file traj_server_ros.cpp revised version
 * @brief for minco planner server
 * @version 0.2
 * @date 2021-04
 * @copyright Copyright (c) 2021
 */

#include "plan_utils/traj_server_ros.h"

namespace plan_utils
{

  TrajPlannerServer::TrajPlannerServer(ros::NodeHandle nh, int ego_id)
      : nh_(nh), work_rate_(20.0), ego_id_(ego_id) 
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    // p_smm_vis_   = new semantic_map_manager::Visualizer(nh, ego_id);
    p_traj_vis_  = new TrajVisualizer(nh, ego_id);

    nh.getParam("enable_urban", enable_urban_);
    p_planner_   = new plan_manage::TrajPlanner(nh, ego_id, enable_urban_);
    
   

  }
  //use this!
  TrajPlannerServer::TrajPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id)
      : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) 
  {
    p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
        config_.kInputBufferSize);
    // p_smm_vis_   = new semantic_map_manager::Visualizer(nh, ego_id);
    p_traj_vis_  = new TrajVisualizer(nh, ego_id);

    nh.getParam("enable_urban", enable_urban_);
    nh.param("isparking", isparking,true);
    p_planner_   = new plan_manage::TrajPlanner(nh, ego_id, enable_urban_);

    parking_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &TrajPlannerServer::ParkingCallback, this);

    if(!isparking){
      trajplan = std::bind(&plan_manage::TrajPlanner::RunOnce,p_planner_);
    }
    else{
      trajplan = std::bind(&plan_manage::TrajPlanner::RunOnceParking,p_planner_);
    }
    

  }

  void TrajPlannerServer::PushSemanticMap(const SemanticMapManager& smm) 
  {
    if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
  }


  void TrajPlannerServer::Init(const std::string& config_path) 
  {

    p_planner_->Init(config_path);
    nh_.param("use_sim_state", use_sim_state_, true);
    std::string traj_topic = std::string("/vis/agent_") +
                           std::to_string(ego_id_) +
                           std::string("/minco/exec_traj");

    // to publish command to sim
    ctrl_signal_pub_ = nh_.advertise<vehicle_msgs::ControlSignal>("ctrl", 20);
    executing_traj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(traj_topic, 1);
    debug_pub =  nh_.advertise<std_msgs::Bool>("/DEBUG", 1);
    end_pt_.setZero();


  }

  //call the main thread
  void TrajPlannerServer::Start() 
  {
    // printf("\033[34mTrajPlannerServer]TrajPlannerServer::Start\n\033[0m");


    if (is_replan_on_) {
      return;
    }
    is_replan_on_ = true;  // only call once
    //@yuwei : set map interface to planner
    p_planner_->set_map_interface(&map_adapter_);
    printf("[TrajPlannerServer]Planner server started.\n");

    std::thread(&TrajPlannerServer::MainThread, this).detach();
    std::thread(&TrajPlannerServer::PublishThread, this).detach();
  }
  void TrajPlannerServer::PublishThread(){
    using namespace std::chrono;
    system_clock::time_point current_start_time{system_clock::now()};
    system_clock::time_point next_start_time{current_start_time};
    const milliseconds interval{static_cast<int>(10)}; // 50ms
    while (true) {
      current_start_time = system_clock::now();
      next_start_time = current_start_time + interval;
      PublishData();
     
      std::this_thread::sleep_until(next_start_time);
    }
  }
  void TrajPlannerServer::MainThread() {
    using namespace std::chrono;
    system_clock::time_point current_start_time{system_clock::now()};
    system_clock::time_point next_start_time{current_start_time};
    const milliseconds interval{static_cast<int>(1000.0 / work_rate_)}; // 50ms
    while (true) {
      current_start_time = system_clock::now();
      next_start_time = current_start_time + interval;
      PlanCycleCallback();
      std::this_thread::sleep_until(next_start_time);
    }
  }


  void TrajPlannerServer::PlanCycleCallback() {
    if (!is_replan_on_) {
      return;
    }
    while (p_input_smm_buff_->try_dequeue(last_smm_)) {
      is_map_updated_ = true;
    }
    /*find the latest smm
    if no smm, the process will return */
    if (!is_map_updated_) {
      printf("No map is updated");
      return;
    }
    // Update map
    auto map_ptr =
        std::make_shared<semantic_map_manager::SemanticMapManager>(last_smm_);
    map_adapter_.set_map(map_ptr);  //=maptp, the mapinterface used in p_planner
    // PublishData();
    //vis smm, vis lasttraj, pub control law
    double current_time = ros::Time::now().toSec();
    if (executing_traj_!=nullptr && current_time > executing_traj_->at(final_traj_index_).end_time) {

      printf("[TrajPlannerServer]Mission complete.\n");
      m.lock();
      executing_traj_.release();
      m.unlock();
      p_planner_->release_initial_state();
      return;
    }
     std_msgs::Bool debug;
      debug.data = 1;
      debug_pub.publish(debug);
    if (enable_urban_){
      // the requency is high enough so that we will not consider event triggered replanning
      static int fsm_num = 0;
      fsm_num++;
      if (fsm_num > 100000||CheckReplan()) {
        if (next_traj_ == nullptr) {
          Replan();
          // return;
        }
        if (next_traj_ !=nullptr) {
          m.lock();
          executing_traj_ = std::move(next_traj_);
          next_traj_.release();
          fsm_num = 0;
          final_traj_index_ = executing_traj_->size() - 1;
          exe_traj_index_ = 0; 
          m.unlock();
          
          p_traj_vis_->displayPolyTraj(executing_traj_);
          Display();
    // ros::Duration(100.0).sleep();

          return;
        }
      }

    }
    else{
      CheckReplan();
    }
  }


  void TrajPlannerServer::PublishData() {
    using common::VisualizationUtil;
    m.lock();
    ros::Time current_time = ros::Time::now();
    if(!is_map_updated_) 
    {
      m.unlock();
      return;}
    // smm visualization
    if(is_map_updated_)
    {
      // p_smm_vis_->VisualizeDataWithStamp(current_time, last_smm_);
      // p_smm_vis_->SendTfWithStamp(current_time, last_smm_);
    }

    if (executing_traj_ == nullptr ||exe_traj_index_ > final_traj_index_ ||
     executing_traj_->at(exe_traj_index_).duration < 1e-5) {
       common::State state = ego_state;
       state.velocity = 0.0;
       state.steer = 0.0;
       state.acceleration = 0.0;
       state.curvature = 0.0;
       double t = current_time.toSec();
       state.time_stamp = t;
       if(ctrl_state_hist_.size()==0){
           ctrl_state_hist_.push_back(ego_state);
       }
       else{
          FilterSingularityState(ctrl_state_hist_, &state);
          ctrl_state_hist_.push_back(state);
          if (ctrl_state_hist_.size() > 100) ctrl_state_hist_.erase(ctrl_state_hist_.begin());
       }
       
     
      
       vehicle_msgs::ControlSignal ctrl_msg;
       vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
              common::VehicleControlSignal(state), ros::Time(t),
              std::string("map"), &ctrl_msg);
       ctrl_signal_pub_.publish(ctrl_msg);
       m.unlock();
       return;
     }

    // trajectory feedback 
    {
      if (use_sim_state_) {
        common::State state;
        
        double t = current_time.toSec();

        state.time_stamp = t;

        if (executing_traj_->at(exe_traj_index_).end_time <= t){
          exe_traj_index_ += 1;
        }
        if (exe_traj_index_ > final_traj_index_){ 
          m.unlock();return;}
        // std::cout << " exe_traj_index_ " << exe_traj_index_ <<  std::endl;
        // std::cout << " final_traj_index_ " << final_traj_index_ <<  std::endl;
        executing_traj_->at(exe_traj_index_).traj.GetState(t-executing_traj_->at(exe_traj_index_).start_time, &state);
        // std::cout<<"state: "<<state.vec_position.transpose()<<"\n"; 
        FilterSingularityState(ctrl_state_hist_, &state);
        ctrl_state_hist_.push_back(state);
        if (ctrl_state_hist_.size() > 100) ctrl_state_hist_.erase(ctrl_state_hist_.begin());
        
        vehicle_msgs::ControlSignal ctrl_msg;
        vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
                common::VehicleControlSignal(state), ros::Time(t),
                std::string("map"), &ctrl_msg);

        //hzchzc

        /*
        vec_position: 
    x: 9.96869580051
    y: 58.806571087
    z: 0.0
  angle: 0.777951436039
  curvature: 0.000542476085517
  velocity: 9.70671770974
  acceleration: 0.443944701338
  steer: 0.00154605561188
        */

        // ctrl_msg.state.acceleration = 0;
        // ctrl_msg.state.curvature = 0;
        // ctrl_msg.state.acceleration = 0;
        // ctrl_msg.state.steer = 0;


        // std::cout<<"ctrl_msg: "<<ctrl_msg.state.vec_position.x<<" "<<ctrl_msg.state.vec_position.y<<std::endl;
        ctrl_signal_pub_.publish(ctrl_msg);
      }
    }
    
    // trajectory visualization
    /*{
      {
        auto color = common::cmap["magenta"];
        if (require_intervention_signal_) color = common::cmap["yellow"];
        visualization_msgs::MarkerArray traj_mk_arr;
        if (require_intervention_signal_) {
          visualization_msgs::Marker traj_status;
          common::State state_begin;
          state_begin.time_stamp = executing_traj_->at(exe_traj_index_).start_time;  // define the timestamp before call get state
          executing_traj_->at(exe_traj_index_).traj.GetState(0.0, &state_begin);
          Vec3f pos = Vec3f(state_begin.vec_position[0],
                            state_begin.vec_position[1], 5.0);
          common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
              pos, std::string("Intervention Needed!"), common::cmap["red"],
              Vec3f(5.0, 5.0, 5.0), 0, &traj_status);
          traj_mk_arr.markers.push_back(traj_status);
        }
        int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
        common::VisualizationUtil::FillHeaderIdInMarkerArray(
            current_time, std::string("map"), last_trajmk_cnt_,
            &traj_mk_arr);
        last_trajmk_cnt_ = num_traj_mks;
        executing_traj_vis_pub_.publish(traj_mk_arr);
      }
    }*/
    m.unlock();
  }

  // minco display
  void TrajPlannerServer::Display(){  
    // std::cout  <<  "\033[34m[TrajPlannerServer]TrajPlannerServer::Display. \033[0m" << std::endl; 
 
    p_traj_vis_->displayPolyH(p_planner_->display_hPolys());
    p_traj_vis_->displayObs(p_planner_->display_vec_obs());
    p_traj_vis_->displayKinoPath(p_planner_->display_kino_path());
    p_traj_vis_->displayInnerList(p_planner_->display_InnterPs(), 0);
    p_traj_vis_->displayFitSurTrajs(p_planner_->display_surround_trajs());



    
  }

  ErrorType TrajPlannerServer::FilterSingularityState(
      const vec_E<common::State>& hist, common::State* filter_state) {
    if (hist.empty()) {
      return kWrongStatus;
    }
    double duration = filter_state->time_stamp - hist.back().time_stamp;
    double max_steer = M_PI / 4.0;
    double singular_velocity = kBigEPS;
    double max_orientation_rate = tan(max_steer) / 2.85 * singular_velocity;
    double max_orientation_change = max_orientation_rate * duration;

    if (fabs(filter_state->velocity) < singular_velocity && 
        fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
            max_orientation_change) {
      // printf(
      //     "[TrajPlannerServer]Detect singularity velocity %lf angle (%lf, %lf).\n",
      //     filter_state->velocity, hist.back().angle, filter_state->angle);
      filter_state->angle = hist.back().angle;
      // printf("[TrajPlannerServer]Filter angle to %lf.\n", hist.back().angle);
    }
    return kSuccess;
  }


  bool TrajPlannerServer::CheckReplan(){
      //return 1: replan 0: not
      if(executing_traj_==nullptr) return true;
      bool is_near = false;
      bool is_collision = false;
      bool is_close_turnPoint = false;
      double cur_time = ros::Time::now().toSec();
      Eigen::Vector2d localTarget;
      localTarget = executing_traj_->back().traj.getPos(executing_traj_->back().duration);
      double totaltrajTime = 0.0;
      for(int i = 0; i<executing_traj_->size(); i++){
          totaltrajTime += executing_traj_->at(i).duration;
      }
      //is close to turnPoint?
      if(exe_traj_index_ == final_traj_index_) is_close_turnPoint = false;
      else{
        if((executing_traj_->at(exe_traj_index_).end_time - cur_time)<2.5)
          is_close_turnPoint = true;
      }
      //is near?
      if((executing_traj_->back().end_time - cur_time)<2*totaltrajTime / 3.0) is_near = true;
      else is_near = false;
      if(is_near && !is_close_turnPoint&&(localTarget-end_pt_.head(2)).norm()>0.1){
        return true;
      }
      //collision-check
      for(int i = 0; i < executing_traj_->size(); i++){
        for(double t = 0.0; t < executing_traj_->at(i).duration; t+=0.05){
          Eigen::Vector2d pos;
          Eigen::Vector3d state;
          double yaw;
          pos = executing_traj_->at(i).traj.getPos(t);
          yaw = executing_traj_->at(i).traj.getAngle(t);
          state << pos[0],pos[1],yaw;
          map_adapter_.CheckIfCollisionUsingPosAndYaw(vp_,state,&is_collision);
          if(is_collision)  return true;   
        }

      }
      // executing_traj_/
      
      return false;
      // map_adapter_.CheckIfCollisionUsingPosAndYaw   
  }



  ErrorType TrajPlannerServer::Replan() {
    if (!is_replan_on_) return kWrongStatus;
    time_profile_tool_.tic();
    // define the timestamp before call get state
    common::State desired_state;
    if(map_adapter_.GetEgoState(&desired_state)!=kSuccess){
      return kWrongStatus;
    }
    desired_state.time_stamp = ros::Time::now().toSec()+Budget;
    if(executing_traj_ ==nullptr){
      p_planner_->set_initial_state(desired_state);
      if (trajplan()!= kSuccess) {
        Display();
        return kWrongStatus;
      }
        //wait 
      double curt = ros::Time::now().toSec();
      if(0){
      // if(curt>desired_state.time_stamp){
        ROS_WARN("exceed time budget");
        return kWrongStatus;
      }
      else{
        while(true){
        curt = ros::Time::now().toSec();
        if(curt>=desired_state.time_stamp){
          break;
        } 
       }
      }
      desired_state_hist_.clear();
      desired_state_hist_.push_back(last_smm_.ego_vehicle().state());
      ctrl_state_hist_.clear();
      ctrl_state_hist_.push_back(last_smm_.ego_vehicle().state());
      //record the hist state and control 
    }
    else if(executing_traj_->at(exe_traj_index_).duration >= 0.0){

      //locate the piece idx
      int pidx = exe_traj_index_;
      while(true){
        if(desired_state.time_stamp<=executing_traj_->at(pidx).start_time+executing_traj_->at(pidx).duration){
          break;
        }
        else{
          pidx++;
          if(pidx>=executing_traj_->size()){
            pidx--;
            break;
          }
        }
      }
      double t = desired_state.time_stamp - executing_traj_->at(pidx).start_time; 
      executing_traj_->at(pidx).traj.GetState(t, &desired_state);
      FilterSingularityState(desired_state_hist_, &desired_state);
      desired_state_hist_.push_back(desired_state);
      if (desired_state_hist_.size() > 100)
        desired_state_hist_.erase(desired_state_hist_.begin());
      p_planner_->set_initial_state(desired_state);
      if (trajplan() != kSuccess) {
        Display();      
        return kWrongStatus;
      }
      printf("\033[32m[TrajPlannerServer]Traj planner succeed in %lf ms.\n\033[0m",
      time_profile_tool_.toc());

      //wait 
      double curt = ros::Time::now().toSec();
      if(curt>desired_state.time_stamp){
        ROS_WARN("exceed time budget");
        return kWrongStatus;
      }
      else{
        while(true){
        curt = ros::Time::now().toSec();
        if(curt>=desired_state.time_stamp){
          break;
        } 
       }
      }
      


    }
    else{
      return kWrongStatus;
    }


    next_traj_ = std::move(p_planner_->trajectory());
    if(next_traj_->at(0).duration <= 1e-5){
      next_traj_.release();
      return kWrongStatus;
    }
    return kSuccess;
  }


  void TrajPlannerServer::ParkingCallback(const geometry_msgs::PoseStamped &msg)
  {
    std::cout << "Triggered parking mode!" << std::endl;
    end_pt_ <<  msg.pose.position.x, msg.pose.position.y,
                tf::getYaw(msg.pose.orientation), 1.0e-2;
    std::cout<<"end_pt: "<<end_pt_.transpose()<<std::endl;
  end_pt_ << -20.2606 ,-7.24047 ,-3.13672 ,    0.01;
   

    have_parking_target_ = true;
    p_planner_->setParkingEnd(end_pt_);
  }



}  // namespace planning