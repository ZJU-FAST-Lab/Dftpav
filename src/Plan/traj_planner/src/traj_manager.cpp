#include "plan_manage/traj_manager.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>
#include <cmath>
#include <fstream>
namespace plan_manage
{

  ErrorType TrajPlanner::Init(const std::string config_path) {
    ReadConfig(config_path);

    // * Planner config
    printf("\nTrajPlanner Config:\n");
    printf(" -- weight_proximity: %lf\n", cfg_.planner_cfg().weight_proximity());

    LOG(INFO) << "[PolyTrajManager]TrajPlanner Config:";
    LOG(INFO) << "[PolyTrajManager] -- low spd threshold: "
              << cfg_.planner_cfg().low_speed_threshold();
    LOG(INFO) << "[PolyTrajManager] -- weight_proximity: "
              << cfg_.planner_cfg().weight_proximity();

    traj_piece_duration_   = cfg_.opt_cfg().traj_piece_duration();
    traj_res = cfg_.opt_cfg().traj_resolution();
    dense_traj_res = cfg_.opt_cfg().des_traj_resolution();
    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh_, cfg_);
    //ploy_traj_opt_->setSurroundTrajs(&traj_container_.surround_traj);
    
    /*  kino a* intial  */
    kino_path_finder_.reset(new path_searching::KinoAstar);
    kino_path_finder_->init(cfg_,nh_);



    /*teb local planner hzc*/
    /*
    blp_loader_ = std::make_shared<pluginlib::ClassLoader<nav_core::BaseLocalPlanner>>("nav_core", "nav_core::BaseLocalPlanner");
    buffer  = new tf2_ros::Buffer(ros::Duration(10));
    tf = new tf2_ros::TransformListener(*buffer);
    costmap_ros_ =  new costmap_2d::Costmap2DROS("local_costmap",*buffer);
    costmap_ros_->pause();
    tc_ = blp_loader_->createInstance(std::string("teb_local_planner/TebLocalPlannerROS"));
    tc_->initialize(blp_loader_->getName(std::string("teb_local_planner/TebLocalPlannerROS")),buffer,costmap_ros_);
    costmap_ros_->start();*/



    KinopathPub = nh_.advertise<nav_msgs::Path>("/KinoPathMsg", 1);
    DenseKinopathPub = nh_.advertise<nav_msgs::Path>("/vis_dense_kino_traj", 1);
    DensePathVisPub = nh_.advertise<geometry_msgs::PoseArray>("/libai/vis_front_end",1);
    DensePathPub = nh_.advertise<nav_msgs::Path>("/libai/front_end",1);

    /*debug*/

    Debugtraj0Pub = nh_.advertise<nav_msgs::Path>("/debug/vis_traj_0", 1);
    Debugtraj1Pub = nh_.advertise<nav_msgs::Path>("/debug/vis_traj_1", 1);
    DebugCorridorPub = nh_.advertise<visualization_msgs::Marker>("/debug/corridor", 1);  
    DebugtrajPub =   nh_.advertise<visualization_msgs::Marker>("/debug/traj", 1);  


    return kSuccess;
  }


  // use kinodynamic a* to generate a path
  ErrorType TrajPlanner::getKinoPath(Eigen::Vector4d &end_state){

    Eigen::Vector4d start_state;
    Eigen::Vector2d init_ctrl;

    start_state << head_state_.vec_position, head_state_.angle, head_state_.velocity;    
    init_ctrl << head_state_.steer, head_state_.acceleration;
    //steer and acc
    // std::cout << "[kino replan]: start_state " << start_state  << std::endl;

    kino_path_finder_->reset();
    double searcht1 = ros::Time::now().toSec();

    // start_state << -26.3909,  20.7379  ,1.57702 ,       0;      end_state << -45.4141,   10.3171 ,0.0600779     , 0.01;
    std::cout<<"start state: "<<start_state.transpose()<<" end_state: "<<end_state.transpose()<<std::endl;
    std::cout<<"init ctrl: "<<init_ctrl.transpose()<<std::endl;
    int status = kino_path_finder_->search(start_state, init_ctrl, end_state, true);
    double searcht2 = ros::Time::now().toSec();
    std::cout<<"search time: "<<(searcht2-searcht1)<<std::endl;
    if (status == path_searching::KinoAstar::NO_PATH)
    {
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

      // retry searching with discontinuous initial state
      kino_path_finder_->reset();
      status = kino_path_finder_->search(start_state, init_ctrl, end_state, false);
      if (status == path_searching::KinoAstar::NO_PATH)
      {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return kWrongStatus;
      }
      else
      {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    }
    else
    {
      std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    kino_path_finder_->getKinoNode(kino_trajs_);
    
    // ROS_WARN("hzc debug kinodynamic search");
    // std::cout << " kino_trajs_.size() :" <<   kino_trajs_.size()  << std::endl;
    // std::cout << " kino_trajs_.at(0).start_state  :" <<  kino_trajs_.at(0).start_state << std::endl;
    // std::cout << " kino_trajs_.at(0).final_state :" <<   kino_trajs_.at(0).final_state << std::endl;
    //ros::shutdown();
    return kSuccess;
  }

  ErrorType TrajPlanner::ReadConfig(const std::string config_path) {
    printf("\n[EudmPlanner] Loading Traj_planner planner config\n");
    using namespace google::protobuf;
    int fd = open(config_path.c_str(), O_RDONLY);
    io::FileInputStream fstream(fd);
    TextFormat::Parse(&fstream, &cfg_);
    if (!cfg_.IsInitialized()) {
      LOG(ERROR) << "failed to parse config from " << config_path;
      assert(false);
    }
    return kSuccess;
  }

  ErrorType TrajPlanner::set_initial_state(const State& state) {
    has_init_state_ = true;
    head_state_ = state;
    return kSuccess;
  }


  Eigen::MatrixXd TrajPlanner::state_to_flat_output(const State& state) {

    Eigen::MatrixXd flatOutput(2, 3);

    double vel = state.velocity, angle = state.angle; // vel < 0 is ok
    double acc = state.acceleration;

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
              sin(angle),   cos(angle);

    if (vel == 0){
      vel = 1e-5;
    }

    flatOutput << state.vec_position, init_R*Eigen::Vector2d(vel, 0.0), 
                  init_R*Eigen::Vector2d(acc, state.curvature * std::pow(vel, 2));

    return flatOutput;
  }

  ErrorType TrajPlanner::RunOnceParking(){
    if(!have_parking_target) return kWrongStatus;
    have_parking_target = false;      
    Eigen::Vector4d parking_end = end_pt;
    is_init = false;
    stamp_ = map_itf_->GetTimeStamp();
    static TicToc Traj_planner_timer;
    Traj_planner_timer.tic();
    static TicToc timer_prepare;
    timer_prepare.tic();
    // get ego vehicle
    if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
      LOG(ERROR) << "[PolyTrajManager]fail to get ego vehicle info.";
      return kWrongStatus;
    }
    if (!has_init_state_) { head_state_ = ego_vehicle_.state();
      std::cout<<"head state:"<<head_state_.velocity<<"\n";
      is_init = true;
    }
    has_init_state_ = false;
    if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
      LOG(ERROR) << "[PolyTrajManager]fail to get obstacle map.";
      return kWrongStatus;
    }
    // should call before run MINCO
    // point cloud
    // will be used in corridor generation,plc is stored in the vec_obs
    if (UpdateObsGrids() != kSuccess)
    {
      LOG(ERROR) << "[PolyTrajManager]no obs points.\n";
      return kWrongStatus;
    }
    // static int tri_flag = 0;
    // if(tri_flag) return kWrongStatus;
    Eigen::Vector4d start_state;
    start_state << head_state_.vec_position, head_state_.angle, head_state_.velocity; 
    if ((parking_end - start_state).head(2).norm() < 1.0){
      ROS_WARN("arrive!");
      have_parking_target = false; //hzc
      return kWrongStatus;
    }

    double frontendt1 = ros::Time::now().toSec();
    if (getKinoPath(parking_end) != kSuccess){
      LOG(ERROR) << "[PolyTrajManager Parking] fail to get the front-end.\n";
      return kWrongStatus;   
    }
    double frontendt2 = ros::Time::now().toSec();
    ROS_INFO_STREAM("front_end time is: "<<1000.0*(frontendt2-frontendt1)<<" ms");
    // ros::Duration(1000.0).sleep();

    // tri_flag =1;
    std::cout<<"traj segs num: "<<kino_trajs_.size()<<"\n";
    
    if (RunMINCOParking()!= kSuccess)
    {
      LOG(ERROR) << "[PolyTrajManager Parking] fail to optimize the trajectories.\n";
      return kWrongStatus;
    }
    else{
      ROS_INFO("parking trajectory generate!");
    }

    /*vis*/
    nav_msgs::Path vis_pathmsg;
    std::vector<Eigen::Vector4d> trajlist = kino_path_finder_->SamplePosList(200);
    for(auto pos : trajlist){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = pos[0];//x
      pose.pose.position.y = pos[1];//y
      pose.pose.position.z = 0.2;
      pose.header.stamp =  ros::Time().now();//time
      vis_pathmsg.poses.push_back(pose);
    }
    vis_pathmsg.header.frame_id = "map";
    DenseKinopathPub.publish(vis_pathmsg);
   return kSuccess;
   
  }

  ErrorType TrajPlanner::RunOnce() 
  {

    stamp_ = map_itf_->GetTimeStamp();
    Eigen::Vector4d parking_end = end_pt;
    is_init = false;


    static TicToc Traj_planner_timer;
    Traj_planner_timer.tic();

    static TicToc timer_prepare;
    timer_prepare.tic();

    // get ego vehicle
    if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
      LOG(ERROR) << "[PolyTrajManager]fail to get ego vehicle info.";
      return kWrongStatus;
    }

    if (!has_init_state_) { head_state_ = ego_vehicle_.state(); is_init = true;
    }

    has_init_state_ = false;

    // get obs grid map (may not use it)
    // not use the gridmap!!!!
    if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
      LOG(ERROR) << "[PolyTrajManager]fail to get obstacle map.";
      return kWrongStatus;
    }

    // should call before run MINCO
    // point cloud
    // will be used in corridor generation,plc is stored in the vec_obs
    if (UpdateObsGrids() != kSuccess)
    {
      LOG(ERROR) << "[PolyTrajManager]no obs points.\n";
      return kWrongStatus;
    }

    if(enable_urban_){

      // get ego vehicle reference lane
      if (map_itf_->GetLocalReferenceLane(&nav_lane_local_) != kSuccess) {
        LOG(ERROR) << "[PolyTrajManager]fail to find ego lane.";
        return kWrongStatus;
      }

      // get ego vehicle behavior
      if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
        LOG(ERROR) << "[PolyTrajManager]fail to get ego behavior.";
        return kWrongStatus;
      }//lat_behavior

      // get ego and other forward trajectories
      if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                          &surround_forward_trajs_) != kSuccess) {
        LOG(ERROR) << "[PolyTrajManager]fail to get forward trajectories.";
        return kWrongStatus;
      }//
      if (RunMINCO() != kSuccess)
      {
        // LOG(ERROR) << "[PolyTrajManager]fail to optimize the trajectories.\n";
        return kWrongStatus;
      }
      
      if (UpdateTrajectoryWithCurrentBehavior() != kSuccess) {
        // LOG(ERROR) << "[PolyTrajManager]fail: current behavior "
                  // << static_cast<int>(ego_behavior_) << " not valid.";
        // LOG(ERROR) << "[PolyTrajManager]fail: has "  << valid_behaviors_.size() << " behaviors.";
        return kWrongStatus;
      }
      
    }else{

      return kWrongStatus;

    }


    

    return kSuccess;
  }



  ErrorType TrajPlanner::RunMINCO(){
    
    // forward_trajs_ = n*11  (n=1,2,3,... the number of behaviors) 3
    
    // int num_behaviors = forward_behaviors_.size();
    // valid_behaviors_.clear();
    // ref_trajactory_list_.clear();
    // nav_msgs::Path debug_msg0,debug_msg1;
    // Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);
    // flat_headState  = state_to_flat_output(head_state_);
    // // p v a for x-y axis
    // // for each behavior, do minco optimization
    // for (int i = 0; i < num_behaviors; ++i) 
    // { 
   
    //   // get surround traj
    //   ConvertSurroundTraj(&surround_trajs, i);
    //   //dynamic obs processing
    //   ploy_traj_opt_->setSurroundTrajs(&surround_trajs);
    //   // find the nearest point to the head_state timeStamp
    //   int piece_nums;
    //   int start_idx;
    //   for(int k = 0; k < forward_trajs_[i].size(); k++){
    //     if(forward_trajs_[i][k].state().time_stamp > head_state_.time_stamp + 1.0e-2){
    //       start_idx = k;
    //       piece_nums = forward_trajs_[i].size() - k;
    //       break;
    //     }
    //   }
    //   State final_state_ = forward_trajs_[i].back().state();
    //   flat_finalState = state_to_flat_output(final_state_);
    //   Eigen::MatrixXd ego_innerPs(2, piece_nums-1);
    //   Eigen::VectorXd ego_piece_dur_vec(piece_nums);
    //   ego_piece_dur_vec[0] = forward_trajs_[i][start_idx].state().time_stamp - head_state_.time_stamp;

    //   for(int k = start_idx; k < forward_trajs_[i].size()-1; k++){
    //     State traj_state = forward_trajs_[i][k].state();
    //     ego_innerPs(0, k-start_idx) =  traj_state.vec_position[0];
    //     ego_innerPs(1, k-start_idx) =  traj_state.vec_position[1];
    //     ego_piece_dur_vec[k-start_idx+1] = forward_trajs_[i][k+1].state().time_stamp-traj_state.time_stamp;
    //   }
    //   plan_utils::MinJerkOpt initMJO;
    //   initMJO.reset(flat_headState, flat_finalState, piece_nums);
    //   initMJO.generate(ego_innerPs, ego_piece_dur_vec);
    //   plan_utils::Trajectory initTraj = initMJO.getTraj(1); //urban case are always forward moving
    //   double total_dur = initTraj.getTotalDuration();
    //   //behavior traj visualization
    //   {
    //     for(double t  = 0.0; t <= initTraj.getTotalDuration(); t+=0.05){
    //       geometry_msgs::PoseStamped pose;
    //       pose.header.frame_id = "map";
    //       pose.pose.position.x = initTraj.getPos(t)[0];//x
    //       pose.pose.position.y = initTraj.getPos(t)[1];//y
    //       pose.pose.position.z = 0.2;
    //       debug_msg0.poses.push_back(pose);
    //     }
    //     debug_msg0.header.frame_id = "map";
    //   }

    //   //re-paramalize the traj

    //   piece_nums = std::max(int(total_dur/traj_piece_duration_+0.5),2);
    //   ego_piece_dur_vec.resize(piece_nums);
    //   for(int i = 0; i < piece_nums; i++){
    //       ego_piece_dur_vec[i] = total_dur/piece_nums;
    //   }
    //   ego_innerPs.resize(2, piece_nums-1);
    //   std::vector<Eigen::Vector3d> statelist;
    //   double res_time = 0.0;
      
    //   for(int i = 0; i < piece_nums; i++ ){
    //     for(int k = 0; k <= traj_res; k++){
    //       double t = res_time + 1.0*k/traj_res*ego_piece_dur_vec[i];
    //       Eigen::Vector2d pos = initTraj.getPos(t);
    //       double yaw = initTraj.getAngle(t);
    //       statelist.emplace_back(pos[0],pos[1],yaw);
    //       if(k==traj_res && i!=piece_nums-1){
    //         ego_innerPs.col(i) = pos; 
    //       }
    //     } 
    //     res_time += ego_piece_dur_vec[i];
    //   }
    //   double t1 = ros::Time::now().toSec();
    //   // getRectangleConst(statelist);
    //   {
    //     hPolys_.clear();
    //     for(const auto state : statelist){
    //       Eigen::MatrixXd hPoly;
    //       Eigen::Vector2d pos = state.head(2);
    //       double yaw = state[2];
    //       std::vector<Eigen::Vector2d> add_vec_obs;
    //       Eigen::Matrix2d R;
    //       R << cos(yaw), -sin(yaw),
    //           sin(yaw),  cos(yaw);
    //       Eigen::Vector2d p;
    //       double d_x = 15.0;
    //       double d_y = 15.0;
    //       p = R*Eigen::Vector2d(d_x, d_y); //8 8
    //       add_vec_obs.push_back(pos + p);
    //       p = R*Eigen::Vector2d(d_x, -d_y);
    //       add_vec_obs.push_back(pos + p);
    //       p = R*Eigen::Vector2d(-d_x, d_y);
    //       add_vec_obs.push_back(pos + p);
    //       p = R*Eigen::Vector2d(-d_x, -d_y);
    //       add_vec_obs.push_back(pos + p);
    //       plan_utils::corridorBuilder2d(pos, 100.0, 15.0, 15.0, vec_obs_, add_vec_obs, hPoly);
    //       hPolys_.push_back(hPoly);
    //     }
    //   }
    //   double t2 = ros::Time::now().toSec();
    //   ROS_INFO_STREAM("corridor generation time: "<<(t2-t1)*1000.0<<" ms");
    //   initMJO.reset(flat_headState, flat_finalState, piece_nums);
    //   initMJO.generate(ego_innerPs, ego_piece_dur_vec);
    //   initTraj = initMJO.getTraj(1); //urban case are always forward moving
    //   // initial guess visualization
    //   {
    //     for(double t  = 0.0; t <= initTraj.getTotalDuration(); t+=0.05){
    //       geometry_msgs::PoseStamped pose;
    //       pose.header.frame_id = "map";
    //       pose.pose.position.x = initTraj.getPos(t)[0];//x
    //       pose.pose.position.y = initTraj.getPos(t)[1];//y
    //       pose.pose.position.z = 0.2;
    //       debug_msg1.poses.push_back(pose);
    //     }
    //     debug_msg1.header.frame_id = "map";
    //   }

    //   bool flag_success = false;
    //   // Init planning
    //   if(!checkShapeInside(hPolys_.back(), initTraj.getBoundPts(initTraj.getTotalDuration()))){
    //     ROS_WARN("end_pt is not in corridor");
    //     // printLateralBehavior(forward_behaviors_[i]);
    //   }
    //   if(!checkShapeInside(hPolys_.front(), initTraj.getBoundPts(0))){
    //     ROS_WARN("start_pt is not in the corridor");
    //     // printLateralBehavior(forward_behaviors_[i]);
    //   }
    //   std::vector<Eigen::MatrixXd> flat_headStateContainer;
    //   std::vector<Eigen::MatrixXd> flat_endStateContainer;
    //   std::vector<Eigen::MatrixXd> egoPtsContainer;
    //   std::vector<Eigen::VectorXd> initTContainer;
    //   std::vector<std::vector<Eigen::MatrixXd>> hPoly_container;
    //   std::vector<int> singuls;
    //   flat_headStateContainer.push_back(flat_headState);
    //   flat_endStateContainer.push_back(flat_finalState);
    //   egoPtsContainer.push_back(ego_innerPs);
    //   initTContainer.push_back(ego_piece_dur_vec);
    //   hPoly_container.push_back(hPolys_);
    //   singuls.push_back(1);
    //   // std::cout<<"head_state: \n"<<flat_headState<<std::endl;
    //   // std::cout<<"end_state: \n"<<flat_finalState<<std::endl;

    //   // std::cout<<"piecenum: "<<piece_nums<<std::endl;
    //   // std::cout<<"ego_innerPs:\n"<<ego_innerPs<<std::endl;
    //   // std::cout<<"ego_piece_dur_vec: "<<ego_piece_dur_vec.transpose()<<std::endl;

    //   flag_success = ploy_traj_opt_->OptimizeTrajectory(flat_headStateContainer, flat_endStateContainer, 
    //                                                     egoPtsContainer, initTContainer, 
    //                                                     hPoly_container, singuls,head_state_.time_stamp,1.0e-4); //1 means always forward in urban case
    //   if (flag_success)
    //   {
    //     valid_behaviors_.push_back(forward_behaviors_[i]);
    //     ref_trajactory_list_.push_back((*ploy_traj_opt_->getMinJerkOptPtr())[0].getTraj(1));
    //     //urban case are always forward moving
    //   }
    //   else{
    //     ROS_ERROR("Minco failed!");
    //     display_InnterPs_ = ego_innerPs;
    //     display_hPolys_ = hPolys_;      
    //   }
    //   if(forward_behaviors_[i] == ego_behavior_){
    //     display_InnterPs_ = ego_innerPs;
    //     display_hPolys_ = hPolys_;
    //   }
    // }
    // Debugtraj0Pub.publish(debug_msg0);
    // Debugtraj1Pub.publish(debug_msg1);
    return kSuccess;
  }


  ErrorType TrajPlanner::RunMINCOParking(){
    //TO DO
    
    
    traj_container_.clearSingul();
    Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);
    Eigen::VectorXd ego_piece_dur_vec;
    Eigen::MatrixXd ego_innerPs;
    ROS_WARN("begin to run minco");
    nav_msgs::Path debug_msg0,debug_msg1;
    display_hPolys_.clear();
    double worldtime =  head_state_.time_stamp;
    double basetime = 0.0;

    /*try to merge optimization process*/
    std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
    std::vector<int> singul_container;
    Eigen::VectorXd duration_container;
    std::vector<Eigen::MatrixXd> waypoints_container;
    std::vector<Eigen::MatrixXd> iniState_container,finState_container;
    duration_container.resize(kino_trajs_.size());

    for(unsigned int i = 0; i < kino_trajs_.size(); i++){
      double timePerPiece = traj_piece_duration_;
      plan_utils::FlatTrajData kino_traj = kino_trajs_.at(i);
      singul_container.push_back(kino_traj.singul);
      std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
      plan_utils::MinJerkOpt initMJO;
      plan_utils::Trajectory initTraj;
      int piece_nums;
      double initTotalduration = 0.0;
      for(const auto pt : pts){
        initTotalduration += pt[2];
      }
      piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
      timePerPiece = initTotalduration / piece_nums; 
      ego_piece_dur_vec.resize(piece_nums);
      ego_piece_dur_vec.setConstant(timePerPiece);
      duration_container[i] = timePerPiece * piece_nums;
      ego_innerPs.resize(2, piece_nums-1);
      std::vector<Eigen::Vector3d> statelist;
      double res_time = 0;
      for(int i = 0; i < piece_nums; i++ ){
        int resolution;
        if(i==0||i==piece_nums-1){
          resolution = dense_traj_res;
        }
        else{
          resolution = traj_res;
        }
        for(int k = 0; k <= resolution; k++){
          double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[i];
          Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
          statelist.push_back(pos);
          if(k==resolution && i!=piece_nums-1){
            ego_innerPs.col(i) = pos.head(2); 
          }
        } 
        res_time += ego_piece_dur_vec[i];
      }
      std::cout<<"s: "<<kino_traj.singul<<"\n";
      double tm1 = ros::Time::now().toSec();
      getRectangleConst(statelist);
      sfc_container.push_back(hPolys_);
      display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
      waypoints_container.push_back(ego_innerPs);
      iniState_container.push_back(kino_traj.start_state);
      finState_container.push_back(kino_traj.final_state);
      basetime += initTotalduration;
      //visualization
      initMJO.reset(ego_piece_dur_vec.size());
      initMJO.generate(ego_innerPs, timePerPiece,kino_traj.start_state, kino_traj.final_state);
      initTraj = initMJO.getTraj(kino_traj.singul);
      {
        for(double t  = 0.0; t <= initTraj.getTotalDuration(); t+=0.01){
          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = "map";
          pose.pose.position.x = initTraj.getPos(t)[0];//x
          pose.pose.position.y = initTraj.getPos(t)[1];//y
          pose.pose.position.z = 0.2;
          debug_msg1.poses.push_back(pose);
        }
        debug_msg1.header.frame_id = "map";
      }
      Debugtraj1Pub.publish(debug_msg1);  
    }
    Debugtraj1Pub.publish(debug_msg1);

    double t1= ros::Time::now().toSec();
    if(map_itf_->GetMovingObsTraj(&sur_discretePoints)!=kSuccess){
      return kWrongStatus;
    }
    ConverSurroundTrajFromPoints(sur_discretePoints,&surround_trajs);
    double t2 = ros::Time::now().toSec();
    std::cout<<"convert time: "<<(t2-t1)<<std::endl;
    ploy_traj_opt_->setSurroundTrajs(&surround_trajs);
    // ploy_traj_opt_->setSurroundTrajs(NULL);
    std::cout<<"try to optimize!\n";
    
    int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container, 
                                                        waypoints_container,duration_container, 
                                                        sfc_container,  singul_container,worldtime,0.0);
    std::cout<<"optimize ended!\n";
   


    if (flag_success)
    {
        std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
        for(unsigned int i = 0; i < kino_trajs_.size(); i++){
          traj_container_.addSingulTraj( (*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]), worldtime, ego_id_); // todo time
          std::cout<<"init duration: "<<duration_container[i]<<std::endl;
          std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
          std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
          std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost()<<std::endl;
          worldtime = traj_container_.singul_traj.back().end_time;
        }

        //debug
        double max_acc = -1, max_lat  = -1;
        for(unsigned int i = 0; i < kino_trajs_.size(); i++){
          plan_utils::Trajectory  traj = traj_container_.singul_traj[i].traj;
          for(double rest = 0.0; rest <= traj.getTotalDuration(); rest += 0.01){
            double lonacc = std::fabs(traj.getAcc(rest));
            double latacc = std::fabs(traj.getLatAcc(rest));
            max_acc = max(max_acc,lonacc);
            max_lat = max(max_lat,latacc);
          }
          std::cout <<"trajid: "<<i<<" duration: "<< traj.getTotalDuration()<<std::endl;
        }
        ROS_INFO_STREAM("max lon acc: " << max_acc <<" max lat acc: "<<max_lat);
    }
    else{
        ROS_ERROR("[PolyTrajManager] Planning fails! ");
        return kWrongStatus;
    }
    if(is_init){
      //reset the timeStamp
      for(auto & it:traj_container_.singul_traj ){
        it.start_time = ros::Time::now().toSec()-head_state_.time_stamp+it.start_time;
        it.end_time = ros::Time::now().toSec()-head_state_.time_stamp+it.end_time;
      }
    }

    return kSuccess;

  }


  void TrajPlanner::printLateralBehavior(LateralBehavior lateral_behavior){
    static std::string lateral_behavior_str[5] = {"kUndefined", "kLaneKeeping", "kLaneChangeLeft", "kLaneChangeRight"};
    std::cout  <<  "\033[34m[LateralBehavior]The behavior is: " << lateral_behavior_str[int(lateral_behavior)] <<  "\033[0m" << std::endl; 
  }

  ErrorType TrajPlanner::UpdateTrajectoryWithCurrentBehavior() {
    int num_valid_behaviors = static_cast<int>(valid_behaviors_.size());
    if (num_valid_behaviors < 1) {
      return kWrongStatus;
    }
    bool find_exact_match_behavior = false;
    int index = 0;
    for (int i = 0; i < num_valid_behaviors; i++) {
      //printLateralBehavior(forward_behaviors_[i]);
      //printLateralBehavior(ego_behavior_);
      if (valid_behaviors_[i] == ego_behavior_) {
        find_exact_match_behavior = true;
        index = i;
      }
    }
    bool find_candidate_behavior = false;
    LateralBehavior candidate_bahavior = common::LateralBehavior::kLaneKeeping;
    if (!find_exact_match_behavior) {
      ROS_WARN("Keeping Lane!");
      for (int i = 0; i < num_valid_behaviors; i++) {
        if (valid_behaviors_[i] == candidate_bahavior) {
          find_candidate_behavior = true;
          index = i;
        }
      }
    }
    if (!find_exact_match_behavior && !find_candidate_behavior)
      return kWrongStatus;

    printLateralBehavior(forward_behaviors_[index]);
    traj_container_.clearSingul();
    traj_container_.addSingulTraj(ref_trajactory_list_[index], head_state_.time_stamp, ego_id_); // todo time

    return kSuccess;
  }

  ErrorType TrajPlanner::ConvertSurroundTraj(plan_utils::SurroundTrajData* surround_trajs_ptr, int index){

    // * Surrounding vehicle trajs from MPDM
    Eigen::MatrixXd headState(2, 3);
    Eigen::MatrixXd flat_finalState(2, 3);
    Eigen::MatrixXd innerPs;
    Eigen::VectorXd piece_dur_vec;
    int i = 0;
    surround_trajs_ptr->resize(surround_forward_trajs_[index].size());
    //surrounding traj in one behavior 
    //resize: number of surrounded vehicle
    for (auto it = surround_forward_trajs_[index].begin();
              it != surround_forward_trajs_[index].end(); ++it)
    {
      plan_utils::MinJerkOpt surMJO;
      int piece_nums = (int)it->second.size() -1;
      Eigen::MatrixXd ref_wp_sur(2, piece_nums+1);
      for (unsigned int k = 0; k < it->second.size(); ++k)
      {
        State traj_state = it->second[k].state();  // states
        // vertices
        ref_wp_sur(0,k) =  traj_state.vec_position[0];
        ref_wp_sur(1,k) =  traj_state.vec_position[1];
      }

      State init_surround_state = it->second[0].state();
      State fin_surround_state  = it->second[piece_nums].state();

      double ts = (fin_surround_state.time_stamp - init_surround_state.time_stamp)/piece_nums;  // 4s 

      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);

      headState = state_to_flat_output(init_surround_state);
      flat_finalState = state_to_flat_output(fin_surround_state);

      /* generate the init of init trajectory */
      innerPs.resize(2, piece_nums-1);
      innerPs = ref_wp_sur.block(0, 1, 2, piece_nums-1);
      surMJO.reset(piece_nums);
      surMJO.generate(innerPs, ts,headState, flat_finalState);
      plan_utils::Trajectory sur_traj= surMJO.getTraj(1);
      // for other moving objects, we only account the forward
     
      surround_trajs_ptr->at(i).drone_id = it->first;
      surround_trajs_ptr->at(i).traj = sur_traj;
      surround_trajs_ptr->at(i).duration = sur_traj.getTotalDuration();
      surround_trajs_ptr->at(i).start_pos = sur_traj.getJuncPos(0);
      surround_trajs_ptr->at(i).start_time = init_surround_state.time_stamp;
      surround_trajs_ptr->at(i).end_time = init_surround_state.time_stamp + sur_traj.getTotalDuration();
      surround_trajs_ptr->at(i).init_angle = init_surround_state.angle;

      i += 1;
    }

    return kSuccess;
  }

  ErrorType TrajPlanner::ConverSurroundTrajFromPoints(std::vector<std::vector<common::State>> sur_trajs,plan_utils::SurroundTrajData* surround_trajs_ptr){
    // for other moving objects, we only account the forward
    Eigen::MatrixXd headState(2, 3);
    Eigen::MatrixXd flat_finalState(2, 3);
    Eigen::MatrixXd innerPs;
    Eigen::VectorXd piece_dur_vec;
    int idx = 0;
    surround_trajs_ptr->resize(sur_trajs.size());
    if(sur_trajs.size() == 0){
      return kWrongStatus;
    }
    for(auto sur_traj : sur_trajs){
      plan_utils::MinJerkOpt surMJO;
      int pieceNum = sur_traj.size()-1;
      innerPs.resize(2,pieceNum-1);
      piece_dur_vec.resize(pieceNum);
      
      for(int i = 1; i < sur_traj.size()-1; i++ ){
        innerPs.col(i-1) = sur_traj[i].vec_position;
        piece_dur_vec[i-1] = sur_traj[i].time_stamp-sur_traj[i-1].time_stamp;
      }
      piece_dur_vec[pieceNum-1] = sur_traj[sur_traj.size()-1].time_stamp-sur_traj[sur_traj.size()-2].time_stamp;
      common::State start_state, end_state;
      start_state = sur_traj.front();
      headState = state_to_flat_output(start_state);
      end_state = sur_traj.back();
      flat_finalState = state_to_flat_output(end_state);

      /* generate the init of init trajectory */
      
      surMJO.reset(pieceNum);
      surMJO.generate(innerPs, piece_dur_vec.sum() / pieceNum,headState, flat_finalState );
      plan_utils::Trajectory sur_fittraj= surMJO.getTraj(1);

     
      surround_trajs_ptr->at(idx).drone_id = idx+1;
      surround_trajs_ptr->at(idx).traj = sur_fittraj;
      surround_trajs_ptr->at(idx).duration = sur_fittraj.getTotalDuration();
      surround_trajs_ptr->at(idx).start_pos = sur_fittraj.getJuncPos(0);
      //debug
      surround_trajs_ptr->at(idx).start_time = sur_traj.front().time_stamp;
      surround_trajs_ptr->at(idx).end_time = surround_trajs_ptr->at(idx).start_time + sur_fittraj.getTotalDuration();
      surround_trajs_ptr->at(idx).init_angle = sur_traj.front().angle;
      idx += 1;
    }
    return kSuccess;
  }


  ErrorType TrajPlanner::set_map_interface(map_utils::TrajPlannerMapItf *map_itf) {
    if (map_itf == nullptr) return kIllegalInput;
    map_itf_ = map_itf;
    map_valid_ = true;

    kino_path_finder_->intialMap(map_itf);

    return kSuccess;
  }


  ErrorType TrajPlanner::getSikangConst(plan_utils::Trajectory &Traj, 
                                        Eigen::MatrixXd &innerPs,  
                                        Eigen::VectorXd &piece_dur_vec){

    hPolys_.clear();
    polys_.clear();
    Eigen::Vector2d first_point, second_point;

    double total_dur = Traj.getTotalDuration();
    int piece_nums = round(total_dur / traj_piece_duration_);
    if (piece_nums < 3) piece_nums = 3;


    double piece_dur = total_dur / (double)piece_nums;
    Eigen::MatrixXd hPoly;
    double interval_time = piece_dur;

    std::vector<Eigen::Vector2d> temp_inner;
    std::vector<double> temp_dur;
  
    double t = 0;
    while(t < total_dur+1e-6)
    {
      
      first_point = Traj.getPos(t);
      std::vector<Eigen::Vector2d> BoundPts = Traj.getBoundPts(t);
      
      if(t > 0) // t >= piece_dur
      {
        if (checkShapeInside(hPoly, BoundPts))
        {
          interval_time = piece_dur;
        }
        else
        {
          // find the closest point
          for (double k = t; k > t - piece_dur; k -= 0.1)
          {
            first_point = Traj.getPos(k);
            BoundPts = Traj.getBoundPts(k);
            if (checkShapeInside(hPoly, BoundPts))
            {

              interval_time = k-(t-piece_dur);
              break;
            }
          }
        }

        if (interval_time < 1e-3) { // need to regenerate a new polytope
          return kWrongStatus;
        }

        temp_inner.push_back(first_point);
        temp_dur.push_back(interval_time);
      }
      //hPoly?
      t += interval_time;

      second_point = Traj.getPos(t);

      vec_Vec2f seed_path;
      seed_path.push_back(first_point);
      seed_path.push_back(second_point);

      EllipsoidDecomp2D decomp_util;
      decomp_util.set_obs(vec_obs_);
      decomp_util.set_local_bbox(Vec2f(12, 12));
      decomp_util.dilate(seed_path);
      vec_E<Polyhedron2D> seed_poly = decomp_util.get_polyhedrons();
      polys_.push_back(seed_poly[0]);

      vec_E<Hyperplane2D> vs = seed_poly[0].hyperplanes();
      Eigen::MatrixXd hPoly(4, vs.size());
      // // normal in sikang's go outside
      for (unsigned int i = 0; i < vs.size(); i++){


        hPoly.col(i).tail<2>()  = vs[i].p_;
        hPoly.col(i).head<2>()  = vs[i].n_; 
      }
      hPolys_.push_back(hPoly);
    }

    int final_size = temp_inner.size()-1;
    innerPs.resize(2, final_size);
    piece_dur_vec.setZero(final_size+1);

    for(int j = 0 ; j < final_size ; j++ ){

      innerPs.col(j) =  temp_inner.at(j);
      piece_dur_vec(j) = temp_dur.at(j);

    }
    piece_dur_vec(final_size) = temp_dur.at(final_size);

    return kSuccess;
  }

  ErrorType TrajPlanner::getGalaxyConst(plan_utils::Trajectory &Traj, 
                                        Eigen::MatrixXd &innerPs,  
                                        Eigen::VectorXd &piece_dur_vec) 
  {
    /*
    ego_piece_dur_vec(0) = total_dur;
    // piece_dur
    ego_piece_dur_vec(1) = total_dur / (double)piece_nums;
    */
    //init_traj wp_list total_time total_time/piecenum
    hPolys_.clear();
    polys_.clear();
    Eigen::MatrixXd hPoly;

    Eigen::Vector2d first_point;
    double angle;
    // set as 15, 15 for normal urban traffic 
    double d_x = 15.0;
    double d_y = 15.0;
    // get the store values
    double total_dur = piece_dur_vec(0);
    double piece_dur = piece_dur_vec(1);


    // std::cout << " total_dur  " <<  total_dur  << std::endl;
    // std::cout << " piece_dur  " <<  piece_dur  << std::endl;


    double interval_time = piece_dur;

    std::vector<Eigen::Vector2d> temp_inner;
    std::vector<double> temp_dur;
  
    double t = 0;
    int index = 0;
    int piecenum = total_dur/piece_dur;

    /*std::vector<Eigen::Vector2d> wp_list;
    std::vector<double> angle_list;
    int piecenum = total_dur/piece_dur;
    for(int  i = 0;i < piecenum; i++){
      double t  =i * piece_dur;
      first_point = Traj.getPos(t);
      angle = Traj.getAngle(t);
      wp_list.push_back(first_point);
      angle_list.push_back(angle);
    } 
    for(int i = 0; i < piecenum; i++){
      angle = angle_list[i];
      first_point = wp_list[i];
      std::vector<Eigen::Vector2d> add_vec_obs;
      Eigen::Matrix2d R;
      R << cos(angle), -sin(angle),
           sin(angle),  cos(angle);
      Eigen::Vector2d p;
      p = R*Eigen::Vector2d(d_x, d_y); //8 8
      add_vec_obs.push_back(first_point + p);
      p = R*Eigen::Vector2d(d_x, -d_y);
      add_vec_obs.push_back(first_point + p);
      p = R*Eigen::Vector2d(-d_x, d_y);
      add_vec_obs.push_back(first_point + p);
      p = R*Eigen::Vector2d(-d_x, -d_y);
      add_vec_obs.push_back(first_point + p);
      plan_utils::corridorBuilder2d(first_point, 100.0, 15.0, 15.0, vec_obs_, add_vec_obs, hPoly);
      hPolys_.push_back(hPoly);
      //check if next point is in the hpoly
      
      if(i<piecenum-1){
        if(checkPosInside(hPoly,wp_list[i+1])){
          temp_inner.push_back(first_point);
          temp_dur.push_back(interval_time);
        }
        else{
          while(true){
            if (checkPosInside(hPoly, wp_list[i+1]))
            {
              break;
            }
            else
            {
              interval_time = 0.0;
              // find the closest point
              for (double k = total_dur; k > final_time; k -= 0.01)
              {
                first_point = Traj.getPos(k);
                BoundPts = Traj.getBoundPts(k);
                // if (checkShapeInside(hPoly, BoundPts))
                if (checkPosInside(hPoly, first_point))
                {
                  angle = Traj.getAngle(k);
                  interval_time = k-final_time;
                  break;
                }
              }
            }
            if (interval_time < 1e-3) { // need to regenerate a new polytope
              ROS_ERROR("corridor generate failed!");
              return kWrongStatus;
              d_x += 1;
              d_y += 1;
              hPolys_.pop_back();

            }else{
              temp_inner.push_back(first_point);
              temp_dur.push_back(interval_time);
              final_time = final_time+interval_time;
              std::vector<Eigen::Vector2d> add_vec_obs;
              Eigen::Matrix2d R;
              R << cos(angle), -sin(angle),
                  sin(angle),  cos(angle);

              Eigen::Vector2d p;

              p = R*Eigen::Vector2d(d_x, d_y); //8 8
              add_vec_obs.push_back(first_point + p);
              p = R*Eigen::Vector2d(d_x, -d_y);
              add_vec_obs.push_back(first_point + p);
              p = R*Eigen::Vector2d(-d_x, d_y);
              add_vec_obs.push_back(first_point + p);
              p = R*Eigen::Vector2d(-d_x, -d_y);
              add_vec_obs.push_back(first_point + p);
              plan_utils::corridorBuilder2d(first_point, 100.0, 15.0, 15.0, vec_obs_, add_vec_obs, hPoly);
              hPolys_.push_back(hPoly);
            }
          }
        }
      }
      else{
        //i = piecenum-1

      }
    }*/







    while(t <= total_dur - piece_dur)
    {
      first_point = Traj.getPos(t);
      angle = Traj.getAngle(t);
      std::vector<Eigen::Vector2d> BoundPts = Traj.getBoundPts(t);
      //Corner points of the vehicle

      if(t > 0) // t >= piece_dur from the second
      {
        // if (checkShapeInside(hPoly, BoundPts))
        if (checkPosInside(hPoly, first_point))
        {
          interval_time = piece_dur;
        }
        else
        {
          interval_time = 0.0;
          // find the closest point
          for (double k = t; k > t - piece_dur; k -= 0.05)
          {
            first_point = Traj.getPos(k);
            BoundPts = Traj.getBoundPts(k);
            // if (checkShapeInside(hPoly, BoundPts))
            if (checkPosInside(hPoly, first_point))
            {
              angle = Traj.getAngle(k);
              interval_time = k-(t-piece_dur);
              
              break;
            }
          }
        }

        if (interval_time < 1e-3) { // need to regenerate a new polytope
          
          std::cout<< "parking fail to generate the polytope..." << std::endl;
          return kWrongStatus;
          d_x += 1;
          d_y += 1;
          hPolys_.pop_back();

        }else{

          temp_inner.push_back(first_point);
          temp_dur.push_back(interval_time);

        }
        
      }

      t += interval_time;

      std::vector<Eigen::Vector2d> add_vec_obs;


      Eigen::Matrix2d R;
      R << cos(angle), -sin(angle),
           sin(angle),  cos(angle);

      Eigen::Vector2d p;

      p = R*Eigen::Vector2d(d_x, d_y); //8 8
      add_vec_obs.push_back(first_point + p);
      p = R*Eigen::Vector2d(d_x, -d_y);
      add_vec_obs.push_back(first_point + p);
      p = R*Eigen::Vector2d(-d_x, d_y);
      add_vec_obs.push_back(first_point + p);
      p = R*Eigen::Vector2d(-d_x, -d_y);
      add_vec_obs.push_back(first_point + p);
      
      plan_utils::corridorBuilder2d(first_point, 100.0, 15.0, 15.0, vec_obs_, add_vec_obs, hPoly);
      hPolys_.push_back(hPoly);
      index += 1;

    }
    double final_time = t-interval_time;
    //make sure the end point is corvered by the corridor
    while(true){
      first_point = Traj.getPos(total_dur);
      angle = Traj.getAngle(total_dur);
      std::vector<Eigen::Vector2d> BoundPts = Traj.getBoundPts(total_dur);
      //Corner points of the vehicle
      if (checkPosInside(hPoly, first_point))
      // if (checkShapeInside(hPoly, BoundPts))
      {
        // interval_time = piece_dur;
        break;
      }
      else
      {
        interval_time = 0.0;
        // find the closest point
        for (double k = total_dur; k > final_time; k -= 0.01)
        {
          first_point = Traj.getPos(k);
          BoundPts = Traj.getBoundPts(k);
          // if (checkShapeInside(hPoly, BoundPts))
          if (checkPosInside(hPoly, first_point))
          {
            angle = Traj.getAngle(k);
            interval_time = k-final_time;
            break;
          }
        }
      }
      if (interval_time < 1e-3) { // need to regenerate a new polytope
        ROS_ERROR("corridor generate failed!");
        return kWrongStatus;
        d_x += 1;
        d_y += 1;
        hPolys_.pop_back();

      }
      else{
        temp_inner.push_back(first_point);
        temp_dur.push_back(interval_time);
        final_time = final_time+interval_time;
        std::vector<Eigen::Vector2d> add_vec_obs;
        Eigen::Matrix2d R;
        R << cos(angle), -sin(angle),
            sin(angle),  cos(angle);

        Eigen::Vector2d p;

        p = R*Eigen::Vector2d(d_x, d_y); //8 8
        add_vec_obs.push_back(first_point + p);
        p = R*Eigen::Vector2d(d_x, -d_y);
        add_vec_obs.push_back(first_point + p);
        p = R*Eigen::Vector2d(-d_x, d_y);
        add_vec_obs.push_back(first_point + p);
        p = R*Eigen::Vector2d(-d_x, -d_y);
        add_vec_obs.push_back(first_point + p);
        plan_utils::corridorBuilder2d(first_point, 100.0, 15.0, 15.0, vec_obs_, add_vec_obs, hPoly);
        hPolys_.push_back(hPoly);
      }
    }
    double left_time = total_dur-final_time;
    if (left_time < 1e-3){ // reset the final state
      //bug
      int final_size = temp_inner.size()-1;
      innerPs.resize(2, final_size);
      piece_dur_vec.setZero(final_size+1);

      for(int j = 0 ; j < final_size ; j++ ){

        innerPs.col(j) =  temp_inner.at(j);
        piece_dur_vec(j) = temp_dur.at(j);

      }

      piece_dur_vec(final_size) = temp_dur.at(final_size);
      hPolys_.pop_back();
      ROS_ERROR("reset");
 
    }else{

      int final_size = temp_inner.size();
      innerPs.resize(2, final_size);
      piece_dur_vec.setZero(final_size+1);

      for(int j = 0 ; j < final_size ; j++ ){

        innerPs.col(j) =  temp_inner.at(j);
        piece_dur_vec(j) = temp_dur.at(j);

      }

      piece_dur_vec(final_size) = left_time;

    }
    return kSuccess;
  }
  ErrorType TrajPlanner::getRectangleConst(std::vector<Eigen::Vector3d> statelist){
    hPolys_.clear();
    GridMap2D grid_map;
    map_itf_->GetObstacleMap(&grid_map);    
    double resolution = grid_map.dims_resolution(0);
    double step = resolution * 1.0;
    double limitBound = 10.0;
    visualization_msgs::Marker  carMarkers;
    //generate a rectangle for this state px py yaw
    for(const auto state : statelist){
      //generate a hPoly
      Eigen::MatrixXd hPoly;
      hPoly.resize(4, 4);
      Eigen::Matrix<int,4,1> NotFinishTable = Eigen::Matrix<int,4,1>(1,1,1,1);      
      Eigen::Vector2d sourcePt = state.head(2);
      Eigen::Vector2d rawPt = sourcePt;
      double yaw = state[2];
      bool test = false;
      common::VehicleParam sourceVp,rawVp;
      Eigen::Matrix2d egoR;
      egoR << cos(yaw), -sin(yaw),
              sin(yaw), cos(yaw);
      common::VehicleParam vptest;
      map_itf_->CheckIfCollisionUsingPosAndYaw(vptest,state,&test);
      if(test){
        ROS_WARN(
          "init traj is not safe?"
        );
        std::cout<<"yaw: "<<yaw<<"\n";
        carMarkers.action = visualization_msgs::Marker::ADD;
        carMarkers.id = 0;
        carMarkers.type = visualization_msgs::Marker::LINE_LIST;
        carMarkers.pose.orientation.w = 1.00;
        carMarkers.ns = "libaicorridorF";
        carMarkers.color.r = 0.00;
        carMarkers.color.g = 0.00;
        carMarkers.color.b = 0.00;
        carMarkers.color.a = 1.00;
        carMarkers.scale.x = 0.05;
        carMarkers.header.frame_id = "map";
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;
        geometry_msgs::Point point3;
        geometry_msgs::Point point4;
        Eigen::Matrix2d R;
        R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
        Eigen::Vector2d offset1, tmp1;
        offset1 = R*Eigen::Vector2d(vptest.length()/2.0+vptest.d_cr(),vptest.width()/2.0);
        tmp1 = state.head(2)+offset1;
        point1.x = tmp1[0]; 
        point1.y = tmp1[1];
        point1.z = 0;
        Eigen::Vector2d offset2, tmp2;
        offset2 = R*Eigen::Vector2d(vptest.length()/2.0+vptest.d_cr(),-vptest.width()/2.0);
        tmp2 = state.head(2)+offset2;
        point2.x = tmp2[0]; 
        point2.y = tmp2[1];
        point2.z = 0;
        Eigen::Vector2d offset3, tmp3;
        offset3 = R*Eigen::Vector2d(-vptest.length()/2.0+vptest.d_cr(),-vptest.width()/2.0);
        tmp3 = state.head(2)+offset3;
        point3.x = tmp3[0]; 
        point3.y = tmp3[1];
        point3.z = 0;
        Eigen::Vector2d offset4, tmp4;
        offset4 = R*Eigen::Vector2d(-vptest.length()/2.0+vptest.d_cr(),vptest.width()/2.0);
        tmp4 = state.head(2)+offset4;
        point4.x = tmp4[0]; 
        point4.y = tmp4[1];
        point4.z = 0;
        carMarkers.points.push_back(point1);
        carMarkers.points.push_back(point2);
        carMarkers.points.push_back(point2);
        carMarkers.points.push_back(point3);
        carMarkers.points.push_back(point3);
        carMarkers.points.push_back(point4);
        carMarkers.points.push_back(point4);
        carMarkers.points.push_back(point1);

      }
       

      Eigen::Vector4d expandLength;
      expandLength << 0.0, 0.0, 0.0, 0.0;
      //dcr width length
      while(NotFinishTable.norm()>0){ 
        //+dy  +dx -dy -dx  
        for(int i = 0; i<4; i++){
            if(!NotFinishTable[i]) continue;
            //get the new source and vp
            Eigen::Vector2d NewsourcePt = sourcePt;
            common::VehicleParam NewsourceVp = sourceVp;
            Eigen::Vector2d point1,point2,newpoint1,newpoint2;

            bool isocc = false;
            switch (i)
            {
            //+dy
            case 0:
              point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);   
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);
              //1 new1 new1 new2 new2 2
              map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,step/2.0);
              NewsourceVp.set_width(NewsourceVp.width() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            //+dx
            case 1:
              point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);   
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              //1 new1 new1 new2 new2 2
              map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(step/2.0,0.0);
              NewsourceVp.set_length(NewsourceVp.length() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            //-dy
            case 2:
              point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);   
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);
              //1 new1 new1 new2 new2 2
              map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,-step/2.0);
              NewsourceVp.set_width(NewsourceVp.width() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            //-dx
            case 3:
              point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,sourceVp.width()/2.0);
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,-sourceVp.width()/2.0);
              //1 new1 new1 new2 new2 2
              map_itf_->CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              map_itf_->CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(-step/2.0,0.0);
              NewsourceVp.set_length(NewsourceVp.length() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            }   
        }
     }
      Eigen::Vector2d point1,norm1;
      point1 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],rawVp.width()/2.0+expandLength[0]);
      norm1 << -sin(yaw), cos(yaw);
      hPoly.col(0).head<2>() = norm1;
      hPoly.col(0).tail<2>() = point1;
      Eigen::Vector2d point2,norm2;
      // point2 = sourcePt+egoR*Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
      point2 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],-rawVp.width()/2.0-expandLength[2]);
      norm2 << cos(yaw), sin(yaw);
      hPoly.col(1).head<2>() = norm2;
      hPoly.col(1).tail<2>() = point2;
      Eigen::Vector2d point3,norm3;
      // point3 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
      point3 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],-rawVp.width()/2.0-expandLength[2]);
      norm3 << sin(yaw), -cos(yaw);
      hPoly.col(2).head<2>() = norm3;
      hPoly.col(2).tail<2>() = point3;
      Eigen::Vector2d point4,norm4;
      // point4 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
      point4 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],rawVp.width()/2.0+expandLength[0]);
      norm4 << -cos(yaw), -sin(yaw);
      hPoly.col(3).head<2>() = norm4;
      hPoly.col(3).tail<2>() = point4;
      hPolys_.push_back(hPoly);
  };
  DebugCorridorPub.publish(carMarkers);
  return kSuccess;
}

  bool TrajPlanner::checkShapeInside(Eigen::MatrixXd &hPoly, 
                                     std::vector<Eigen::Vector2d> vertices) 
  {

    Eigen::Vector2d p_, n_;
    double signed_dist;
    for (int i = 0; i < hPoly.cols(); i++)
    {
      p_ = hPoly.col(i).tail<2>();
      n_ = hPoly.col(i).head<2>();

      for (auto &pt : vertices)
      {
        signed_dist = n_.dot(pt- p_);
        if (signed_dist > 1e-6){ 
          return false;
        }
      }
    }

    return true; // inside

  }
  bool TrajPlanner::checkPosInside(Eigen::MatrixXd &hPoly,Eigen::Vector2d pos){
    Eigen::Vector2d p_, n_;
    double signed_dist;
    for (int i = 0; i < hPoly.cols(); i++)
    {
      p_ = hPoly.col(i).tail<2>();
      n_ = hPoly.col(i).head<2>();
      signed_dist = n_.dot(pos- p_);
      if (signed_dist > 1e-6){ 
        return false;
      }
    }
    return true; // inside
  }

  ErrorType TrajPlanner::UpdateObsGrids(){
    
    map_itf_->GetObstacleGrids(&obs_grids_);
    vec_obs_.resize(obs_grids_.size());

    int i = 0;

    for (const auto obs : obs_grids_)
    {
        vec_obs_[i](0) = obs[0];
        vec_obs_[i](1) = obs[1];
        i += 1;

    }

    return kSuccess;
  }
}  // namespace plan_manage