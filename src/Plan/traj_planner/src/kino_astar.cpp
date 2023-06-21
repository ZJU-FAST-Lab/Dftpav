#include <path_searching/kino_astar.h>
#include <sstream>
#include "common/math/calculations.h"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

namespace path_searching
{
  KinoAstar::~KinoAstar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }
  void KinoAstar::stateTransit(Eigen::Vector3d &state0,  Eigen::Vector3d &state1,
              Eigen::Vector2d &ctrl_input){
    //helpful var
    double psi = ctrl_input[0]; double s = ctrl_input[1]; 
    if(psi!=0){
      double k = vp_.wheel_base()/tan(psi);
      state1[0] = state0[0] + k*(sin(state0[2]+s/k)-sin(state0[2]));
      state1[1] = state0[1] - k*(cos(state0[2]+s/k)-cos(state0[2]));
      state1[2] = state0[2] + s/k;
    }
    else{
      state1[0] = state0[0] + s * cos(state0[2]);
      state1[1] = state0[1] + s * sin(state0[2]);
      state1[2] = state0[2]; 
    }
  }
  int KinoAstar::search(Eigen::Vector4d start_state, Eigen::Vector2d init_ctrl,
                               Eigen::Vector4d end_state,bool use3d)
  {
    bool isocc = false;  bool initsearch = false;
    TicToc time_profile_tool_;
    time_profile_tool_.tic();
    frontend_map_itf_->CheckIfCollisionUsingPosAndYaw(vp_,start_state.head(3),&isocc);
    if(isocc){
      ROS_ERROR("KinoAstar: head is not free!");
      return NO_PATH;
    }
    frontend_map_itf_->CheckIfCollisionUsingPosAndYaw(vp_,end_state.head(3),&isocc);
    if(isocc){
      ROS_WARN("KinoAstar: end is not free!");
      return NO_PATH;
    }
    start_state_ = start_state;
    start_ctrl = init_ctrl;
    end_state_ = end_state;
    Eigen::Vector2i end_index;
    end_index = posToIndex(end_state.head(2));
    /* ---------- initialize ---------- */
    PathNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state = start_state.head(3);
    cur_node->index = posToIndex(start_state.head(2));
    cur_node->yaw_idx = yawToIndex(start_state[2]);
    cur_node->g_score = 0.0;
    cur_node->input = Eigen::Vector2d(0.0,0.0);
    cur_node->singul = getSingularity(start_state(3));
    cur_node->f_score = lambda_heu_ * getHeu(cur_node->state, end_state);
    cur_node->node_state = IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_ += 1;
    if(!use3d)
      expanded_nodes_.insert(cur_node->index, cur_node);
    else
      expanded_nodes_.insert(cur_node->index, yawToIndex(start_state[2]),cur_node);
    PathNodePtr terminate_node = NULL;
    if(cur_node->singul == 0){ initsearch = true;}
    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
        // ros::Duration(0.1).sleep();

      // std::cout<<"cur_state: "<<cur_node->state.transpose()<<"\n";
      /* ---------- get lowest f_score node ---------- */
      cur_node = open_set_.top();

      /* ---------- determine termination ---------- */
      // to decide the near end
      bool reach_horizon = (cur_node->state.head(2) - start_state_.head(2)).norm() >= horizon_;
      double t1 = ros::Time::now().toSec();
      if((cur_node->state.head(2) - end_state_.head(2)).norm()<15.0&& initsearch){
    
        is_shot_sucess(cur_node->state,end_state_.head(3));
      }
      double t2 = ros::Time::now().toSec();
      // std::cout<<"one-shot time: "<<(t2-t1)*1000<<" ms"<<std::endl;
      
      if (is_shot_succ_)
      {
        terminate_node = cur_node;
        retrievePath(terminate_node);
        has_path_ = true;
        if (is_shot_succ_)
        {
          /* one shot trajectory */

          ROS_WARN("one shot! iter num: %d",iter_num_);
          return REACH_END;
        }
        else if (reach_horizon)
        {
          cout << "[Kino Astar]: Reach horizon_" << endl;
          return REACH_HORIZON;
        }
      }
      double runTime = time_profile_tool_.toc() / 1000.0;
      if(runTime > max_seach_time){
        terminate_node = cur_node;
        retrievePath(terminate_node);
        has_path_ = true;
        if (terminate_node->parent == NULL)
        {

          cout << "[34mKino Astar]: terminate_node->parent == NULL" << endl;
          printf("\033[Kino Astar]: NO_PATH \n\033[0m");
          return NO_PATH;
        }
        else
        {
          ROS_WARN("KinoSearch: Reach the max seach time");
          return REACH_END;
        }
      }
      /* ---------- pop node and add to close set ---------- */
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;
      /* ---------- init state propagation ---------- */
      Eigen::Vector3d cur_state = cur_node->state;
      Eigen::Vector3d pro_state;
      Eigen::Vector2d ctrl_input;
      vector<Eigen::Vector2d> inputs;
      double res = 0.5;
      if(!initsearch ){
        if(start_state_[3]>0){
          for(double arc = resolution_; arc <= 2*resolution_ + 1e-3; arc += resolution_){
            for(double steer = -max_steer_; steer <= max_steer_ + 1e-3; steer += res * max_steer_* 1.0){
              ctrl_input<< steer, arc;
              inputs.push_back(ctrl_input);
            }
          }
        }
        else{
          for(double arc = -resolution_; arc >= -2*resolution_ - 1e-3; arc-= resolution_){
            for(double steer = -max_steer_; steer <= max_steer_ + 1e-3; steer += res * max_steer_* 1.0){
              ctrl_input<< steer, arc;
              inputs.push_back(ctrl_input);
            }
          } 
        }
        initsearch = true;
      }
      else{
        for (double arc = -step_arc; arc <= step_arc + 1e-3; arc += 0.5*step_arc){
          if(fabs(arc)<1.0e-2) continue;
          for (double steer = -max_steer_; steer <= max_steer_ + 1e-3; steer += res * max_steer_*1.0)
          {
            ctrl_input << steer, arc;
            inputs.push_back(ctrl_input);
          }
        }
      }
      /* ---------- state propagation loop ---------- */
      for (auto& input:inputs){
        int singul = input[1]>0?1:-1;
        stateTransit(cur_state, pro_state, input);
        NodeVis(pro_state);

        // std::cout<<"cur_state: "<<cur_state.transpose()<<" pro: "<<pro_state.transpose()<<" input: "<<input.transpose()<<"\n";

        /* inside map range */
        if (pro_state(0) <= origin_(0) || pro_state(0) >= map_size_3d_(0)*0.5 ||
            pro_state(1) <= origin_(1) || pro_state(1) >= map_size_3d_(1)*0.5)
        {
          std::cout << "[Kino Astar]: out of map range" << endl;
          continue;
        }
        /* not in close set */
        Eigen::Vector2i pro_id = posToIndex(pro_state.head(2));
        double pro_yaw_id = yawToIndex(pro_state[2]);
        PathNodePtr pro_node;
        if(use3d)
          pro_node = expanded_nodes_.find(pro_id, pro_yaw_id);
        else
          pro_node = expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          // std::cout<<"in close set!\n";
          continue;
        }

        // /* not in the same voxel */
        Eigen::Vector2i diff = pro_id - cur_node->index;
        int diff_yaw = pro_yaw_id - cur_node->yaw_idx;
        if (diff.norm() == 0 && ((!use3d) || diff_yaw == 0))
        {
          std::cout<<"diff.norm() == 0 && ((!use3d) || diff_yaw == 0)!\n";
          continue;
        }
        /* collision free */
        Eigen::Vector3d xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double tmparc = input[1] * double(k) / double(check_num_);
          Eigen::Vector2d tmpctrl; tmpctrl << input[0],tmparc;
          stateTransit(cur_state, xt, tmpctrl);
          frontend_map_itf_->CheckIfCollisionUsingPosAndYaw(vp_, xt, &is_occ);
          if (is_occ)
          {
            // std::cout<<"occ!\n";
            break;
          }

        }

        if (is_occ)  continue;

        /* ---------- compute cost ---------- */
        double tmp_g_score = 0.0;
        double tmp_f_score = 0.0;
        int lastDir = cur_node->singul;
        if(singul>0){
          tmp_g_score +=  std::fabs(input[1]) * traj_forward_penalty;
        }
        else{
          tmp_g_score += std::fabs(input[1]) * traj_back_penalty;
        }
        // std::cout<<"1111111111111111gscore: "<<tmp_g_score<<"\n";
        if(singul * lastDir < 0){
          tmp_g_score += traj_gear_switch_penalty;
        }
        tmp_g_score += traj_steer_penalty * std::fabs(input[0]) * std::fabs(input[1]);
        tmp_g_score += traj_steer_change_penalty * std::fabs(input[0]-cur_node->input[0]);
        tmp_g_score += cur_node->g_score;
        // std::cout<<"2222222222222222222222gscrore: "<<tmp_g_score<<"\n";
        tmp_f_score = tmp_g_score + lambda_heu_ * getHeu(pro_state, end_state);
        // tmp_f_score = tmp_g_score + lambda_heu_ * getObsHeu(pro_state.head(2));
        // std::cout<<"tmpfscore: "<<tmp_f_score<<"\n";
        // std::cout<<"g1: "<<( ctrl_input(1)*ctrl_input(1) + 10*ctrl_input(0)*ctrl_input(0) + w_time_) * tau<<" g0: "<<backward_penalty<<
        // "h: "<<lambda_heu_ * getHeu(pro_state, end_state)<<"\n";
        /* ---------- compare expanded node in this loop ---------- */
        if (pro_node == NULL)
        {
          pro_node = path_node_pool_[use_node_num_];
          pro_node->index = pro_id;
          pro_node->state = pro_state;
          pro_node->yaw_idx = pro_yaw_id;
          pro_node->f_score = tmp_f_score;
          pro_node->g_score = tmp_g_score;
          pro_node->input = input;
          pro_node->parent = cur_node;
          pro_node->node_state = IN_OPEN_SET;
          pro_node->singul = singul;
          open_set_.push(pro_node);
          if(use3d)
            expanded_nodes_.insert(pro_id, pro_yaw_id, pro_node);
          else
            expanded_nodes_.insert(pro_id,pro_node);
          use_node_num_ += 1;
          if (use_node_num_ == allocate_num_)
          {
            cout << "run out of memory." << endl;
            return NO_PATH;
          }
        }
        else if (pro_node->node_state == IN_OPEN_SET)
        {

          if (tmp_g_score < pro_node->g_score)
          {
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->yaw_idx = pro_yaw_id;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = input;
            pro_node->parent = cur_node;
            pro_node->singul = singul;
          }
        }
        else
        {
          cout << "error type in searching: " << pro_node->node_state << endl;
        }
      }
    }

    /* ---------- open set empty, no path ---------- */
    cout << "open set empty, no path." << endl;
    return NO_PATH;
  }


  bool KinoAstar::is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2){
    
    std::vector<Eigen::Vector3d> path_list;
    double len;
    double ct1 = ros::Time::now().toSec();
    computeShotTraj(state1,state2,path_list,len);
    double ct2 = ros::Time::now().toSec();
    // std::cout<<"compute shot traj time: "<<(ct2-ct1)*1000.0<<" ms"<<std::endl;
    bool is_occ = false;
    double t1 = ros::Time::now().toSec();
    for(unsigned int i = 0; i < path_list.size(); ++i){
        frontend_map_itf_->CheckIfCollisionUsingPosAndYaw(vp_, path_list[i], &is_occ);
        if(is_occ) return false;
    }
    double t2 = ros::Time::now().toSec();


    is_shot_succ_ = true;
    return true;
  }



  void KinoAstar::computeShotTraj(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                                    std::vector<Eigen::Vector3d> &path_list,
                                    double& len){
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
    to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
    std::vector<double> reals;
    len = shotptr->distance(from(), to());

    for (double l = 0.0; l <=len; l += checkl)
    {
      shotptr->interpolate(from(), to(), l/len, s());
      reals = s.reals();
      path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
    }
  
    return;
  }



  // to retrieve the path to the correct order
  void KinoAstar::retrievePath(PathNodePtr end_node)
  {
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }


  void KinoAstar::intialMap(map_utils::TrajPlannerMapItf *map_itf)
  {
    frontend_map_itf_ = map_itf;
    // Dp_map.intialMap(map_itf);
  }

  void KinoAstar::init(planning::minco::Config cfg_, ros::NodeHandle nh)
  {

    // Dp_map.init(cfg_);
    horizon_ = cfg_.map_cfg().horizon();
    yaw_resolution_ = cfg_.map_cfg().phi_grid_resolution();
    lambda_heu_ = cfg_.map_cfg().lambda_heu();
    allocate_num_ = cfg_.map_cfg().allocate_num();
    check_num_ = cfg_.map_cfg().check_num();
    max_seach_time  = cfg_.map_cfg().max_search_time();
    traj_forward_penalty = cfg_.map_cfg().traj_forward_penalty();
    traj_back_penalty = cfg_.map_cfg().traj_back_penalty();
    traj_gear_switch_penalty = cfg_.map_cfg().traj_gear_switch_penalty();
    traj_steer_penalty = cfg_.map_cfg().traj_steer_penalty();
    traj_steer_change_penalty = cfg_.map_cfg().traj_steer_change_penalty();
    step_arc = cfg_.map_cfg().step_arc();
    checkl = cfg_.map_cfg().checkl();
    
    



    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new PathNode;
    }

    use_node_num_ = 0;
    iter_num_ = 0;

    map_size_3d_(0) = cfg_.map_cfg().map_size_x();
    map_size_3d_(1) = cfg_.map_cfg().map_size_y();

    resolution_ = cfg_.map_cfg().map_resl();
    origin_(0) = -0.5*map_size_3d_(0);
    origin_(1) = -0.5*map_size_3d_(1);
    yaw_origin_ = -M_PI;
         
    max_forward_vel = cfg_.opt_cfg().max_frontend_forward_vel();
    max_forward_acc = cfg_.opt_cfg().max_frontend_forward_acc();
    max_backward_vel = cfg_.opt_cfg().max_frontend_backward_vel();
    max_backward_acc = cfg_.opt_cfg().max_frontend_backward_acc();


    max_cur_ = cfg_.opt_cfg().max_frontend_cur();
    max_steer_ = std::atan(vp_.wheel_base()*max_cur_);
    inv_resolution_ = 1.0 / resolution_;
    inv_yaw_resolution_ = 1.0 / yaw_resolution_;

    shotptr =std::make_shared<ompl::base::ReedsSheppStateSpace>(1.0/max_cur_);

    //set margain
    vp_.set_length(vp_.length()+0.2);
    vp_.set_width(vp_.width()+0.2);

    // non_siguav = 1.0e-2;


    //

    expandNodesVis = nh.advertise<sensor_msgs::PointCloud2>("/vis/expanded_nodes", 1);







  }
    

  void KinoAstar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      PathNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
  }
  std::vector<Eigen::Vector3d> KinoAstar::getSampleTraj(){
    return SampleTraj;
  }
  /*hzchzc px py yaw*/
  Eigen::Vector3d KinoAstar::evaluatePos(double t){
    t = std::min<double>(std::max<double>(0,t),totalTrajTime);
    double startvel = fabs(start_state_[3]);
    double endvel = fabs(end_state_[3]);
    int index = -1;
    double tmpT = 0;
    double CutTime;
    //locate the local traj
    for(int i = 0;i<shot_timeList.size();i++){
      tmpT+=shot_timeList[i];
      if(tmpT>=t) {
        index = i; 
        CutTime = t-tmpT+shot_timeList[i];
        break;
        }
    }

    double initv = non_siguav,finv = non_siguav;
    if(index==0) {initv  = startvel;}
    if(index==shot_lengthList.size() - 1) finv = endvel;

    double localtime = shot_timeList[index];
    double locallength = shot_lengthList[index];
    int front = shotindex[index]; int back =  shotindex[index+1];
    std::vector<Eigen::Vector3d> localTraj;localTraj.assign(SampleTraj.begin()+front,SampleTraj.begin()+back+1);
    //find the nearest point
    double arclength;
    if(shot_SList[index] > 0)
      arclength= evaluateLength(CutTime,locallength,localtime,max_forward_vel,max_forward_acc, initv,finv);
    else
      arclength= evaluateLength(CutTime,locallength,localtime,max_backward_vel, max_backward_acc, initv,finv);
    double tmparc = 0;
    for(int i = 0; i < localTraj.size()-1;i++){
      tmparc += (localTraj[i+1]-localTraj[i]).head(2).norm();
      if(tmparc>=arclength){
        double l1 = tmparc-arclength;
        double l = (localTraj[i+1]-localTraj[i]).head(2).norm();
        double l2 = l-l1;//l2
        Eigen::Vector3d state = l1/l*localTraj[i]+l2/l*localTraj[i+1];
        if(fabs(localTraj[i+1][2]-localTraj[i][2])>=M_PI){   
          double normalize_yaw;
          if(localTraj[i+1][2]<=0){
            normalize_yaw = l1/l*localTraj[i][2]+l2/l*(localTraj[i+1][2]+2*M_PI);
          }
          else if(localTraj[i][2]<=0){
            normalize_yaw = l1/l*(localTraj[i][2]+2*M_PI)+l2/l*localTraj[i+1][2];
          }
          state[2] = normalize_yaw;
        }
        return state;
      }
    }
    return localTraj.back();
  }
  std::vector<Eigen::Vector4d> KinoAstar::SamplePosList(int N){
      double dt = totalTrajTime/N;
      std::vector<Eigen::Vector4d>  path;
      for(int i=0;i<=N;i++){
        double time = i*dt;
        Eigen::Vector4d posAndt;
        posAndt<<evaluatePos(time),time;
        path.push_back(posAndt);
      }
      return path;
  }
  
  void KinoAstar::NodeVis(Eigen::Vector3d state){
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::PointXYZ pt;
    pt.x = state[0];
    pt.y = state[1];
    pt.z = 0.2;
    cloudMap.points.push_back(pt); 
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.stamp = ros::Time::now();
    globalMap_pcd.header.frame_id = "map";
    expandNodesVis.publish(globalMap_pcd);



    return;
  }
  void KinoAstar::getKinoNode(plan_utils::KinoTrajData &flat_trajs)
  {
    double truncate_len = 25.0;
    bool exceed_len = false;

    flat_trajs.clear();
    std::vector<Eigen::Vector3d> roughSampleList;
    double startvel = fabs(start_state_[3]);
    double endvel = fabs(end_state_[3]);
    PathNodePtr node = path_nodes_.back();
    std::vector<Eigen::Vector3d> traj_pts;  // 3, N
    std::vector<double> thetas;
    Eigen::Vector4d x0, xt;
    Vector2d ut, u0;
    while(node->parent != NULL){
      for (int k = check_num_; k >0; k--)
      {
        Eigen::Vector3d state;
        double tmparc = node->input[1] * double(k) / double(check_num_);
        Eigen::Vector2d tmpctrl; tmpctrl << node->input[0],tmparc;
        stateTransit(node->parent->state, state, tmpctrl);
        state[2] = normalize_angle(state[2]);
        roughSampleList.push_back(state);

      }
      node = node->parent;
    } 
    start_state_[2] = normalize_angle(start_state_[2]);
    roughSampleList.push_back(start_state_.head(3));
    reverse(roughSampleList.begin(),roughSampleList.end());

    if(is_shot_succ_){
      ompl::base::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
      Eigen::Vector3d state1,state2;
      state1 = roughSampleList.back();
      state2 = end_state_.head(3);
      from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
      to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
      double shotLength = shotptr->distance(from(), to());
      std::vector<double> reals;
      for(double l = checkl; l < shotLength; l += checkl){
        shotptr->interpolate(from(), to(), l/shotLength, s());
        reals = s.reals();
        roughSampleList.push_back(Eigen::Vector3d(reals[0], reals[1], normalize_angle(reals[2])));        
      }
      end_state_[2] = normalize_angle(end_state_[2]);
      roughSampleList.push_back(end_state_.head(3));
    }
    //truncate the init trajectory
    double tmp_len = 0;
    int truncate_idx = 0;
    for(truncate_idx = 0;truncate_idx <roughSampleList.size()-1;truncate_idx++){  
      tmp_len += (roughSampleList[truncate_idx+1]-roughSampleList[truncate_idx]).norm();
      // if(tmp_len>truncate_len){
      //   break;
      // }
    }
    roughSampleList.assign(roughSampleList.begin(),roughSampleList.begin()+truncate_idx+1);
    SampleTraj = roughSampleList;
    /*divide the whole shot traj into different segments*/   
    shot_lengthList.clear();
    shot_timeList.clear();
    shotindex.clear();
    shot_SList.clear(); 
    double tmpl = 0;
    bool ifnewtraj = false;
    int lastS = (SampleTraj[1]-SampleTraj[0]).head(2).dot(Eigen::Vector2d(cos(SampleTraj[0][2]),sin(SampleTraj[0][2])))>=0?1:-1;
    //hzchzc attention please!!! max_v must = min_v  max_acc must = min_acc
    shotindex.push_back(0);
    for(int i = 0; i<SampleTraj.size()-1; i++){
      Eigen::Vector3d state1 = SampleTraj[i];
      Eigen::Vector3d state2 = SampleTraj[i+1];
      int curS = (state2-state1).head(2).dot(Eigen::Vector2d(cos(state1[2]),sin(state1[2]))) >=0 ? 1:-1;
      if(curS*lastS >= 0){
        tmpl += (state2-state1).head(2).norm();
      }
      else{  
        shotindex.push_back(i);
        shot_SList.push_back(lastS);
        shot_lengthList.push_back(tmpl);
        if(lastS>0)
          shot_timeList.push_back(evaluateDuration(tmpl,max_forward_vel, max_forward_acc, non_siguav,non_siguav));
        else
          shot_timeList.push_back(evaluateDuration(tmpl,max_backward_vel, max_backward_acc, non_siguav,non_siguav));

        tmpl = (state2-state1).head(2).norm();
      }       
      lastS = curS;
    }
    shot_SList.push_back(lastS);
    shot_lengthList.push_back(tmpl);
    if(lastS>0)
      shot_timeList.push_back(evaluateDuration(tmpl,max_forward_vel, max_forward_acc, non_siguav,non_siguav));
    else
      shot_timeList.push_back(evaluateDuration(tmpl,max_backward_vel, max_backward_acc, non_siguav,non_siguav));
    shotindex.push_back(SampleTraj.size()-1);
    if(shot_timeList.size()>=2){
      if(shot_SList[0]>0)
        shot_timeList[0] = evaluateDuration(shot_lengthList[0],max_forward_vel,max_forward_acc, startvel,non_siguav);
      else
        shot_timeList[0] = evaluateDuration(shot_lengthList[0],max_backward_vel, max_backward_acc, startvel,non_siguav);
      if(shot_SList[shot_timeList.size()-1]>0)
        shot_timeList[shot_timeList.size()-1] = evaluateDuration(shot_lengthList.back(),max_forward_vel,max_forward_acc,non_siguav,endvel);
      else
        shot_timeList[shot_timeList.size()-1] = evaluateDuration(shot_lengthList.back(),max_backward_vel,max_backward_acc,non_siguav,endvel);
    }
    else{
      if(shot_SList[0]>0)
        shot_timeList[0] = evaluateDuration(shot_lengthList[0],max_forward_vel,max_forward_acc, startvel,endvel);
      else  
        shot_timeList[0] = evaluateDuration(shot_lengthList[0],max_backward_vel,max_backward_acc, startvel,endvel);
    }
    /*extract flat traj  
    the flat traj include the end point but not the first point*/
    for(int i=0;i<shot_lengthList.size();i++){
      double initv = non_siguav,finv = non_siguav;
      Eigen::Vector2d Initctrlinput,Finctrlinput;
      Initctrlinput<<0,0;Finctrlinput<<0,0;
      if(i==0) {initv  = startvel; Initctrlinput = start_ctrl;}
      if(i==shot_lengthList.size() - 1) finv = endvel;

      double locallength = shot_lengthList[i];
      int sig = shot_SList[i];
      std::vector<Eigen::Vector3d> localTraj;localTraj.assign(SampleTraj.begin()+shotindex[i],SampleTraj.begin()+shotindex[i+1]+1);
      traj_pts.clear();
      thetas.clear();        
      double samplet;
      double tmparc = 0;
      int index = 0;
      double sampletime = 0.1;
      if(shot_timeList[i]<=sampletime){
        sampletime = shot_timeList[i] / 2.0;
      }
      for(samplet = sampletime; samplet<shot_timeList[i]; samplet+=sampletime){
        double arc;
        if(sig > 0){
          arc = evaluateLength(samplet,locallength,shot_timeList[i],max_forward_vel, max_forward_acc, initv, finv);
        }
        else{
          arc = evaluateLength(samplet,locallength,shot_timeList[i],max_backward_vel, max_backward_acc, initv, finv);
        }
        
        //find the nearest point
        for(int k = index; k<localTraj.size()-1 ;k++)
        {
          tmparc+= (localTraj[k+1]-localTraj[k]).head(2).norm();
          if(tmparc>=arc){
            index = k;
            double l1 = tmparc-arc;
            double l = (localTraj[k+1]-localTraj[k]).head(2).norm();
            double l2 = l-l1;//l2
            double px = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[0];
            double py = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[1];
            double yaw= (l1/l*localTraj[k]+l2/l*localTraj[k+1])[2];
            if(fabs(localTraj[k+1][2]-localTraj[k][2])>=M_PI){   
              double normalize_yaw;
              if(localTraj[k+1][2]<=0){
                normalize_yaw = l1/l*localTraj[k][2]+l2/l*(localTraj[k+1][2]+2*M_PI);
              }
              else if(localTraj[k][2]<=0){
                normalize_yaw = l1/l*(localTraj[k][2]+2*M_PI)+l2/l*localTraj[k+1][2];
              }
              yaw = normalize_yaw;
            }
            traj_pts.push_back(Eigen::Vector3d(px,py,sampletime));
            thetas.push_back(yaw);
            tmparc -=(localTraj[k+1]-localTraj[k]).head(2).norm();
            break;
          }
        }      
      }
      traj_pts.push_back(Eigen::Vector3d(localTraj.back()[0],localTraj.back()[1],shot_timeList[i]-(samplet-sampletime)));
      thetas.push_back(localTraj.back()[2]);
      plan_utils::FlatTrajData flat_traj;
      Eigen::MatrixXd startS;
      Eigen::MatrixXd endS;
      getFlatState(Eigen::Vector4d(localTraj.front()[0],localTraj.front()[1],localTraj.front()[2],initv),Initctrlinput,startS,sig);
      getFlatState(Eigen::Vector4d(localTraj.back()[0],localTraj.back()[1],localTraj.back()[2],finv),Finctrlinput,endS,sig);
      flat_traj.traj_pts = traj_pts;
      flat_traj.thetas = thetas;
      flat_traj.start_state = startS;
      flat_traj.final_state = endS;
      flat_traj.singul = sig;
      flat_trajs.push_back(flat_traj);
    }
    totalTrajTime = 0.0;
    for(const auto dt : shot_timeList){
       totalTrajTime += dt; 
    }
  }
  double KinoAstar::evaluateDuration(double length, double max_vel, double max_acc, double startV, double endV){
   double critical_len; //the critical length of two-order optimal control, acc is the input
    if(startV>max_vel||endV>max_vel){
      ROS_ERROR("kinoAstar:evaluateDuration:start or end vel is larger that the limit!");
    }
    double startv2 = pow(startV,2);
    double endv2 = pow(endV,2);
    double maxv2 = pow(max_vel,2);
    critical_len = (maxv2-startv2)/(2*max_acc)+(maxv2-endv2)/(2*max_acc);
    if(length>=critical_len){
      return (max_vel-startV)/max_acc+(max_vel-endV)/max_acc+(length-critical_len)/max_vel;
    }
    else{
      double tmpv = sqrt(0.5*(startv2+endv2+2*max_acc*length));
      return (tmpv-startV)/max_acc + (tmpv-endV)/max_acc;
    }


  }
  double KinoAstar::evaluateLength(double curt,double locallength,double localtime,double max_vel, double max_acc, double startV, double endV){
    double critical_len; //the critical length of two-order optimal control, acc is the input
    if(startV>max_vel||endV>max_vel){
      ROS_ERROR("kinoAstar:evaluateLength:start or end vel is larger that the limit!");
    }
    double startv2 = pow(startV,2);
    double endv2 = pow(endV,2);
    double maxv2 = pow(max_vel,2);
    critical_len = (maxv2-startv2)/(2*max_acc)+(maxv2-endv2)/(2*max_acc);
    if(locallength>=critical_len){
      double t1 = (max_vel-startV)/max_acc;
      double t2 = t1+(locallength-critical_len)/max_vel;
      if(curt<=t1){
        return startV*curt + 0.5*max_acc*pow(curt,2);
      }
      else if(curt<=t2){
        return startV*t1 + 0.5*max_acc*pow(t1,2)+(curt-t1)*max_vel;
      }
      else{
        return startV*t1 + 0.5*max_acc*pow(t1,2) + (t2-t1)*max_vel + max_vel*(curt-t2)-0.5*max_acc*pow(curt-t2,2);
      }
    }
    else{
      double tmpv = sqrt(0.5*(startv2+endv2+2*max_acc*locallength));
      double tmpt = (tmpv-startV)/max_acc;
      if(curt<=tmpt){
        return startV*curt+0.5*max_acc*pow(curt,2);
      }
      else{
        return startV*tmpt+0.5*max_acc*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*max_acc*pow(curt-tmpt,2);
      }
    }
  }

  std::vector<PathNodePtr> KinoAstar::getVisitedNodes()
  {
    vector<PathNodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  Eigen::Vector2i KinoAstar::posToIndex(Eigen::Vector2d pt)
  {
    int idx = std::round((pt[0]-origin_[0])/resolution_);
    int idy = std::round((pt[1]-origin_[1])/resolution_);
    return Eigen::Vector2i(idx,idy);
  }

  int KinoAstar::yawToIndex(double yaw)
  {
    yaw = normalize_angle(yaw);
    int idx = floor((yaw - yaw_origin_) * inv_yaw_resolution_);
    return idx;
  }


  void KinoAstar::getState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                                     common::State &common_state)
  {

    common_state.vec_position = state.head(2); 
    common_state.angle        = state(2); 
    common_state.velocity     = state(3); 

    common_state.steer        = control_input(0);
    common_state.acceleration = control_input(1);
    common_state.curvature    = std::tan(common_state.steer) / vp_.wheel_base();

  }


  void KinoAstar::getFlatState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                                  Eigen::MatrixXd &flat_state, int singul)
  {

    flat_state.resize(2, 3);

    double angle = state(2);
    double vel   = state(3); // vel > 0 

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
              sin(angle),   cos(angle);

    if (abs(vel) <= non_siguav){
      vel = singul * non_siguav;
    }
    else{
      vel = singul * vel;
    }
    
    flat_state << state.head(2), init_R*Eigen::Vector2d(vel, 0.0), 
                  init_R*Eigen::Vector2d(control_input(1), std::tan(control_input(0)) / vp_.wheel_base() * std::pow(vel, 2));

  }

} // namespace resilient_planner
