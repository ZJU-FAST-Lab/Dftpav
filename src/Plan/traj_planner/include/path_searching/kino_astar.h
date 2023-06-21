#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <math.h>
#include <utility> 

#include "map_utils/map_adapter.h"
#include "minco_config.pb.h"
#include "kino_model.hpp"
#include "plan_utils/traj_container.hpp"
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/point_types.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
namespace path_searching{

using namespace std;

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30


  class PathNode {
  public:
    /* -------------------- */
    Eigen::Vector2i index;
    int yaw_idx;
    /* --- the state is x y theta(orientation) */
    Eigen::Vector3d state;
    double g_score, f_score;
    /* control input should be steer and arc */
    Eigen::Vector2d input;
    PathNode* parent;
    char node_state;
    int singul = 0;
    /* -------------------- */
    PathNode() {
      parent = NULL;
      node_state = NOT_EXPAND;
    }
    ~PathNode(){};
  };
  typedef PathNode* PathNodePtr;




  class NodeComparator {
  public:
    template <class NodePtr>
    bool operator()(NodePtr node1, NodePtr node2) {
      return node1->f_score > node2->f_score;
    }
  };

  template <typename T>
  struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
      size_t seed = 0;
      for (long int i = 0; i < matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };
  template <class NodePtr>
  class NodeHashTable {
  private:
    /* data */

    std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash<Eigen::Vector2i>> data_2d_;
    std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;



  public:
    NodeHashTable(/* args */) {
    }
    ~NodeHashTable() {
    }
    // : for 2d vehicle planning
    void insert(Eigen::Vector2i idx, NodePtr node) {
      data_2d_.insert(std::make_pair(idx, node));
    }
    //for 3d vehicle planning
    void insert(Eigen::Vector2i idx, int yaw_idx, NodePtr node) {
      data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), yaw_idx), node));
    }
    void insert(Eigen::Vector3i idx,NodePtr node ){
      data_3d_.insert(std::make_pair(idx,node));
    }

    NodePtr find(Eigen::Vector2i idx) {
      auto iter = data_2d_.find(idx);
      return iter == data_2d_.end() ? NULL : iter->second;
    }
    NodePtr find(Eigen::Vector2i idx, int yaw_idx) {
      auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), yaw_idx));
      return iter == data_3d_.end() ? NULL : iter->second;
    }

    void clear() {
      data_2d_.clear();
      data_3d_.clear();
    }
  };

 
  class KinoAstar {
  
  private:
    /* ---------- main data structure ---------- */
    vector<PathNodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    NodeHashTable<PathNodePtr> expanded_nodes_;
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
    std::vector<PathNodePtr> path_nodes_;

    /* ---------- record data ---------- */
    Eigen::Vector4d start_state_, end_state_;
    Eigen::Vector2d start_ctrl;
    bool is_shot_succ_ = false;
    bool has_path_ = false;
    /* ---------- parameter ---------- */
    /* search */
    double step_arc = 1.0;
    double max_cur_ = 0.35;
    double max_steer_ = 0.78539815;
    double max_seach_time = 0.1;
    double max_forward_vel = 4.0;
    double max_forward_acc = 2.0;
    double max_backward_vel = 2.0;
    double max_backward_acc = 1.0;

    double traj_forward_penalty = 1.0;
    double traj_back_penalty = 1.0;
    double traj_gear_switch_penalty = 10.0;
    double traj_steer_penalty = 0.0;
    double traj_steer_change_penalty = 0.0;
    double horizon_;
    double lambda_heu_;


    //double margin_;
    int allocate_num_;
    int check_num_;
    double tie_breaker_ = 1.0 + 1.0 / 10000;




    /* map */
    double resolution_, inv_resolution_, yaw_resolution_, inv_yaw_resolution_;
    Eigen::Vector2d origin_, map_size_3d_;
    double yaw_origin_ = -M_PI;
    /*shot hzc*/
    std::vector<double>  shot_timeList;
    std::vector<double>  shot_lengthList;
    std::vector<int>     shotindex;
    std::vector<int>     shot_SList; 
    std::vector<Eigen::Vector3d> SampleTraj;

    /* helper */
    Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    int yawToIndex(double yaw);
    void retrievePath(PathNodePtr end_node);


    void getState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                  common::State &common_state);
    void getFlatState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                      Eigen::MatrixXd &flat_state, int singul);

    bool is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2);
    void computeShotTraj(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                          std::vector<Eigen::Vector3d> &path_list,
                          double& len); 
    void stateTransit(Eigen::Vector3d &state0,  Eigen::Vector3d &state1,
              Eigen::Vector2d &ctrl_input);

    double evaluateLength(double curt,double locallength,double localtime, double max_vel, double max_acc, double startV = 0.0, double endV = 0.0);
    double evaluateDuration(double length, double max_vel, double max_acc, double startV = 0.0, double endV = 0.0);


    map_utils::TrajPlannerMapItf* frontend_map_itf_;
    common::VehicleParam vp_;
    double non_siguav=0.2;
    ompl::base::StateSpacePtr shotptr;

    inline int getSingularity(double vel)
    {
      int singul = 0;
      if (fabs(vel) > 1e-2){
        if (vel >= 0.0){singul = 1;} 
        else{singul = -1;}      
      }
      
      return singul;
    }
    // getHeu
    inline double getHeu(Eigen::VectorXd x1, Eigen::VectorXd x2)
    {
      double dx = abs(x1(0) - x2(0));
      double dy = abs(x1(1) - x2(1));
      return tie_breaker_* sqrt(dx * dx + dy * dy);
    }



  public:
    KinoAstar(){};
    ~KinoAstar();

    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};

    /* main API */
    void setParam(ros::NodeHandle& nh);
    void init(planning::minco::Config cfg_, ros::NodeHandle nh);

    void reset();
    int search(Eigen::Vector4d start_state, Eigen::Vector2d init_ctrl,
              Eigen::Vector4d end_state, bool use3d = false);
    // inital semantic map
    void intialMap(map_utils::TrajPlannerMapItf *map_itf);
    // get kino traj for optimization  
    void getKinoNode(plan_utils::KinoTrajData &flat_trajs);
    void NodeVis(Eigen::Vector3d state);

    /*hzchzc*/
    Eigen::Vector3d evaluatePos(double t);
    std::vector<Eigen::Vector4d> SamplePosList(int N); //px py yaw t 
    std::vector<Eigen::Vector3d> getSampleTraj();
    double totalTrajTime;
    double checkl = 0.2;

    ros::Publisher expandNodesVis;



    std::vector<PathNodePtr> getVisitedNodes();

    std::vector<Eigen::Vector4d>  state_list;
    std::vector<Eigen::Vector3d> acc_list;

    typedef shared_ptr<KinoAstar> Ptr;


  };

}  // namespace 

#endif
