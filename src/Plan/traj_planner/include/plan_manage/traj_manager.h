#ifndef _TRAJ_MANAGER_H_
#define _TRAJ_MANAGER_H_

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>
 
#include <ros/ros.h>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/primitive/frenet_primitive.h"
#include "common/spline/spline_generator.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_bezier_traj.h"
#include "common/trajectory/frenet_primitive_traj.h"
#include "minco_config.pb.h"

#include "map_utils/map_adapter.h"

#include "traj_optimizer.h"

#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"

#include "plan_utils/CorridorBuilder2d.hpp"
#include "path_searching/kino_astar.h"
#include "tf/tf.h"
//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>
//#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/PoseArray.h>
using std::vector;


std::ostream & operator<<(std::ostream &os, const common::LateralBehavior &lbehavior)
{
  os<<static_cast<std::underlying_type<common::LateralBehavior>::type>(lbehavior);
  return os;
}


namespace plan_manage
{
  using ObstacleMapType = uint8_t;
  using Lane = common::Lane;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;
  
  class TrajPlanner
  {
  public:

    friend std::ostream & operator << (std::ostream &os, const common::LateralBehavior &lbehavior);
    ///////////////////////////////////////////////////////////////
    TrajPlanner(ros::NodeHandle nh, int ego_id, bool enable_urban): nh_(nh), ego_id_(ego_id), enable_urban_(enable_urban) {}
    ~TrajPlanner() {
      /*hzc teb*/
      // delete costmap_ros_;delete buffer; delete tf;
      }

    ros::NodeHandle nh_;

    ErrorType Init(const std::string config_path);
    ErrorType RunOnce();
    ErrorType RunOnceParking();

    ErrorType set_map_interface(map_utils::TrajPlannerMapItf* map_itf);
    ErrorType set_initial_state(const State& state);
    void  release_initial_state(){has_init_state_ = false;};
    Vehicle ego_vehicle() const { return ego_vehicle_; }
    vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }
    vec_E<Polyhedron2D> polys() const { return polys_; }
    vec_Vec2f display_vec_obs() const { return vec_obs_; }
    std::vector<Eigen::MatrixXd> display_hPolys() const { return display_hPolys_; }
    // points visualizations
    Eigen::MatrixXd display_InnterPs() const {return display_InnterPs_;}
    Eigen::MatrixXd display_OptimalPs() const {return display_OptimalPs_;}
    plan_utils::SurroundTrajData display_surround_trajs() const {return surround_trajs;};

    std::unique_ptr<plan_utils::SingulTrajData> trajectory() const {
      return std::unique_ptr<plan_utils::SingulTrajData>(
        new plan_utils::SingulTrajData(traj_container_.singul_traj));
    }
    std::unique_ptr<plan_utils::KinoTrajData> display_kino_path() const {
      return std::unique_ptr<plan_utils::KinoTrajData>(
        new plan_utils::KinoTrajData(kino_trajs_)); 
    }
    
    
    void setParkingEnd(Eigen::Vector4d end_pt_){
      end_pt = end_pt_;
      have_parking_target = true;
    }
    //set moving obstalces, only used in the parking scene
    void set_surPoints(std::vector<std::vector<common::State>> surpoints){
      sur_discretePoints = surpoints;

    }
    decimal_t time_cost() const { return time_cost_; }

    planning::minco::Config cfg_;
    plan_utils::TrajContainer traj_container_;
    double traj_piece_duration_;
    int traj_res,dense_traj_res;





  private:

    ErrorType ReadConfig(const std::string config_path);
    /**
     * @brief transform all the states in a batch
     */

    int continous_failures_count_{0};
    int ego_id_;

    //_____________________________________
    /* kinodynamic a star path and parking*/
    std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
    plan_utils::KinoTrajData kino_trajs_;
    ErrorType getKinoPath(Eigen::Vector4d &end_state);
    ErrorType RunMINCOParking();
    bool enable_urban_ = false;

    /*teb local planner, for benchmark hzc*/
    /*
    std::shared_ptr<pluginlib::ClassLoader<nav_core::BaseLocalPlanner>> blp_loader_;
    boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* buffer;
    tf2_ros::TransformListener* tf;*/




    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    Vehicle ego_vehicle_;
    LateralBehavior ego_behavior_;
    Lane nav_lane_local_;



    // vehicle state
    State head_state_;
    bool has_init_state_ = false;
    bool is_init;


    //_____________
    /* Map related */
    map_utils::TrajPlannerMapItf* map_itf_;
    bool map_valid_ = false;
    decimal_t stamp_ = 0.0;
    decimal_t time_cost_ = 0.0;
  
    GridMap2D grid_map_;
    std::set<std::array<decimal_t, 2>> obs_grids_;
    std::vector<Eigen::MatrixXd> hPolys_, display_hPolys_;
    vec_E<Polyhedron2D> polys_;
    vec_Vec2f vec_obs_;
    


    vec_E<State> ref_states_;
    vec_E<plan_utils::Trajectory> ref_trajactory_list_;
    vec_E<vec_E<Vehicle>> forward_trajs_;   
    std::vector<LateralBehavior> forward_behaviors_;
    std::vector<LateralBehavior> valid_behaviors_; 
    vec_E<State> final_ref_states_;
    plan_utils::SurroundTrajData surround_trajs;




    Eigen::MatrixXd display_InnterPs_, display_OptimalPs_;
    std::vector<std::vector<common::State>> sur_discretePoints; // only used in the parking scene, moving obstacles





    vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

    ErrorType ConvertSurroundTraj(plan_utils::SurroundTrajData* surround_trajs_ptr, int index);
    ErrorType ConverSurroundTrajFromPoints(std::vector<std::vector<common::State>> sur_trajs,plan_utils::SurroundTrajData* surround_trajs_ptr);


    ErrorType UpdateTrajectoryWithCurrentBehavior();
    ErrorType RunMINCO();
    
    Eigen::MatrixXd state_to_flat_output(const State& state);

    void printLateralBehavior(LateralBehavior lateral_behavior);

    ErrorType UpdateObsGrids();
    
    ErrorType getSikangConst(plan_utils::Trajectory &Traj,
                             Eigen::MatrixXd &innerPs,  
                             Eigen::VectorXd &piece_dur_vec);
    // using galaxy to generate corridor                         
    ErrorType getGalaxyConst(plan_utils::Trajectory &Traj,
                             Eigen::MatrixXd &innerPs,  
                             Eigen::VectorXd &piece_dur_vec); 
    //using rectangles to represent corridor
    ErrorType getRectangleConst(std::vector<Eigen::Vector3d> statelist);

    bool checkShapeInside(Eigen::MatrixXd &hPoly, std::vector<Eigen::Vector2d> vertices);
    bool checkPosInside(Eigen::MatrixXd &hPoly,Eigen::Vector2d pos);
    void addBoundBox(Eigen::Vector2d &first_point, double angle, int i);

    /*for benchmark*/
    
    ros::Publisher KinopathPub;//for obca
    ros::Publisher DensePathVisPub;//for libai
    ros::Publisher DensePathPub;//for libai

    /*debug*/
    ros::Publisher Debugtraj0Pub;
    ros::Publisher Debugtraj1Pub;
    ros::Publisher DebugCorridorPub;
    ros::Publisher DebugtrajPub;
    /*vis dense hybridastar*/
    ros::Publisher DenseKinopathPub;
    
    /*if parking*/
    bool have_parking_target = false;
    Eigen::Vector4d end_pt;
    /*vehicle param*/



  };
}  // namespace plan_manage



#endif
