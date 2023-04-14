/**
 * @file Traj_server_ros.h
 * @author HKUST Aerial Robotics Group
 * @brief planner server
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

/**
 * @file map_server_ros.h revised version
 * @author Yuwei, from ZJU fast lab
 * @brief map adapter for minco planner
 * @version 0.2
 * @date 2021-04
 * @copyright Copyright (c) 2021
 */

#ifndef _UTIL_TRAJ_PLANNER_TRAJ_SERVER_ROS_H_
#define _UTIL_TRAJ_PLANNER_TRAJ_SERVER_ROS_H_

#include <chrono>
#include <numeric>
#include <thread>

#include "common/basics/colormap.h"
#include "common/basics/tic_toc.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/trajectory/frenet_traj.h"
#include "common/visualization/common_visualization_util.h"

#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"

#include "ros/ros.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"


#include "map_utils/map_adapter.h"


#include <plan_manage/traj_manager.h>

#include "traj_visualizer.h"


#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/encoder.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <functional>
#include<mutex>
#define Budget 0.5
namespace plan_utils
{

  class TrajPlannerServer 
  {
  public:
    using SemanticMapManager = semantic_map_manager::SemanticMapManager;
    struct Config {
      int kInputBufferSize{100};
    };

    TrajPlannerServer(ros::NodeHandle nh, int ego_id);
    TrajPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id);

    void Init(const std::string &config_path);
    void PushSemanticMap(const SemanticMapManager &smm);
    void Start();
    void SetEgoState(const common::State state){
      ego_state = state;
    }

  private:

    void PlanCycleCallback();
    
    void PublishData();

    void Display();

    void MainThread();
    void PublishThread();
    ErrorType FilterSingularityState(const vec_E<common::State> &hist, common::State *filter_state);

    ErrorType Replan();

    Config config_;

    TicToc time_profile_tool_;
    
    int final_traj_index_, exe_traj_index_;
    std::unique_ptr<SingulTrajData>  executing_traj_;
    std::unique_ptr<SingulTrajData>  next_traj_;
    
    bool is_replan_on_ = false;
    bool is_map_updated_ = false;
    bool use_sim_state_ = true;

    
    //_______________
    /* ros related */
    ros::NodeHandle nh_;
    ros::Publisher ctrl_signal_pub_, executing_traj_vis_pub_,debug_pub;
    decimal_t work_rate_ = 20.0;
    int ego_id_;
    bool enable_urban_ = false;

    //________________
    /* traj and map */
    plan_manage::TrajPlanner *p_planner_;
    map_utils::TrajPlannerAdapter map_adapter_;
    std::function<ErrorType()> trajplan;
    TrajVisualizer *p_traj_vis_{nullptr};


    bool require_intervention_signal_ = false;


    // input buffer
    moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;
    SemanticMapManager last_smm_;
    semantic_map_manager::Visualizer *p_smm_vis_{nullptr};

    int last_trajmk_cnt_{0};

    vec_E<common::State> desired_state_hist_;
    vec_E<common::State> ctrl_state_hist_;


    //______________________
    /* for parking module */
    void ParkingCallback(const geometry_msgs::PoseStamped &msg);
    bool CheckReplan();
    Eigen::Vector4d end_pt_;
    ros::Subscriber parking_sub_;
    common::State ego_state;
    std::mutex m;
    
    bool have_parking_target_ = false;
    bool isparking = true;
    common::VehicleParam vp_;


    /*publish the kinotraj as Path msg*/
    ros::Publisher kinopath_pub;


  };

}  // namespace plan_utils

#endif