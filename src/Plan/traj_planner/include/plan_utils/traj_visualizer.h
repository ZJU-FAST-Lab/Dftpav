#ifndef _TRAJ_VISUALIZER_H_
#define _TRAJ_VISUALIZER_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <assert.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <vector>
#include <stdlib.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"
#include "traj_container.hpp"


using std::vector;

namespace plan_utils
{
  class TrajVisualizer 
  {
  public:
    TrajVisualizer(ros::NodeHandle nh, int node_id);
    ~TrajVisualizer() {}

    void displayGoalPoint(Eigen::Vector2d goal_point, Eigen::Vector4d color, const double scale, int id);

    void displayGlobalPathList(vector<Eigen::Vector2d> global_pts, const double scale, int id);

    void displayInitPathList(vector<Eigen::Vector2d> init_pts, const double scale, int id);

    void displayMultiInitPathList(vector<vector<Eigen::Vector2d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayInnerList(Eigen::MatrixXd inner_pts, int id);

    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector2d> &list, double scale, Eigen::Vector4d color, int id);
    
    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
    
    void displaySikangPoly(vec_E<Polyhedron2D> polys);

    void displayObs(vec_Vec2f vec_obs);
    void displayPolyH(const std::vector<Eigen::MatrixXd> hPolys);
    void displayPolyTraj(std::unique_ptr<SingulTrajData> &display_traj);
    void displayKinoPath(std::unique_ptr<KinoTrajData> kino_trajs);
    void displayFitSurTrajs(plan_utils::SurroundTrajData sur_trajs);


  private:

    ros::NodeHandle nh_;
    int node_id_;

    decimal_t start_time_;


    ros::Publisher ego_vehicle_pub_;
    ros::Publisher forward_trajs_pub_;
    ros::Publisher sur_vehicle_trajs_pub_;

    ros::Publisher goal_point_pub;
    ros::Publisher optimal_list_pub, failed_list_pub, inner_list_pub, traj_path_pub,wholebody_traj_pub, path_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher sur_fitTraj_pub;

    ros::Publisher sikang_poly_pub_, galaxy_poly_pub_, obs_pub_;

    ros::Publisher intermediate_pt0_pub;
    ros::Publisher intermediate_pt1_pub;
    ros::Publisher intermediate_grad0_pub;
    ros::Publisher intermediate_grad1_pub;
    ros::Publisher intermediate_grad_smoo_pub;
    ros::Publisher intermediate_grad_dist_pub;
    ros::Publisher intermediate_grad_feas_pub;

    typedef std::shared_ptr<TrajVisualizer> Ptr;

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector2d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
                           
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector2d> &list, double scale, Eigen::Vector4d color, int id);

    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector2d> &list, double scale, Eigen::Vector4d color, int id);

    int last_traj_list_marker_cnt_ = 0;
    int last_surrounding_vehicle_marker_cnt_ = 0;
    int last_sur_vehicle_traj_mk_cnt = 0;
    int last_forward_traj_mk_cnt = 0;

    common::VehicleParam vp_;


  };  // TrajVisualizer
}  // namespace plan_utils

#endif  // _UTIL_TRAJ_PLANNER_VISUALIZER_H_