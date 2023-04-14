#ifndef _TRAJ_CONTAINER_H_
#define _TRAJ_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include "poly_traj_utils.hpp"


using std::vector;


namespace plan_utils
{


  struct FlatTrajData
  {
    int singul;
    std::vector<Eigen::Vector3d> traj_pts;      // 3, N  x,y dur
    std::vector<double> thetas;
    Eigen::MatrixXd start_state;   // start flat state (2, 3)
    Eigen::MatrixXd final_state;   // end flat state (2, 3)

  };

  struct LocalTrajData
  {
    Trajectory traj;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time 
    double end_time;   // world time
    Eigen::Vector2d start_pos;
    double init_angle;
  };


  typedef std::vector<LocalTrajData> SurroundTrajData;
  typedef std::vector<LocalTrajData> SingulTrajData;
  typedef std::vector<FlatTrajData> KinoTrajData;


  class TrajContainer
  {
  public:
    
    SurroundTrajData surround_traj;
    int traj_id = 0;

    TrajContainer(){}
    ~TrajContainer() {}

    SingulTrajData singul_traj;

    void addSingulTraj(const Trajectory &trajectory, const double &world_time, const int drone_id = -1)
    {

      LocalTrajData local_traj;

      local_traj.drone_id = drone_id;
      local_traj.traj_id = ++traj_id;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.end_time = world_time + local_traj.duration;
      local_traj.traj = trajectory;

      singul_traj.push_back(local_traj);

    }

    void clearSingul(){

      singul_traj.clear();

    }

  };

} // namespace plan_utils

#endif