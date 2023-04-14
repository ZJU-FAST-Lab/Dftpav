/**
 * @file map_interface.h
 * @author HKUST Aerial Robotics Group
 * @brief map interface for ssc planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

/**
 * @file map interface.h revised version
 * @author Yuwei, from ZJU fast lab
 * @brief map interface for minco planner
 * @version 0.2
 * @date 2021-04
 * @copyright Copyright (c) 2021
 */

#ifndef _UTIL_TRAJ_PLANNER_MAP_INTERFACE_H_
#define _UTIL_TRAJ_PLANNER_MAP_INTERFACE_H_

#include <array>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace map_utils {
class TrajPlannerMapItf {
 public:
  using ObstacleMapType = uint8_t;
  using State = common::State;
  using Lane = common::Lane;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using Behavior = common::SemanticBehavior;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  virtual bool IsValid() = 0;
  virtual decimal_t GetTimeStamp() = 0;
  virtual ErrorType GetEgoVehicle(Vehicle* vehicle) = 0;
  virtual ErrorType GetEgoState(State* state) = 0;
  virtual ErrorType GetEgoReferenceLane(Lane* lane) = 0;
  virtual ErrorType GetLocalReferenceLane(Lane* lane) = 0;
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) = 0;
  virtual ErrorType GetObstacleMap(GridMap2D* grid_map) = 0;
  virtual ErrorType CheckCollisionUsingGlobalPosition(Eigen::Vector2d& pos, bool* res) = 0;
  virtual ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                                     const State& state, bool* res) = 0;
  virtual ErrorType CheckIfCollisionUsingPosAndYaw(const common::VehicleParam& vehicle_param, 
                                                   const Eigen::Vector3d &state, bool* res) = 0;
  
  virtual ErrorType CheckIfCollisionUsingLine(const Eigen::Vector2d p1, 
                                           const Eigen::Vector2d p2, bool* res,double checkl) = 0;                                    
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) = 0;
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<Vehicle>>>* sur_trajs) = 0;
  virtual ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) = 0;
  virtual ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) = 0;
  virtual ErrorType GetMovingObsTraj(
      std::vector<std::vector<common::State>>* sur_trajs) = 0;

};

}  // namespace map_utils

#endif  // _UTIL_TRAJ_PLANNER_MAP_INTERFACE_H_