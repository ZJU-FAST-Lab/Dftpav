/**
 * @file map_adapter.h
 * @author HKUST Aerial Robotics Group
 * @brief map adapter (inherits map interface) for Scc planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

/**
 * @file map_adapter.h revised version
 * @author Yuwei, from ZJU fast lab
 * @brief map adapter for minco planner
 * @version 0.2
 * @date 2021-04
 * @copyright Copyright (c) 2021
 */


#ifndef _UTIL_TRAJ_PLANNER_MAP_ADAPTER_H_
#define _UTIL_TRAJ_PLANNER_MAP_ADAPTER_H_

#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "map_interface.h"

namespace map_utils {

class TrajPlannerAdapter : public TrajPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  bool IsValid() override;
  decimal_t GetTimeStamp() override;
  ErrorType GetEgoVehicle(Vehicle* vehicle) override;
  ErrorType GetEgoState(State* state) override;
  ErrorType GetEgoReferenceLane(Lane* lane) override;
  ErrorType GetLocalReferenceLane(Lane* lane) override;
  ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) override;
  ErrorType GetObstacleMap(GridMap2D* grid_map) override;
  ErrorType CheckCollisionUsingGlobalPosition(Eigen::Vector2d& pos, bool* res) override;
  ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                             const State& state, bool* res) override;
  ErrorType CheckIfCollisionUsingPosAndYaw(const common::VehicleParam& vehicle_param, 
                                           const Eigen::Vector3d &state, bool* res) override;
  
  ErrorType CheckIfCollisionUsingLine(const Eigen::Vector2d p1, 
                                           const Eigen::Vector2d p2, bool* res,double checkl) override;

  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) override;
  ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) override;
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs)
      override;
  ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) override;
  ErrorType GetMovingObsTraj(
      std::vector<std::vector<common::State>>* sur_trajs) override;
  ErrorType set_map(std::shared_ptr<IntegratedMap> map);


 private:
  std::shared_ptr<IntegratedMap> map_;
  bool is_valid_ = false;
};

}  // namespace map_utils

#endif