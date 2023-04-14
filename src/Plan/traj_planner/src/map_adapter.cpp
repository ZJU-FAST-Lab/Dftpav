#include "map_utils/map_adapter.h"

namespace map_utils {

ErrorType TrajPlannerAdapter::set_map(std::shared_ptr<IntegratedMap> map) {
  map_ = map;  // maintain a snapshop of the environment
  is_valid_ = true;
  return kSuccess;
}

bool TrajPlannerAdapter::IsValid() { return is_valid_; }

decimal_t TrajPlannerAdapter::GetTimeStamp() { return map_->time_stamp(); }

ErrorType TrajPlannerAdapter::GetEgoVehicle(Vehicle* vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetEgoState(State* state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetLocalReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  *sur_trajs = map_->ego_behavior().surround_trajs;
  
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetEgoDiscretBehavior(
    LateralBehavior* lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().lat_behavior == common::LateralBehavior::kUndefined)
    return kWrongStatus;
  *lat_behavior = map_->ego_behavior().lat_behavior;
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetLaneByLaneId(const int lane_id, Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
  }
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetEgoReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    // printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

ErrorType TrajPlannerAdapter::GetObstacleMap(GridMap2D* grid_map) {
  if (!is_valid_) return kWrongStatus;
  *grid_map = map_->obstacle_map();
  return kSuccess;
}
ErrorType TrajPlannerAdapter::CheckCollisionUsingGlobalPosition(Eigen::Vector2d& pos, bool* res){
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingGlobalPosition(pos, res);
  return kSuccess;
}
ErrorType TrajPlannerAdapter::CheckIfCollision(
    const common::VehicleParam& vehicle_param, const State& state, bool* res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

ErrorType TrajPlannerAdapter::CheckIfCollisionUsingPosAndYaw(
    const common::VehicleParam& vehicle_param, const Eigen::Vector3d &state, bool* res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingPosAndYaw(vehicle_param, state, res);
  return kSuccess;
}

ErrorType TrajPlannerAdapter::CheckIfCollisionUsingLine(const Eigen::Vector2d p1, 
                                           const Eigen::Vector2d p2, bool* res, double checkl){
  if (!is_valid_) return kWrongStatus;    
  for(double dl = 0.0; dl < (p2-p1).norm(); dl+=checkl){
    Eigen::Vector2d pos = (p2-p1)*dl/(p2-p1).norm()+p1;
    map_->CheckCollisionUsingGlobalPosition(pos,res);  
    if(*res){
      return kSuccess;
    }
  }
  map_->CheckCollisionUsingGlobalPosition(p2,res);
  return kSuccess;
}
ErrorType TrajPlannerAdapter::GetMovingObsTraj(
      std::vector<std::vector<common::State>>* sur_trajs){
  if (!is_valid_) return kWrongStatus;
  *sur_trajs = map_->movingObstraj();
  return kSuccess;
}
ErrorType TrajPlannerAdapter::GetObstacleGrids(
    std::set<std::array<decimal_t, 2>>* obs_grids) {
  if (!is_valid_) return kWrongStatus;
  *obs_grids = map_->obstacle_grids();
  return kSuccess;
}

}  // namespace map_utils