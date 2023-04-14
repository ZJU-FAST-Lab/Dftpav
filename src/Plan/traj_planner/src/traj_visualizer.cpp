
/*
 * To visualize all the trajectories and intermediate points
 * by zx
 */

#include "plan_utils/traj_visualizer.h"
#include <nav_msgs/Path.h>


namespace plan_utils
{


  TrajVisualizer::TrajVisualizer(ros::NodeHandle nh, int node_id): nh_(nh), node_id_(node_id) 
  {
    std::cout << "node_id_ = " << node_id_ << std::endl;

    std::string ego_optimal_list_topic = std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/minco/ego_optimal_list");
    std::string ego_failed_list_topic = std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/minco/ego_failed_list");
    std::string ego_inner_list_topic =  std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/minco/ego_inner_list");
    std::string traj_topic =  std::string("/vis/agent_") +
                              std::to_string(node_id_) +
                              std::string("/minco/poly_traj");

    optimal_list_pub = nh_.advertise<visualization_msgs::Marker>(ego_optimal_list_topic, 2);
    failed_list_pub = nh_.advertise<visualization_msgs::Marker>(ego_failed_list_topic, 2);
    inner_list_pub = nh_.advertise<visualization_msgs::Marker>(ego_inner_list_topic, 2);
    sur_fitTraj_pub = nh_.advertise<visualization_msgs::MarkerArray>("/vis/fit_sur_trajs",10);
    traj_path_pub = nh_.advertise<nav_msgs::Path>(traj_topic, 2);
    wholebody_traj_pub = nh_.advertise<visualization_msgs::Marker>(traj_topic+std::string("_wholebody"), 2);
    //path_pub = nh.advertise<nav_msgs::Path>("kino_trajs_pts", 1, true);
    path_pub = nh.advertise<visualization_msgs::Marker>("kino_trajs", 10);


    intermediate_pt0_pub = nh_.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    intermediate_grad0_pub = nh_.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    intermediate_pt1_pub = nh_.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    intermediate_grad1_pub = nh_.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    intermediate_grad_smoo_pub = nh_.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    intermediate_grad_dist_pub = nh_.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    intermediate_grad_feas_pub = nh_.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);

    std::string sikang_polyhedron = std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/minco/sikang_polyhedron");

    std::string galaxy_polyhedron = std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/minco/galaxy_polyhedron");

    sikang_poly_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>(sikang_polyhedron, 1, true);
    galaxy_poly_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>(galaxy_polyhedron, 1, true);
  

    obs_pub_ = nh.advertise<visualization_msgs::Marker>("cloud_point", 1, true);

  }

  void TrajVisualizer::displayFitSurTrajs(plan_utils::SurroundTrajData sur_trajs){
    visualization_msgs::MarkerArray surtrajs;
    
    for(int i = 0; i < sur_trajs.size(); i++){
      visualization_msgs::Marker traj;
      traj.action = visualization_msgs::Marker::ADD;
      traj.id = i;
      traj.type = visualization_msgs::Marker::SPHERE_LIST;
      traj.pose.orientation.w = 1.00;
      traj.color.r = 0.00;
      traj.color.g = 0.00;
      traj.color.b = 1.00;
      traj.color.a = 1.00;
      traj.scale.x = 0.2;
      traj.scale.y = 0.2;
      traj.scale.z = 0.2;
      traj.header.frame_id = "map";
      traj.header.stamp =ros::Time().now();
      for(double t = 0.0; t <= sur_trajs[i].traj.getTotalDuration(); t+=0.02){
        geometry_msgs::Point point;
        point.x =  sur_trajs[i].traj.getPos(t)[0];
        point.y =  sur_trajs[i].traj.getPos(t)[1];        
        point.z = 0.0;
        traj.points.push_back(point);
      }
      surtrajs.markers.push_back(traj);
    }
    sur_fitTraj_pub.publish(surtrajs);
  }
  void TrajVisualizer::displayKinoPath(std::unique_ptr<KinoTrajData> kino_trajs)
  {


    visualization_msgs::Marker sphere, line_strip, carMarkers;
    sphere.header.frame_id = line_strip.header.frame_id = carMarkers.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = carMarkers.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    carMarkers.type = visualization_msgs::Marker::LINE_LIST;


    sphere.action = visualization_msgs::Marker::DELETE;
    line_strip.action = visualization_msgs::Marker::DELETE;
    carMarkers.action = visualization_msgs::Marker::DELETE;

    path_pub.publish(sphere);
    path_pub.publish(line_strip);
    path_pub.publish(carMarkers);

    sphere.action = line_strip.action = carMarkers.action = visualization_msgs::Marker::ADD;
    sphere.id = 0;
    line_strip.id = 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.a = line_strip.color.a = 0.5;
    sphere.scale.x = 0.5;
    sphere.scale.y = 0.5;
    sphere.scale.z = 0.5;
    line_strip.scale.x = 0.25;


    carMarkers.id = 20;
    carMarkers.pose.orientation.w = 1.00;
    carMarkers.ns = "libaicorridorF";
    carMarkers.color.r = 1.00;
    carMarkers.color.g = 0.00;
    carMarkers.color.b = 1.00;
    carMarkers.color.a = 1.00;
    carMarkers.scale.x = 0.05;


    geometry_msgs::Point pt;
    unsigned int size = kino_trajs->size();
    for (unsigned int i = 0; i < size; ++i){
      sphere.color.r = line_strip.color.r = i*1.0/(size*1.0);
      sphere.color.g = line_strip.color.g = 0.0;
      sphere.color.b = line_strip.color.b = i*1.0/(size*1.0);

      for (int k = 0; k < kino_trajs->at(i).traj_pts.size(); k++)
      //kino_trajs->at(i).traj_pts
      {
        Eigen::Vector3d trajpt = kino_trajs->at(i).traj_pts[k];
        double yaw = kino_trajs->at(i).thetas[k];
        pt.x = trajpt(0);
        pt.y = trajpt(1);
        pt.z = 0.1;
        sphere.points.push_back(pt);
        line_strip.points.push_back(pt);



        geometry_msgs::Point point1;
        geometry_msgs::Point point2;
        geometry_msgs::Point point3;
        geometry_msgs::Point point4;
        Eigen::Matrix2d R;
        R << cos(yaw),-sin(yaw),
             sin(yaw),cos(yaw);
        Eigen::Vector2d offset1, tmp1;
        offset1 = R*Eigen::Vector2d(vp_.length()/2.0+vp_.d_cr(),vp_.width()/2.0);
        tmp1 = trajpt.head(2)+offset1;
        point1.x = tmp1[0]; 
        point1.y = tmp1[1];
        point1.z = 0;

        Eigen::Vector2d offset2, tmp2;
        offset2 = R*Eigen::Vector2d(vp_.length()/2.0+vp_.d_cr(),-vp_.width()/2.0);
        tmp2 = trajpt.head(2)+offset2;
        point2.x = tmp2[0]; 
        point2.y = tmp2[1];
        point2.z = 0;

        Eigen::Vector2d offset3, tmp3;
        offset3 = R*Eigen::Vector2d(-vp_.length()/2.0+vp_.d_cr(),-vp_.width()/2.0);
        tmp3 = trajpt.head(2)+offset3;
        point3.x = tmp3[0]; 
        point3.y = tmp3[1];
        point3.z = 0;

        Eigen::Vector2d offset4, tmp4;
        offset4 = R*Eigen::Vector2d(-vp_.length()/2.0+vp_.d_cr(),vp_.width()/2.0);
        tmp4 = trajpt.head(2)+offset4;
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
    }

    path_pub.publish(sphere);
    path_pub.publish(line_strip);
    // path_pub.publish(carMarkers);
  }



  // void TrajVisualizer::displayKinoPath(std::unique_ptr<KinoTrajData> kino_trajs){

  //   nav_msgs::Path path_msg;


  //   for (unsigned int i = 0; i < kino_trajs->size(); ++i ){
  //     for (const auto &it : kino_trajs->at(i).traj_pts)
  //     {
  //       geometry_msgs::PoseStamped pose;
  //       pose.pose.position.x = it(0);
  //       pose.pose.position.y = it(1);
  //       pose.pose.position.z = 0.05;
  //       pose.pose.orientation.w = 1.0;
  //       pose.pose.orientation.x = 0.0;
  //       pose.pose.orientation.y = 0.0;
  //       pose.pose.orientation.z = 0.0;

  //       path_msg.poses.push_back(pose);
  //     }
  //   }
  //   path_msg.header.frame_id = "map";
  //   path_pub.publish(path_msg);
  // }

  void TrajVisualizer::displayObs(vec_Vec2f vec_obs) {

    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;

    obs_pub_.publish(mk);

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1;
    mk.color.g = 0;
    mk.color.b = 0;
    mk.color.a = 1;

    mk.scale.x = 0.8;
    mk.scale.y = 0.8;
    mk.scale.z = 0.8;

    geometry_msgs::Point pt;

    for (size_t i = 0; i < vec_obs.size(); ++i)
    {
      pt.x = vec_obs[i](0);
      pt.y = vec_obs[i](1);
      pt.z = 0.0;
      mk.points.push_back(pt);
      //std::cout << "mk.points is " << pt << std::endl;
    }

    obs_pub_.publish(mk);

  }

  
  // verification for PolyH visualizations
  void TrajVisualizer::displayPolyH(const std::vector<Eigen::MatrixXd> hPolys){

    vec_E<Polyhedron2D> polyhedra;
    polyhedra.reserve(hPolys.size());
    for (const auto &ele : hPolys)
    {
      Polyhedron2D hPoly;
      for (int i = 0; i < ele.cols(); i++)
      {
        hPoly.add(Hyperplane2D(ele.col(i).tail<2>(), ele.col(i).head<2>()));
      }
      polyhedra.push_back(hPoly);
    }

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = ros::Time::now();
    galaxy_poly_pub_.publish(poly_msg);

  }


  void TrajVisualizer::displaySikangPoly(vec_E<Polyhedron2D> polys){
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
    poly_msg.header.frame_id = "map";
    sikang_poly_pub_.publish(poly_msg);
  }


  // // real ids used: {id, id+1000}
  void TrajVisualizer::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector2d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;


    sphere.action = visualization_msgs::Marker::DELETE;
    line_strip.action = visualization_msgs::Marker::DELETE;

    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);

    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = 0.1;
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void TrajVisualizer::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector2d> &list, 
                                                       double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = 0.0;
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void TrajVisualizer::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector2d> &list, 
                                                        double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = 0.0;
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = 0.0;
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }





void TrajVisualizer::displayPolyTraj(std::unique_ptr<SingulTrajData> &display_traj){


  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  for (unsigned int i = 0; i < display_traj->size(); ++i){

    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.01){

      Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
      pose.pose.position.x = pt(0);
      pose.pose.position.y = pt(1);
      pose.pose.position.z = 0.4;
      path_msg.poses.push_back(pose);
    }

  }

  path_msg.header.frame_id = "map";
  traj_path_pub.publish(path_msg);

  double last_debugyaw =  display_traj->at(0).traj.getAngle(0.0);


  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.01){

      Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
      pose.pose.position.x = pt(0);
      pose.pose.position.y = pt(1);
      pose.pose.position.z = 0.4;
      path_msg.poses.push_back(pose);
      Eigen::Vector2d vel = display_traj->at(i).traj.getdSigma(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      // std::cout<<"pos: "<<pt.transpose()<<" vel: "<<vel.transpose()<<" yaw: "<<yaw<<std::endl;

      // if(fabs(yaw-last_debugyaw)>0.2){
      // }
      last_debugyaw = yaw;
    }

  }
  visualization_msgs::Marker carMarkers;
  carMarkers.header.frame_id = "map";
  carMarkers.header.stamp = ros::Time::now();
  carMarkers.type = visualization_msgs::Marker::LINE_LIST;
  carMarkers.action = visualization_msgs::Marker::DELETE;
  wholebody_traj_pub.publish(carMarkers);
  carMarkers.action = visualization_msgs::Marker::ADD;
  carMarkers.id = 21;
  carMarkers.pose.orientation.w = 1.00;
  carMarkers.ns = "trajwholepub";
  carMarkers.color.r = 1.00;
  carMarkers.color.g = 0.00;
  carMarkers.color.b = 1.00;
  carMarkers.color.a = 1.00;
  carMarkers.scale.x = 0.05;
  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.1){
      Eigen::Vector2d pos = display_traj->at(i).traj.getPos(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.1;
      geometry_msgs::Point point1;
      geometry_msgs::Point point2;
      geometry_msgs::Point point3;
      geometry_msgs::Point point4;
      Eigen::Matrix2d R;
      R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
      Eigen::Vector2d offset1, tmp1;
      offset1 = R*Eigen::Vector2d(vp_.length()/2.0+vp_.d_cr(),vp_.width()/2.0);
      tmp1 = pos+offset1;
      point1.x = tmp1[0]; 
      point1.y = tmp1[1];
      point1.z = 0;

      Eigen::Vector2d offset2, tmp2;
      offset2 = R*Eigen::Vector2d(vp_.length()/2.0+vp_.d_cr(),-vp_.width()/2.0);
      tmp2 = pos+offset2;
      point2.x = tmp2[0]; 
      point2.y = tmp2[1];
      point2.z = 0;

      Eigen::Vector2d offset3, tmp3;
      offset3 = R*Eigen::Vector2d(-vp_.length()/2.0+vp_.d_cr(),-vp_.width()/2.0);
      tmp3 = pos+offset3;
      point3.x = tmp3[0]; 
      point3.y = tmp3[1];
      point3.z = 0;

      Eigen::Vector2d offset4, tmp4;
      offset4 = R*Eigen::Vector2d(-vp_.length()/2.0+vp_.d_cr(),vp_.width()/2.0);
      tmp4 = pos+offset4;
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

  }

  wholebody_traj_pub.publish(carMarkers);
}




////////////////////////////////////////////////////////////////////////////////////////////////
  void TrajVisualizer::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector2d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector2d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1.0, 0.33, 1.0, 1.0); //rgba
    displayMarkerList(optimal_list_pub, list, 0.6, color, id);
  }


  void TrajVisualizer::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector2d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector2d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1.0, 0.0, 0.0, 1.0);
    displayMarkerList(failed_list_pub, list, 0.6, color, id);
  }


  void TrajVisualizer::displayInnerList(Eigen::MatrixXd inner_pts, int id)
  {

    if (inner_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector2d> list;
    for (int i = 0; i < inner_pts.cols(); i++)
    {
      Eigen::Vector2d pt = inner_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.9, 0.9, 0.9, 1.0);
    displayMarkerList(inner_list_pub, list, 0.8, color, id);
  }

////////////////////////////////////////////////////////////////////////////////////////////////


  void TrajVisualizer::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector2d> &list, 
                                            double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  void TrajVisualizer::displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector2d> pts_;
    pts_.reserve(pts.cols());
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));
    }

    if ( !type.compare("0") )
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void TrajVisualizer::displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector2d> arrow_;
    arrow_.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }

    
  }
}  // namespace planning