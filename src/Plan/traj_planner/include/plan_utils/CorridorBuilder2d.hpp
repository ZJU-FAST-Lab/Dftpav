#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <random>

#include <Eigen/Eigen>


namespace plan_utils
{

  void corridorBuilder2d(Eigen::Vector2d origin, float radius, float max_x, float max_y,
                        vec_Vec2f &data, std::vector<Eigen::Vector2d> &add_data, 
                        Eigen::MatrixXd &hPoly) {

    //max_x max_y may be the local map?
    //why add_data?
    //radius = inf?
    //origin is the root point of the poly
      float interior_x = 0.0;
      float interior_y = 0.0;
      float safe_radius = radius;
      std::vector<cv::Point2f> flipData;

      cv::Point2f point;
      vec_Vec2f new_data;


      for (size_t i=0; i<data.size(); i++) {
          float dx = data[i](0) - origin(0);
          float dy = data[i](1) - origin(1);
          float norm2 = std::sqrt(dx*dx + dy*dy);
          if ( abs(dx) > max_x || abs(dy) > max_y) {
            continue;
          }
          if (norm2 < safe_radius) safe_radius = norm2; //safe radius is the nearest distance between the root and obs
          if (norm2 == 0) continue;
          
          point.x = dx + 2*(radius-norm2)*dx/norm2;
          point.y = dy + 2*(radius-norm2)*dy/norm2; //radius is an  enough large value
          new_data.push_back(data[i]);
          flipData.push_back(point);
      }

      for (size_t i=0; i<add_data.size(); i++) {
          float dx = add_data[i](0) - origin(0);
          float dy = add_data[i](1) - origin(1);
          float norm2 = std::sqrt(dx*dx + dy*dy);
          if (norm2 < safe_radius) safe_radius = norm2;
          if (norm2 == 0) continue;
          point.x = dx + 2*(radius-norm2)*dx/norm2;
          point.y = dy + 2*(radius-norm2)*dy/norm2;
          new_data.push_back(add_data[i]);
          flipData.push_back(point);
      }

      std::vector<int> vertexIndice;
      cv::convexHull(flipData,vertexIndice,false,false);
      //obtain the poly containing flipData
      
      bool isOriginAVertex = false;
      int OriginIndex = -1;
      std::vector<cv::Point2f> vertexData;
      for (size_t i=0; i<vertexIndice.size(); i++) {
          unsigned int v = vertexIndice[i];
          if (v == new_data.size()) {
              isOriginAVertex = true;
              OriginIndex = i;
              vertexData.push_back(cv::Point2f(origin(0), origin(1)));
          }else {
              vertexData.push_back(cv::Point2f(new_data[v](0), new_data[v](1)));
          }
      }

      if (isOriginAVertex) {
          int last_index = (OriginIndex - 1)%vertexIndice.size();
          int next_index = (OriginIndex + 1)%vertexIndice.size();
          float dx = (new_data[vertexIndice[last_index]](0) + origin(0) + new_data[vertexIndice[next_index]](0))/3 - origin(0);
          float dy = (new_data[vertexIndice[last_index]](1) + origin(1) + new_data[vertexIndice[next_index]](1))/3 - origin(1);
          float d = std::sqrt(dx*dx + dy*dy);
          interior_x = 0.99*safe_radius*dx/d + origin(0);
          interior_y = 0.99*safe_radius*dy/d + origin(1);
      }else {
          interior_x = origin(0);
          interior_y = origin(1);
      }

      std::vector<int> vIndex2;
      cv::convexHull(vertexData,vIndex2,false,false); // counterclockwise right-hand

      

      std::vector<Eigen::Vector3f> constraints; // (a,b,c) a x + b y <= c
      for (size_t j=0; j<vIndex2.size(); j++) {
          int jplus1 = (j+1)%vIndex2.size();
          cv::Point2f rayV = vertexData[vIndex2[jplus1]] - vertexData[vIndex2[j]];
          Eigen::Vector2f normalJ(rayV.y, -rayV.x);  // point to outside
          normalJ.normalize();
          int indexJ = vIndex2[j];
          while (indexJ != vIndex2[jplus1]) {
              float c = (vertexData[indexJ].x-interior_x) * normalJ(0) + (vertexData[indexJ].y-interior_y) * normalJ(1);
              constraints.push_back(Eigen::Vector3f(normalJ(0), normalJ(1), c));
              indexJ = (indexJ+1)%vertexData.size();
          }
      }    

      std::vector<cv::Point2f> dualPoints(constraints.size(), cv::Point2f(0,0));
      for (size_t i=0; i<constraints.size(); i++) {
          dualPoints[i].x = constraints[i](0)/constraints[i](2);
          dualPoints[i].y = constraints[i](1)/constraints[i](2);
      }
      
      std::vector<cv::Point2f> dualVertex, finalVertex;
      cv::convexHull(dualPoints,dualVertex,true,false);

      for (size_t i=0; i<dualVertex.size(); i++) {
          int iplus1 = (i+1)%dualVertex.size();
          cv::Point2f rayi = dualVertex[iplus1] - dualVertex[i];
          float c = rayi.y*dualVertex[i].x - rayi.x*dualVertex[i].y;
          finalVertex.push_back(cv::Point2f(interior_x+rayi.y/c, interior_y-rayi.x/c));
      }

      unsigned int size = finalVertex.size();
      hPoly.resize(4, size);
      for (unsigned int i = 0; i < size; i++){
        int iplus1 = (i+1)%size;
        cv::Point2f rayi = finalVertex[iplus1] - finalVertex[i];           
        hPoly.col(i).tail<2>()  = Eigen::Vector2d(finalVertex[i].x, finalVertex[i].y);  // the points on the plane
        hPoly.col(i).head<2>()  = Eigen::Vector2d(-rayi.y, rayi.x); // outside
      }
      
  }

} // namespace plan_utils