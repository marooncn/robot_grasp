#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/console.h>
#include "ros/ros.h"
#include "robot_grasp/calibration_transform.h"

class calib_trans
{
public:
  explicit calib_trans(ros::NodeHandle nh)
    : nh_(nh)
  {
    trans_service = nh_.advertiseService("/transform", &calib_trans::transform, this);
    Intrinsic_matrix<<1068.6277, 0, 947.266,
                      0, 1069.2998, 577.550,
                      0, 0, 1.0;
  // Extrinsic_matrix<<0, -1, 0, 0, 1, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 1;
  Extrinsic_matrix<<0.05067265, -0.99716277, 0.05566597, -0.52,
                   -0.99558933, -0.05484203, -0.07611985, -0.59,
                    0.07895671, -0.05156326, -0.9955436, 0.713,
                      0, 0, 0, 1;
    cx=947.266;
    cy=577.551;
    fx=1068.63;
    fy=1069.230;
    depth=0.713;
  }

public:
  ros::NodeHandle nh_;
  ros::ServiceServer trans_service;
  Eigen::Matrix3d Intrinsic_matrix;
  Eigen::Matrix4d Extrinsic_matrix;
  Eigen::Vector4d projection_point,grasp_point;

  float depth;
  float cx;
  float cy;
  float fx;
  float fy;
  float xm, ym; //image co
  float x1, x2, x3;  //projected point

  bool transform(robot_grasp::calibration_transform::Request &req,
           robot_grasp::calibration_transform::Response &res)
  {

    x3 = depth;
    std::vector<float> res_obj_param;

    xm = req.obj_real_detected_in_cam.pose[0];
    ym = req.obj_real_detected_in_cam.pose[1];

    float Orientation = req.obj_real_detected_in_cam.pose[2];
    x1 = (xm-cx)*depth/fx;
    x2 = (ym-cy)*depth/fy;

    projection_point<<x1, x2, depth,1.0;
    grasp_point = Extrinsic_matrix*projection_point;
    res_obj_param.push_back(grasp_point[0]);
    res_obj_param.push_back(grasp_point[1]);
    res_obj_param.push_back(grasp_point[2]);
    res_obj_param.push_back(Orientation);
    res.obj_real_detected_in_base = res_obj_param;
    return true;

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calib_trans_");
  ros::NodeHandle nh("~");

  calib_trans nfe(nh);

  while (ros::ok())
    ros::spin();

  return 0;
}

