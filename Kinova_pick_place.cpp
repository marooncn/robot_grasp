#include <string>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <gazebo_msgs/GetModelState.h>
#include <robot_grasp/PickPlace.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include "robot_grasp/Detected_obj_param.h"
#include "robot_grasp/Detected_obj_param_Array.h"

const double FINGER_MAX = 6400;
const std::string KINOVA_PLANNING_GROUP = "arm";
const std::string KINOVA_GRIPPER_GROUP = "gripper";
const std::string DROPBOX_MESH_PATH = "package://robot_grasp/models/grasp_object/basket.dae";

class kinova_pick_place
{
public:
  explicit kinova_pick_place(ros::NodeHandle nh,
                    const std::string KINOVA_PLANNING_GROUP_,
                    const std::string KINOVA_GRIPPER_GROUP_,
                    const std::string DROPBOX_MESH_PATH_
                    )
    : nh_(nh),
      kinova_move_group(KINOVA_PLANNING_GROUP_),
      kinova_gripper_group(KINOVA_GRIPPER_GROUP_),
      tfListener(tfBuffer)
  {
    robot_connected_ = false;
    kinova_joint_model_group = kinova_move_group.getCurrentState()->getJointModelGroup(KINOVA_PLANNING_GROUP);
    kinova_gripper_joint_model_group = kinova_gripper_group.getCurrentState()->getJointModelGroup(KINOVA_GRIPPER_GROUP);
    //目标物体
    //std::vector<std::string> object_ids;

    //planning scene 中的障碍物
    std::vector<moveit_msgs::CollisionObject> collision_object_list;
    moveit_msgs::CollisionObject green_dropbox_collision_object,blue_dropbox_collision_object;
    geometry_msgs::Pose green_mesh_pose, blue_mesh_pose;

    green_mesh_pose.position.x = 0;
    green_mesh_pose.position.y = -0.71;
    green_mesh_pose.position.z = 0.605;
    green_mesh_pose.orientation.w = 0.707;
    green_mesh_pose.orientation.x = 0;
    green_mesh_pose.orientation.y = 0;
    green_mesh_pose.orientation.z = 0.707;

    blue_mesh_pose.position.x = 0;
    blue_mesh_pose.position.y = -0.71;
    blue_mesh_pose.position.z = 0.605;
    blue_mesh_pose.orientation.w = 0.707;
    blue_mesh_pose.orientation.x = 0;
    blue_mesh_pose.orientation.y = 0;
    blue_mesh_pose.orientation.z = 0.707;

    SetupCollisionObject("green_dropbox", DROPBOX_MESH_PATH_, green_mesh_pose,
                         green_dropbox_collision_object);
    SetupCollisionObject("blue_dropbox", DROPBOX_MESH_PATH_, blue_mesh_pose,
                         blue_dropbox_collision_object);

    collision_object_list.push_back(green_dropbox_collision_object);
    collision_object_list.push_back(blue_dropbox_collision_object);
    planning_scene_interface.addCollisionObjects(collision_object_list);
    /*kinova移动到home位置
    kinova_move_group.setNamedTarget("Home");
    kinova_move_group.move();
    ROS_INFO("Move to Home");*/
    ros::Duration(1.0).sleep();
    /*定义航点*/
    INTERPOLATION_WORLD_pose_1.position.x = 0.42894;
    INTERPOLATION_WORLD_pose_1.position.y = 0.43463;
    INTERPOLATION_WORLD_pose_1.position.z = 0.58683;
    INTERPOLATION_WORLD_pose_1.orientation.x = 0.50902;
    INTERPOLATION_WORLD_pose_1.orientation.y = 0.49672;
    INTERPOLATION_WORLD_pose_1.orientation.z = 0.48642;
    INTERPOLATION_WORLD_pose_1.orientation.w = 0.5075;

    INTERPOLATION_WORLD_pose_2.position.x = 0.42141;
    INTERPOLATION_WORLD_pose_2.position.y = 0.45139;
    INTERPOLATION_WORLD_pose_2.position.z = 0.58812;
    INTERPOLATION_WORLD_pose_2.orientation.x = 0.62353;
    INTERPOLATION_WORLD_pose_2.orientation.y = 0.62799;
    INTERPOLATION_WORLD_pose_2.orientation.z = 0.32513;
    INTERPOLATION_WORLD_pose_2.orientation.w = 0.33337;

    INTERPOLATION_WORLD_pose_3.position.x = 0.37997;
    INTERPOLATION_WORLD_pose_3.position.y = 0.435;
    INTERPOLATION_WORLD_pose_3.position.z = 0.30845;
    INTERPOLATION_WORLD_pose_3.orientation.x = 0.61752;
    INTERPOLATION_WORLD_pose_3.orientation.y = 0.60746;
    INTERPOLATION_WORLD_pose_3.orientation.z = 0.34296;
    INTERPOLATION_WORLD_pose_3.orientation.w = 0.3683;

    INTERPOLATION_WORLD_pose_4.position.x = 0.29027;
    INTERPOLATION_WORLD_pose_4.position.y = 0.44981;
    INTERPOLATION_WORLD_pose_4.position.z = 0.18757;
    INTERPOLATION_WORLD_pose_4.orientation.x = 0.70468;
    INTERPOLATION_WORLD_pose_4.orientation.y = 0.70949;
    INTERPOLATION_WORLD_pose_4.orientation.z = -0.0028649;
    INTERPOLATION_WORLD_pose_4.orientation.w = -0.0060894;

    INTERPOLATION_WORLD_pose_5.position.x = 0.59027;
    INTERPOLATION_WORLD_pose_5.position.y = 0.34981;
    INTERPOLATION_WORLD_pose_5.position.z = 0.12757;
    INTERPOLATION_WORLD_pose_5.orientation.x = 0.70468;
    INTERPOLATION_WORLD_pose_5.orientation.y = 0.70949;
    //INTERPOLATION_WORLD_pose_5.orientation.z = -0.0028649;
    //INTERPOLATION_WORLD_pose_5.orientation.w = -0.0060894;
    INTERPOLATION_WORLD_pose_5.orientation.z = 0;
    INTERPOLATION_WORLD_pose_5.orientation.w = 0;

    /*定义最终目标抓取姿态*/
    obj_grasp_pose.orientation.x = 0.70468;
    obj_grasp_pose.orientation.y = 0.70949;
    obj_grasp_pose.orientation.z = -0.0028649;
    obj_grasp_pose.orientation.w = -0.0060894;

    obj_grasp_pose_1.orientation.x = 0.70468;
    obj_grasp_pose_1.orientation.y = 0.70949;
    obj_grasp_pose_1.orientation.z = -0.0028649;
    obj_grasp_pose_1.orientation.w = -0.0060894;

    /*定义放置点*/
    place_pose.position.x = 0.577;
    place_pose.position.y = -0.20853;
    place_pose.position.z = 0.042587;
    place_pose.orientation.x = 0.70468;
    place_pose.orientation.y = 0.70949;
    place_pose.orientation.z = -0.0028649;
    place_pose.orientation.w = -0.0060894;
  }
  ~kinova_pick_place()
  {

  }
  bool SetupCollisionObject(const std::string &object_id,
                            const std::string &mesh_path,
                            const geometry_msgs::Pose &object_pose,
                            moveit_msgs::CollisionObject &collision_object)
  {
    collision_object.header.frame_id = kinova_move_group.getPlanningFrame();
    collision_object.id = object_id;

    shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);

    ROS_DEBUG_STREAM(object_id << " mesh loaded");

    shape_msgs::Mesh object_mesh;
    shapes::ShapeMsg object_mesh_msg;
    shapes::constructMsgFromShape(m, object_mesh_msg);
    object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.meshes[0] = object_mesh;

    collision_object.mesh_poses[0].position = object_pose.position;
    collision_object.mesh_poses[0].orientation = object_pose.orientation;

    collision_object.meshes.push_back(object_mesh);
    collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
    collision_object.operation = collision_object.ADD;
  }
  bool IsPickPoseWithinLimits(geometry_msgs::Pose &pick_pose,
                              geometry_msgs::Pose &act_obj_pose)
  {
    //Obtain actual pose of the object
    float dist_sq=0;

    dist_sq = (act_obj_pose.position.x-pick_pose.position.x)*(act_obj_pose.position.x-pick_pose.position.x)
      +(act_obj_pose.position.y-pick_pose.position.y)*(act_obj_pose.position.y-pick_pose.position.y)
      +(act_obj_pose.position.z-pick_pose.position.z)*(act_obj_pose.position.z-pick_pose.position.z);

    if(dist_sq < 0.03)
      return true;
    else
      return false;
  }
  tf::Quaternion RPYToQuaternion(float R, float P, float Y)
  {
    tf::Matrix3x3 mat;
    mat.setEulerYPR(Y,P,R);
    tf::Quaternion quat;
    mat.getRotation(quat);
    return quat;
  }
  bool OperateGripper(const bool &close_gripper)
  { 
    moveit::core::RobotStatePtr gripper_current_state = kinova_gripper_group.getCurrentState();
    std::vector<double> gripper_joint_positions;
    gripper_current_state->copyJointGroupPositions(kinova_gripper_joint_model_group,gripper_joint_positions);
    ROS_DEBUG("No. of joints in eef_group: %zd", gripper_joint_positions.size());
    if (close_gripper)
    {
      gripper_joint_positions[0] = 1.0;
      gripper_joint_positions[1] = 1.0;
      gripper_joint_positions[2] = 1.0;
      ROS_INFO("aaaaaaaaaaaaaaaaaaaaaaa");
    }
    else
    {
      gripper_joint_positions[0] = 0.7;
      gripper_joint_positions[1] = 0.7;
      gripper_joint_positions[2] = 0.7;
      ROS_INFO("bbbbbbbbbbbbbbbbbbbbbbb");
    }
    kinova_gripper_group.setJointValueTarget(gripper_joint_positions);
    ros::Duration(5).sleep();
    bool success = kinova_gripper_group.move();
    ros::Duration(5).sleep();
    return success;
  }
  bool gripper_action(double finger_turn)
  {
      if(robot_connected_ == false)
      {
          if (finger_turn>0.5*FINGER_MAX)
          {
            kinova_gripper_group.setNamedTarget("Close");
          }
          else
          {
            kinova_gripper_group.setNamedTarget("Open");
          }
          kinova_gripper_group.move();
          ros::Duration(1.0);
          return true;
      }

      if (finger_turn < 0)
      {
          finger_turn = 0.0;
      }
      else
      {
          finger_turn = std::min(finger_turn, FINGER_MAX);
      }

      kinova_msgs::SetFingersPositionGoal goal;
      goal.fingers.finger1 = finger_turn;
      goal.fingers.finger2 = goal.fingers.finger1;
      goal.fingers.finger3 = goal.fingers.finger1;
      finger_client_->sendGoal(goal);

      if (finger_client_->waitForResult(ros::Duration(5.0)))
      {
          finger_client_->getResult();
          return true;
      }
      else
      {
          finger_client_->cancelAllGoals();
          ROS_WARN_STREAM("The gripper action timed-out");
          return false;
      }
  }
  geometry_msgs::Pose cale_obj_pose_in_robot_base(float w_x,float w_y,float w_z,float R,float P,float Y)
  {
    geometry_msgs::Pose Obj_Pose;
    tf::Quaternion obj_quat;
    obj_quat = RPYToQuaternion(R,P,Y);
    try
    {
      geometry_msgs::PointStamped TEMP_WORLD_pt;
      TEMP_WORLD_pt.point.x = w_x;
      TEMP_WORLD_pt.point.y = w_y;
      TEMP_WORLD_pt.point.z = w_z;
      TEMP_WORLD_pt.header.frame_id = "/world";
      transformStamped = tfBuffer.lookupTransform("j2s7s300_link_base", "world", ros::Time(0),ros::Duration(3.0));
      geometry_msgs::PointStamped  TEMP_ROBOT_pt;
      TEMP_ROBOT_pt.header.frame_id = "/j2s7s300_link_base";
      tf2::doTransform(TEMP_WORLD_pt, TEMP_ROBOT_pt, transformStamped);
      Obj_Pose.position.x = TEMP_ROBOT_pt.point.x;
      Obj_Pose.position.y = TEMP_ROBOT_pt.point.y;
      Obj_Pose.position.z = TEMP_ROBOT_pt.point.z;
      Obj_Pose.orientation.x = obj_quat.getX();
      Obj_Pose.orientation.y = obj_quat.getY();
      Obj_Pose.orientation.z = obj_quat.getZ();
      Obj_Pose.orientation.w = obj_quat.getW();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return Obj_Pose;
  }
  void Robot_Joint_Reader()
  {
    std::vector<double> group_variable_values;
    kinova_move_group.getCurrentState()->copyJointGroupPositions(kinova_move_group.getCurrentState()->getRobotModel()->getJointModelGroup(kinova_move_group.getName()), group_variable_values);
    /*group_variable_values[5] = -1.0;
    kinova_move_group.setJointValueTarget(group_variable_values);
    success = kinova_move_group.plan(my_plan);
    sleep(5.0);*/
    for(int i=0;i<7;i++)
    {
      std::cout<<group_variable_values[i]<<std::endl;
    }
  }
  bool evaluate_plan()
  {
      bool replan = true;
      int count = 0;

      //moveit::planning_interface::MoveGroup::Plan my_plan;

      while (replan == true && ros::ok())
      {
          // reset flag for replan
          count = 0;
          result_ = false;

          // try to find a success plan.
          double plan_time;
          while (result_ == false && count < 5)
          {
              count++;
              plan_time = 20+count*10;
              ROS_INFO("Setting plan time to %f sec", plan_time);
              kinova_move_group.setPlanningTime(plan_time);
              kinova_move_group.setStartStateToCurrentState();
              result_ = kinova_move_group.plan(kinova_arm_plan);
              std::cout << "at attemp: " << count << std::endl;
              ros::WallDuration(0.1).sleep();
          }

          // found a plan
          if (result_ == true)
          {
              std::cout << "plan success at attemp: " << count << std::endl;

              replan = false;
              std::cout << "please input e to execute the plan, r to replan, others to skip: ";
              std::cin >> pause_;
              ros::WallDuration(0.5).sleep();
              if (pause_ == "r" || pause_ == "R" )
              {
                  replan = true;
              }
              else
              {
                  replan = false;
              }
          }
          else // not found
          {
              std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
              replan = false;
              break;
          }
      }

      if(result_ == true)
      {
          if (pause_ == "e" || pause_ == "E")
          {
              kinova_move_group.execute(kinova_arm_plan);
          }
      }
      ros::WallDuration(1.0).sleep();
      return result_;
  }
  void constraints_setup()
  {
    ocm.link_name = "j2s7s300__link_7";//需要约束的链接
    ocm.header.frame_id = "j2s7s300_link_base";//基坐标系
    //四元数约束
    ocm.orientation.w = 1.0;
    //欧拉角约束
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 2*3.14;
    ocm.weight = 1.0;//此限制权重
    endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表
    kinova_move_group.setPathConstraints(endEffector_constraints);
  }
  bool reach_obj()
  {
    waypoints.push_back(INTERPOLATION_WORLD_pose_2);
    waypoints.push_back(INTERPOLATION_WORLD_pose_3);
    waypoints.push_back(INTERPOLATION_WORLD_pose_4);
    waypoints.push_back(INTERPOLATION_WORLD_pose_5);
    double fraction = kinova_move_group.computeCartesianPath(waypoints,0.01,0.0,trajectory_msg,false);
    ROS_INFO("Reach Obj Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);
    kinova_move_group.clearPathConstraints();
    robot_trajectory::RobotTrajectory rt(kinova_move_group.getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*kinova_move_group.getCurrentState(), trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    move_success = iptp.computeTimeStamps(rt);
    ROS_INFO("Reach Obj Computed time stamp %s",move_success?"SUCCEDED":"FAILED");
    rt.getRobotTrajectoryMsg(trajectory_msg);
    kinova_arm_plan.trajectory_ = trajectory_msg;
    kinova_move_group.execute(kinova_arm_plan);
    ROS_INFO("Approach The Target Done");
    ros::WallDuration(1.0).sleep();
    return move_success;
  }
  bool grasp_place_obj()
  {

    robot_state::RobotState start_state(*kinova_move_group.getCurrentState());
    kinova_move_group.setStartState(start_state);
    kinova_move_group.setPoseTarget(obj_grasp_pose_1);
    move_success = evaluate_plan();
    if(move_success == false)
    {
      return false;
    }
    ros::WallDuration(1.0).sleep();
    kinova_move_group.setPoseTarget(obj_grasp_pose);
    move_success = evaluate_plan();
    if(move_success == false)
    {
      return false;
    }
//    ros::WallDuration(1.0).sleep();
    /*手爪闭合*/
    move_success = OperateGripper(false);
    ros::Duration(5.0).sleep();
//    ros::WallDuration(1.0).sleep();
//    kinova_move_group.setPoseTarget(place_pose);
//    move_success = evaluate_plan();
//    if(move_success == false)
//    {
//      return false;
//    }
//    ros::WallDuration(1.0).sleep();
    /*手爪张开*/
    move_success = OperateGripper(true);
//    ros::WallDuration(1.0).sleep();
    return move_success;
  }
  bool kinova_routine(robot_grasp::PickPlace::Request &req,robot_grasp::PickPlace::Response &res)
  {
    if(req.detected_obj_param_list.size()==0)
    {
      std::cout<<"No obj to be grasped !"<<std::endl;
      return true;
    }
    /*路径约束配置*/
    constraints_setup();

    /*kinova移动到指定位置*/
//    move_success = reach_obj();
//    if(move_success == false)
//    {
//      ROS_INFO("REACH OBJ FAILED");
//      res.success = move_success;
//      return false;
//    }

    detected_param_obj_list = req.detected_obj_param_list;

    for(int i=0;i<detected_param_obj_list.size();i++)
    {
      waypoints.clear();
      obj_grasp_label = detected_param_obj_list[i].obj_label;

      obj_grasp_pose_1.position.x = detected_param_obj_list[i].pcaCentroid[0];
      obj_grasp_pose_1.position.y = detected_param_obj_list[i].pcaCentroid[1];
      obj_grasp_pose_1.position.z = detected_param_obj_list[i].pcaCentroid[2];

      obj_grasp_pose.position.x = detected_param_obj_list[i].pcaCentroid[0];
      obj_grasp_pose.position.y = detected_param_obj_list[i].pcaCentroid[1];
      obj_grasp_pose.position.z = detected_param_obj_list[i].pcaCentroid[2];

      std::cout<<"CCCCCCCCCCCCC Grasp Obj "+obj_grasp_label+" at :"<<std::endl;
      std::cout<<obj_grasp_pose.position.x<<" ,"<<obj_grasp_pose.position.y<<" ,"<<obj_grasp_pose.position.z<<" ,"<<std::endl;
      /*抓目标*/
      move_success = grasp_place_obj();
    }
    res.success = move_success;
    return move_success;
  }
public:
  ros::NodeHandle nh_;
  ros::ServiceClient client, grasp_client;

  bool move_success;
  bool robot_connected_;
  std::string pause_;
  bool result_;
/*定义抓取目标和位置*/
  std::string obj_grasp_label;
  geometry_msgs::Pose obj_grasp_pose;
/*抓取点参数列表*/
  std::vector<robot_grasp::Detected_obj_param> detected_param_obj_list;
/*定义航点*/
  geometry_msgs::Pose INTERPOLATION_WORLD_pose_1,INTERPOLATION_WORLD_pose_2,INTERPOLATION_WORLD_pose_3,INTERPOLATION_WORLD_pose_4,INTERPOLATION_WORLD_pose_5;
/*定义抓取航点*/
  geometry_msgs::Pose obj_grasp_pose_1;
/*定义放置点*/
  geometry_msgs::Pose place_pose;
/*定义轨迹插值点向量*/
  std::vector<geometry_msgs::Pose> waypoints;
/*定义运动约束及其轨迹*/
  moveit_msgs::Constraints endEffector_constraints;
  moveit_msgs::OrientationConstraint ocm;
  moveit_msgs::RobotTrajectory trajectory_msg;
  moveit_msgs::RobotTrajectory trajectory_msg_place;

  std::vector<geometry_msgs::Pose> grasp_list;
  moveit::planning_interface::MoveGroup kinova_move_group;
  moveit::planning_interface::MoveGroup kinova_gripper_group;
  const robot_state::JointModelGroup *kinova_joint_model_group;
  const robot_state::JointModelGroup *kinova_gripper_joint_model_group;
  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;
  // Define PlanningSceneInterface object to add and remove collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroup::Plan kinova_arm_plan;
  ros::Publisher world_joint_pub;
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  kinova_pick_place pick_place(nh,KINOVA_PLANNING_GROUP,KINOVA_GRIPPER_GROUP,DROPBOX_MESH_PATH);
  ros::ServiceServer service = nh.advertiseService("/get_pick_place_routine", &kinova_pick_place::kinova_routine, &pick_place);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  //while (ros::ok())
    //ros::spin();
  ros::waitForShutdown();
}
