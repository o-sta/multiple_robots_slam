#include "../include/costmap_to_voronoi.h"
#include<nav_msgs/Odometry.h>

ros::Subscriber odom_sub;
ros::CallbackQueue odom_queue;
geometry_msgs::PoseStamped start_g;

void start_pose_set(const nav_msgs::Odometry::ConstPtr& robot_odom_);
void robot_final_target_CB(const geometry_msgs::PoseStamped &goal);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_to_voronoi");
  ros::NodeHandle odom_nh;

  ros::NodeHandle p("~");
  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  std::vector<geometry_msgs::PoseStamped> plan;
  std::string robot_namespace_;
  p.getParam("namespace",robot_namespace_);
  std::cout << "namespace:" << robot_namespace_ << std::endl;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  odom_nh.setCallbackQueue(&odom_queue);
  sub = nh.subscribe("final_target",1,&robot_final_target_CB);
  odom_sub = odom_nh.subscribe(robot_namespace_ + "/odom",1 ,start_pose_set);
  while(ros::ok()){
    std::cout << "queue callone" << std::endl;
    queue.callOne(ros::WallDuration(1.0));
  }

  return 0;
}


void start_pose_set(const nav_msgs::Odometry::ConstPtr& robot_odom_)
{
  std::cout << "start_pose_set" << std::endl;
  start_g.header.frame_id = robot_odom_->header.frame_id;
  start_g.pose.position.x = robot_odom_->pose.pose.position.x;
  start_g.pose.position.y = robot_odom_->pose.pose.position.y;
  start_g.pose.position.z = robot_odom_->pose.pose.position.z;
  start_g.pose.orientation.x=robot_odom_->pose.pose.orientation.x;
  start_g.pose.orientation.y=robot_odom_->pose.pose.orientation.y;
  start_g.pose.orientation.z=robot_odom_->pose.pose.orientation.z;
  start_g.pose.orientation.w=robot_odom_->pose.pose.orientation.w;
  std::cout << "end start_pose_set" << std::endl;
}
void robot_final_target_CB(const geometry_msgs::PoseStamped &goal)
{
  std::cout << "robot_final_target_CB" << std::endl;
  std::string name;
  std::string robot_frame_id_;
  ros::NodeHandle p("~");
  p.getParam("topic_ns", name);
  std::cout << "topic ns : " << name << std::endl;
  p.getParam(name+"/global_frame",robot_frame_id_);
  std::cout << "global frame : " << robot_frame_id_ << std::endl;
  CostmapToVoronoi ctv(name);
  geometry_msgs::PoseStamped goal_;
  std::vector<geometry_msgs::PoseStamped> plan;
  odom_queue.callOne(ros::WallDuration(0.1));
  goal_ = goal;
  ctv.voronoiProcess(start_g,goal_,plan);
}

