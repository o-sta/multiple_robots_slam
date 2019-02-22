#include <costmap_to_voronoi/costmap_to_voronoi.h>
geometry_msgs::PoseStamped start_pose_set(std::string& frame_id)
{
  std::cout << "start_pose_set" << std::endl;
  geometry_msgs::PoseStamped start_;
  start_.header.frame_id = frame_id;
  start_.pose.position.x = 0.0;
  start_.pose.position.y = 0.0;
  start_.pose.position.x = 0.0;
  start_.pose.orientation.x=0.0;
  start_.pose.orientation.y=0.0;
  start_.pose.orientation.z=0.0;
  start_.pose.orientation.w=1.0;
  return start_;
}
void robot_final_target_CB(const geometry_msgs::PoseStamped &goal)
{
  std::cout << "robot_final_target_CB" << std::endl;
  std::string name;
  std::string robot_frame_id_;
  ros::NodeHandle p("~");
  p.getParam("topic_ns", name);
  p.getParam(name+"global_frame",robot_frame_id_);
  CostmapToVoronoi ctv(name);
  geometry_msgs::PoseStamped start_,goal_;
  std::vector<geometry_msgs::PoseStamped> plan;
  start_ = start_pose_set(robot_frame_id_);
  goal_ = goal;
  ctv.voronoiProcess(start_,goal_,plan);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_to_voronoi");

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  std::vector<geometry_msgs::PoseStamped> plan;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  sub = nh.subscribe("final_target",1,&robot_final_target_CB);

  while(ros::ok()){
    std::cout << "queue callone" << std::endl;
    queue.callOne(ros::WallDuration(1.0));
  }

  return 0;
}
