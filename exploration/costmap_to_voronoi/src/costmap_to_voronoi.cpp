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
void robot1_final_target_CB(const geometry_msgs::PoseStamped &goal)
{
  std::cout << "robot1_final_target_CB" << std::endl;
  std::string name;
  std::string robot1_frame_id_ = "/robot1/map";
  ros::NodeHandle p("~");
  p.getParam("topic_ns", name);
  CostmapToVoronoi r1_CB_ctv(name);
  geometry_msgs::PoseStamped start_,goal_;
  std::vector<geometry_msgs::PoseStamped> plan;
  start_ = start_pose_set(robot1_frame_id_);
  goal_ = goal;
  std::cout << "voronoiProcessが怪しい..." << std::endl;
  r1_CB_ctv.voronoiProcess(start_,goal_,plan);
}

void robot2_final_target_CB(const geometry_msgs::PoseStamped &goal)
{
  std::cout << "robot1_final_target_CB" << std::endl;
  std::string name;
  std::string robot2_frame_id_ = "/robot2/map";
  ros::NodeHandle p("~"); 
  p.getParam("topic_ns", name);
  CostmapToVoronoi r2_CB_ctv(name);
  geometry_msgs::PoseStamped start_,goal_;
  std::vector<geometry_msgs::PoseStamped> plan;
  start_ = start_pose_set(robot2_frame_id_);
  goal_ = goal;
  std::cout << "voronoiProcessが怪しい..." << std::endl;
  r2_CB_ctv.voronoiProcess(start_,goal_,plan);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "costmap_to_voronoi");

  std::string name;
  ros::NodeHandle p("~");
  p.getParam("topic_ns", name);

  CostmapToVoronoi ctv(name);

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  std::vector<geometry_msgs::PoseStamped> plan;

  ros::NodeHandle nh1,nh2;
  ros::Subscriber sub1,sub2;
  ros::CallbackQueue queue1,queue2;
  nh1.setCallbackQueue(&queue1);
  nh2.setCallbackQueue(&queue2);
  sub1 = nh1.subscribe("/robot1/final_target",1,&robot1_final_target_CB);
  sub2 = nh2.subscribe("/robot2/final_target",1,&robot2_final_target_CB);

  while(ros::ok()){
    std::cout << "queue1 callone" << std::endl;
    queue1.callOne(ros::WallDuration(1.0));
    std::cout << "queue2 callone" << std::endl;
    queue2.callOne(ros::WallDuration(1.0));
  }

  return 0;
}
