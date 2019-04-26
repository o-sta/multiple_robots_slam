#include <exploration/branch_search.hpp>
#include <exploration/movement.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sensor_based_exploration_movebase");

    BranchSearch bs;
    Movement mv;

    CommonLib::subStruct<std_msgs::Bool> isEnd("end",1);

    geometry_msgs::Point goal;
    ros::NodeHandle p("~");
    bool DEBUG;
    p.param<bool>("debug",DEBUG,false);

    usleep(2e5);//timeがsim_timeに合うのを待つ

    ros::Time start = ros::Time::now();
    // ROS_DEBUG_STREAM("start time : " << start);
    if(!DEBUG) mv.oneRotation();

    while(ros::ok()){
        bs.getGoal(goal) && !DEBUG ? mv.moveToGoal(goal) : mv.moveToForward();
        if(!isEnd.q.callOne(ros::WallDuration(0.5))&&isEnd.data.data) break;
    }
    
    
    ROS_INFO_STREAM("exploration finish !! time : " << ros::Duration(ros::Time::now()-start).toSec() << " [s]");
    // ROS_DEBUG_STREAM("end time : " << ros::Time::now());

    ros::shutdown();

    return 0;
}