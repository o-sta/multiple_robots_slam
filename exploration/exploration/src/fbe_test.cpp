#include <exploration/frontier_search.hpp>
#include <exploration/movement.hpp>
#include <exploration/path_planning.hpp>
//#include <navfn/navfn_ros.h>
//#include <voronoi_planner/planner_core.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "frontier_based_exploration_test");

    FrontierSearch fs;
    Movement mv;
    //pathPlanning<navfn::NavfnROS> ppn;
    //pathPlanning<voronoi_planner::VoronoiPlanner> ppv;

    geometry_msgs::Point goal;

    mv.oneRotation();
    
    while(ros::ok()){
        if(fs.getGoal(goal)){
            mv.moveToGoal(goal,true);
        }
    }
    
    return 0;
}