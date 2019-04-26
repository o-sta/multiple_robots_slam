#include <exploration/continuity.hpp>
#include <kobuki_msgs/BumperEvent.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "continuity");
    Continuity<kobuki_msgs::BumperEvent> cbe;
    while(ros::ok()) cbe.publish();    
    return 0;
}