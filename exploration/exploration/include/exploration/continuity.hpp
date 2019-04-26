//topicのpublishレートを上げる(中間を埋めるだけ)
#ifndef CONTINUITY_HPP
#define CONTINUITY_HPP

#include <ros/ros.h>
#include <exploration/common_lib.hpp>

template<typename T>
class Continuity
{
private:
    CommonLib::subStruct<T> sub_;
    CommonLib::pubStruct<T> pub_;

public:
    Continuity():sub_("sub_topic",1),pub_("pub_topic",1){};
    void publish(void){
        sub_.q.callOne(ros::WallDuration(0.1));
        pub_.pub.publish(sub_.data);
    };
};

#endif //CONTINUITY_HPP