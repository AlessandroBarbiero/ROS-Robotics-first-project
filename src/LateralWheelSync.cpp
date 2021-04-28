#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class LateralWheelSync
{

    robotics_hw1::MotorSpeed message;
    robotics_hw1::MotorSpeed message2;

private:
    ros::NodeHandle n;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2;
    ros::Publisher pub;
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
    //message_filters::Synchronizer<MySyncPolicy> sync;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

public:
    LateralWheelSync(){
        sub1.subscribe(n, "/motor_speed_fl", 1);
        sub2.subscribe(n, "/motor_speed_rl", 1);
        pub = n.advertise<robotics_hw1::MotorSpeed>("/syncVelocity", 1);
        sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));
        //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
        sync->registerCallback(boost::bind(&LateralWheelSync::syncCallback, this, _1, _2));
    }

    void syncCallback(const robotics_hw1::MotorSpeed::ConstPtr& msg1,
                  const robotics_hw1::MotorSpeed::ConstPtr& msg2) {
        ROS_INFO ("syncCallback");
        pub.publish(*msg1);
        /*ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)",
                  msg1->vector.x,msg1->vector.y,msg1->vector.z,
                  msg2->vector.x, msg2->vector.y, msg2->vector.z);*/
    }





};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Synchronizer");
    LateralWheelSync sync;
    ros::spin();
    return 0;
}


