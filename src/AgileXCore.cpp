#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "project_1/CustomOdometry.h"
#include <sstream>

/*class MyOdometry{

private:
    int x,y,th;
    ros::Time lastTime;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

public:
    MyOdometry(ros::NodeHandle n){
        x=0;
        y=0;
        th=0;
        lastTime = ros::Time::now(); //todo: oppure leggo il primo header dal topic e inizializzo solo la prima volta
        odom_pub = n.advertise<nav_msgs::Odometry>("/odometry", 1);
    }

    void computeOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
        double vx, /*vy,*//* w, dt;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg->header.stamp;

        //Computes dt from last message
        dt = (currentTime - lastTime).toSec();

        //Computes reads linear and angular velocities from message
        vx = msg->twist.linear.x;
        //vy = msg->twist.linear.y;
        w = msg->twist.angular.z;

        //Integration
        x += vx * cos(th) * dt;
        y += vx * sin(th) * dt;
        th += w * dt;

        //Publish tf transformation
        publishTfTransformation(currentTime);
        //Publish odometry message
        publishOdometry(vx, /*vy,*//* w, currentTime);

        //Updates last time
        lastTime= currentTime;
    }

    void publishOdometry(double vx, /*double vy,*//* double w, ros::Time currentTime){
        nav_msgs::Odometry odometry;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

        //set header
        odometry.header.stamp = currentTime; //todo: time::now() o currentTime?
        odometry.header.frame_id = "odom";
        //set pose
        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = odometryQuaternion;
        //set velocity
        odometry.child_frame_id = "baseLink";
        odometry.twist.twist.linear.x = vx;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.angular.z = w;

        //publish odometry
        odom_pub.publish(odometry);
    }

    void publishTfTransformation(ros::Time currentTime){
        geometry_msgs::TransformStamped odometryTransformation;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

        //set header
        odometryTransformation.header.stamp = currentTime;
        odometryTransformation.header.frame_id = "odom";
        odometryTransformation.child_frame_id = "base_link";
        //set transformation
        odometryTransformation.transform.translation.x = x;
        odometryTransformation.transform.translation.y = y;
        odometryTransformation.transform.translation.z = 0;
        odometryTransformation.transform.rotation = odometryQuaternion;

        //publish transformation
        odom_broadcaster.sendTransform(odometryTransformation);
    }
};*/



class Pub_sub_odometry {

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    //MyOdometry odometry = MyOdometry(n);

    ros::Time lastTime;
    double x,y,th;
    ros::Publisher odom_pub;
    ros::Publisher custom_pub;
    tf::TransformBroadcaster odom_broadcaster;


public:
    Pub_sub_odometry() {
        //TODO: mettere topic delle velocità
        sub = n.subscribe("/twist", 1, &Pub_sub_odometry::computeOdometry2, this);

        odom_pub = n.advertise<nav_msgs::Odometry>("/odometry", 1);
        custom_pub = n.advertise<project_1::CustomOdometry>("/custom", 10);
        lastTime = ros::Time::now(); //todo: oppure leggo il primo header dal topic e inizializzo solo la prima volta
        //TODO: Devono essere sempre inizializzati a 0?
        x = 0;
        y = 0;
        th = 0;
    }

    /*void computeOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
        odometry.computeOdometry(msg);
    }*/

    void computeOdometry2(const geometry_msgs::TwistStamped::ConstPtr& msg){
        double vx, /*vy,*/ w, dt;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg->header.stamp;

        //Computes dt from last message
        dt = (currentTime - lastTime).toSec();
        //ROS_INFO("%lf", dt);
        //Computes reads linear and angular velocities from message
        vx = msg->twist.linear.x;
        //vy = msg->twist.linear.y;
        w = msg->twist.angular.z;

        //Integration
        x += vx * cos(th) * dt;
        y += vx * sin(th) * dt;
        th += w * dt;

        //ROS_INFO("vx:%lf, x:%lf, y:%lf, th:%lf",vx, x, y, th);

        //Publish tf transformation
        publishTfTransformation(currentTime);
        //Publish odometry message
        publishOdometry(vx, /*vy,*/ w, currentTime);

        //Updates last time
        lastTime= currentTime;
    }

    void publishOdometry(double vx, /*double vy,*/ double w, ros::Time currentTime){
        nav_msgs::Odometry odometry;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);
        project_1::CustomOdometry msg;

        //set header
        odometry.header.stamp = currentTime; //todo: time::now() o currentTime?
        odometry.header.frame_id = "odom";
        //set pose
        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = odometryQuaternion;
        //set velocity
        odometry.child_frame_id = "baseLInk";
        odometry.twist.twist.linear.x = vx;
        odometry.twist.twist.linear.y = 0;
        odometry.twist.twist.angular.z = w;

        //publish odometry
        msg.odom = odometry;
        //todo: pubblicare in custom metodo integrazione
        odom_pub.publish(odometry);
        custom_pub.publish(msg);
    }

    void publishTfTransformation(ros::Time currentTime){
        geometry_msgs::TransformStamped odometryTransformation;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

        //set header
        odometryTransformation.header.stamp = currentTime;
        odometryTransformation.header.frame_id = "odom";
        odometryTransformation.child_frame_id = "base_link";
        //set transformation
        odometryTransformation.transform.translation.x = x;
        odometryTransformation.transform.translation.y = y;
        odometryTransformation.transform.translation.z = 0;
        odometryTransformation.transform.rotation = odometryQuaternion;

        //publish transformation
        odom_broadcaster.sendTransform(odometryTransformation);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "AgileXCore");
    Pub_sub_odometry pubSubOdometry;
    ros::spin();
    return 0;
}