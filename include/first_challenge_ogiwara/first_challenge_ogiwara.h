#ifndef FIRST_CHALLENGE_OGIWARA_H
#define FIRST_CHALLENGE_OGIWARA_H

#include <ros/ros.h>
#include <iostream>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

class FirstChallenge
{
    public:
        FirstChallenge();
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

        void run(float velocity, float omega);
        void debug();
        float scan();

        int hz_;
        float rot_target_;

        bool initflag_;
        bool rotflag_;
        bool rotating_;
        bool rotated_;
        bool complete_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher pub_cmd_vel_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_scan_;

        nav_msgs::Odometry odometry_;
        nav_msgs::Odometry old_odom_;
        sensor_msgs::LaserScan laser_;
        roomba_500driver_meiji::RoombaCtrl cmd_vel_;


};

#endif

