#include "first_challenge_ogiwara/first_challenge_ogiwara.h"

FirstChallenge::FirstChallenge():private_nh_("~"),nh_("")
{
    private_nh_.param("hz", hz_, {10});

    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/cmd_vel",1);
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_scan_ = nh_.subscribe("/roomba/scan", 100, &FirstChallenge::laser_callback, this);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_ = *msg;
}

void FirstChallenge::run(float v, float r)
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = v;
    cmd_vel_.cntl.angular.z = r;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::debug()
{
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << ": y:" << odometry_.pose.pose.position.y << ": z:" << odometry_.pose.pose.position.z << ": theta:" << tf::getYaw(odometry_.pose.pose.orientation) << std::endl;
}

float FirstChallenge::scan()
{
    float sight = 15.0 / 0.125;
    int laser_front = laser_.ranges.size() / 2;
    float range_min = 1e6;
    for(int i = laser_front - sight ; i < laser_front + sight; i++){
        if(laser_.ranges[i] < range_min && laser_.ranges[i] > 0.20){
            range_min = laser_.ranges[i];
        }
    }
    return range_min;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);

    initflag_ = false;
    rotflag_ = false;
    rotating_ = false;
    rotated_ = false;
    complete_ = false;

    while(ros::ok())
    {
        ros::spinOnce();

        if(!complete_)
        {
            if(!rotating_){
                run(0.20, 0.0);
                rotating_ = odometry_.pose.pose.position.x >= 1.0;
            }
            else if(!rotated_){
                if(!initflag_)
                {
                    rot_target_ = tf::getYaw(odometry_.pose.pose.orientation);
                    old_odom_ = odometry_;
                    initflag_ = true;
                }
                if(!rotflag_)
                {
                    rotflag_ = tf::getYaw(old_odom_.pose.pose.orientation) * tf::getYaw(odometry_.pose.pose.orientation) < -M_PI;
                    old_odom_ = odometry_;
                }

                run(0.0,0.30);
                rotated_ = rotflag_ && fabs( rot_target_ - tf::getYaw(odometry_.pose.pose.orientation) ) < 0.05;
            }
            else{
                run(0.10, 0.0);
                complete_ = scan() <= 0.50;
            }

        }

        else
        {
            run(0.0, 0.0);
        }

        debug();
        loop_rate.sleep();
    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"first_challenge_ogiwara");
    FirstChallenge firstchallenge;
    firstchallenge.process();
    ros::spin();
    return 0;
}
