#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

class LocalizationManager {
private:
    ros::NodeHandle* _nh;
    ros::Rate* _rate;

    ros::Subscriber relative_loc_sub;
    ros::Subscriber absolute_loc_sub;

    ros::Publisher managed_loc_pub;

    void relative_loc_cb(nav_msgs::Odometry msg);
    void absolute_loc_cb(nav_msgs::Odometry msg);

protected:
    nav_msgs::Odometry relative_odom, absolute_odom, managed_odom;

public:
    LocalizationManager(ros::NodeHandle* nh, ros::Rate* rate): _nh(nh), _rate(rate)
    {
        relative_loc_sub = _nh->subscribe("orb_odom", 1, &LocalizationManager::relative_loc_cb, this);
        absolute_loc_sub = _nh->subscribe("tag_localization", 1, &LocalizationManager::absolute_loc_cb, this);
        managed_loc_pub = _nh->advertise<nav_msgs::Odometry>("odom", 1);
    }

    bool localize();


};