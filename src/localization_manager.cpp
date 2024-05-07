#include "localization_manager/localization_manager.h"

void LocalizationManager::relative_loc_cb(nav_msgs::Odometry msg)
{
    relative_odom = msg;

    
}

void LocalizationManager::absolute_loc_cb(nav_msgs::Odometry msg)
{
    absolute_odom = msg;
}

bool LocalizationManager::localize()
{
    ROS_INFO_STREAM(base2cam.transform.translation);
    // absolute_odom.pose.position

    // managed_odom.pose.pose.position = ;
    // managed_odom.pose.pose.orientation = ;
}