#include "localization_manager/localization_manager.h"

void LocalizationManager::relative_loc_cb(nav_msgs::Odometry msg)
{
    relative_odom = msg;
}

void LocalizationManager::absolute_loc_cb(geometry_msgs::PoseStamped msg)
{
    absolute_loc = msg;
}

bool LocalizationManager::localize()
{
    geometry_msgs::PoseStamped base_link;
    base_link.header.frame_id = "world"; // or map
    base_link.header.stamp = ros::Time::now();

    tf2::Transform absolute_loc_tf;
    tf2::fromMsg(absolute_loc.pose, absolute_loc_tf);

    tf2::Transform relative_loc_tf;
    tf2::fromMsg(relative_odom.pose.pose, relative_loc_tf);
    
    tf2::Transform base_link_tf = absolute_loc_tf*base2cam_tf;
    tf2::toMsg(base_link_tf, base_link.pose);
    ROS_INFO_STREAM(absolute_loc);

    tf2::Transform world2odom_tf = absolute_loc_tf*base2cam_tf*relative_loc_tf;

    geometry_msgs::TransformStamped managed_base_link;
    managed_base_link.header = base_link.header;
    managed_base_link.child_frame_id = "base_link";
    managed_base_link.transform.translation.x = base_link.pose.position.x;
    managed_base_link.transform.translation.y = base_link.pose.position.y;
    managed_base_link.transform.translation.z = base_link.pose.position.z;
    managed_base_link.transform.rotation = base_link.pose.orientation;
    managed_base_link_brd.sendTransform(managed_base_link);

    geometry_msgs::TransformStamped world2odom;
    world2odom.header = base_link.header;
    world2odom.child_frame_id = "odom";
    world2odom.transform.translation.x = base_link.pose.position.x;
    world2odom.transform.translation.y = base_link.pose.position.y;
    world2odom.transform.translation.z = base_link.pose.position.z;
    world2odom.transform.rotation = base_link.pose.orientation;
    world2odom_brd.sendTransform(world2odom);
}