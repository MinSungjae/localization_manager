#include "localization_manager/localization_manager.h"

void LocalizationManager::relative_loc_cb(nav_msgs::Odometry msg)
{
    last_relative_loc = ros::Time::now();

    relative_odom = msg;
}

void LocalizationManager::absolute_loc_cb(geometry_msgs::PoseStamped msg)
{
    absolute_loc = msg;
}

bool LocalizationManager::localize()
{
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped base_link;
    base_link.header.frame_id = world_frame_name; // or map
    base_link.header.stamp = now;

    odom_frame.header.frame_id = world_frame_name;
    odom_frame.header.stamp = now;

    if(now - last_relative_loc > ros::Duration(1.0))
        relative_odom_fail.data = true;
    else
        relative_odom_fail.data = false;
    relative_odom_fail_pub.publish(relative_odom_fail);

    // Type conversion to tf2::Transform for easy computation
    tf2::Transform relative_loc_tf;
    tf2::fromMsg(relative_odom.pose.pose, relative_loc_tf);

    tf2::Transform absolute_loc_tf;
    tf2::fromMsg(absolute_loc.pose, absolute_loc_tf);

    // Finding world(Global) to map(ORB) transformation
    if(now - absolute_loc.header.stamp < ros::Duration(0.2))
    {
        ROS_INFO("World to map correction...!");
        world2map_tf = absolute_loc_tf*relative_loc_tf.inverse();
    }

    // Localize base_link w.r.t. world frame
    tf2::Transform base_link_tf = world2map_tf*relative_loc_tf*base2cam_tf;
    tf2::toMsg(base_link_tf, base_link.pose);

    geometry_msgs::TransformStamped managed_base_link;
    managed_base_link.header = base_link.header;
    managed_base_link.child_frame_id = "base_link";
    managed_base_link.transform.translation.x = base_link.pose.position.x;
    managed_base_link.transform.translation.y = base_link.pose.position.y;
    managed_base_link.transform.translation.z = base_link.pose.position.z;
    managed_base_link.transform.rotation = base_link.pose.orientation;
    managed_base_link_brd.sendTransform(managed_base_link);

    // Broadcast tf map frame w.r.t. world frame
    tf2::toMsg(world2map_tf, odom_frame.pose);

    geometry_msgs::TransformStamped world2map;
    world2map.header = odom_frame.header;
    world2map.child_frame_id = relative_frame_name;
    world2map.transform.translation.x = odom_frame.pose.position.x;
    world2map.transform.translation.y = odom_frame.pose.position.y;
    world2map.transform.translation.z = odom_frame.pose.position.z;
    world2map.transform.rotation = odom_frame.pose.orientation;
    world2map_brd.sendTransform(world2map);

    // Publish resultant odometry w.r.t. World frame
    managed_odom.header.stamp = now;
    managed_odom.header.frame_id = world_frame_name;
    managed_odom.pose.pose.position.x = managed_base_link.transform.translation.x;
    managed_odom.pose.pose.position.y = managed_base_link.transform.translation.y;
    managed_odom.pose.pose.position.z = managed_base_link.transform.translation.z;
    managed_odom.pose.pose.orientation = managed_base_link.transform.rotation;
    managed_loc_pub.publish(managed_odom);

    // tf2::Transform absolute_loc_tf;
    // tf2::fromMsg(absolute_loc.pose, absolute_loc_tf);

    // tf2::Transform relative_loc_tf;
    // tf2::fromMsg(relative_odom.pose.pose, relative_loc_tf);
    
    // tf2::Transform base_link_tf = absolute_loc_tf*base2cam_tf;
    // tf2::toMsg(base_link_tf, base_link.pose);

    // tf2::Transform world2odom_tf = absolute_loc_tf*base2cam_tf*relative_loc_tf.inverse();
    // tf2::toMsg(world2odom_tf, odom_frame.pose);

    // geometry_msgs::TransformStamped managed_base_link;
    // managed_base_link.header = base_link.header;
    // managed_base_link.child_frame_id = "base_link";
    // managed_base_link.transform.translation.x = base_link.pose.position.x;
    // managed_base_link.transform.translation.y = base_link.pose.position.y;
    // managed_base_link.transform.translation.z = base_link.pose.position.z;
    // managed_base_link.transform.rotation = base_link.pose.orientation;
    // managed_base_link_brd.sendTransform(managed_base_link);

    // geometry_msgs::TransformStamped world2odom;
    // world2odom.header = odom_frame.header;
    // world2odom.child_frame_id = "map";
    // world2odom.transform.translation.x = odom_frame.pose.position.x;
    // world2odom.transform.translation.y = odom_frame.pose.position.y;
    // world2odom.transform.translation.z = odom_frame.pose.position.z;
    // world2odom.transform.rotation = odom_frame.pose.orientation;
    // world2odom_brd.sendTransform(world2odom);

    // managed_odom.header.stamp = now;
    // managed_odom.header.frame_id = "world";
    // managed_odom.pose.pose.position.x = managed_base_link.transform.translation.x;
    // managed_odom.pose.pose.position.y = managed_base_link.transform.translation.y;
    // managed_odom.pose.pose.position.z = managed_base_link.transform.translation.z;
    // managed_odom.pose.pose.orientation = managed_base_link.transform.rotation;

    // managed_loc_pub.publish(managed_odom);
}