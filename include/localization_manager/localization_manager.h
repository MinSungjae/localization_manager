#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalizationManager {
private:
    ros::NodeHandle* _nh;
    ros::Rate* _rate;

    ros::Subscriber relative_loc_sub;
    ros::Subscriber absolute_loc_sub;

    ros::Publisher managed_loc_pub;
    ros::Publisher relative_odom_fail_pub;

    tf2_ros::TransformBroadcaster managed_base_link_brd, map2odom_brd;

    void relative_loc_cb(nav_msgs::Odometry msg);
    void absolute_loc_cb(geometry_msgs::PoseStamped msg);

    tf2_ros::Buffer* _tf_buffer;

    std::string world_frame_name;
    std::string relative_frame_name;

protected:
    ros::Time last_relative_loc;
    nav_msgs::Odometry relative_odom, managed_odom, latest_relative_odom;
    geometry_msgs::PoseStamped absolute_loc;
    geometry_msgs::TransformStamped base2cam;

    tf2::Transform base2cam_tf;

    geometry_msgs::PoseStamped odom_frame;

    tf2::Transform map2odom_tf;

    std_msgs::Bool relative_odom_fail;

    bool relative_loc_losted = false;
    bool absolute_loc_found = true;

public:
    LocalizationManager(ros::NodeHandle* nh, ros::Rate* rate, tf2_ros::Buffer* buffer): _nh(nh), _rate(rate), _tf_buffer(buffer)
    {
        relative_loc_sub = _nh->subscribe("orb_odom", 1, &LocalizationManager::relative_loc_cb, this);
        absolute_loc_sub = _nh->subscribe("camera_link", 1, &LocalizationManager::absolute_loc_cb, this);
        managed_loc_pub = _nh->advertise<nav_msgs::Odometry>("odom", 1);
        relative_odom_fail_pub = _nh->advertise<std_msgs::Bool>("odom_fail", 1);

        if(_nh->getParam(ros::this_node::getName() + "/world_frame_name", world_frame_name))
            ROS_INFO_STREAM("Got world name: " << world_frame_name);
        if(_nh->getParam(ros::this_node::getName() + "/relative_frame_name", relative_frame_name))
            ROS_INFO_STREAM("Got relative frame name: " << relative_frame_name);

        while (ros::ok())
        {
            try{
                base2cam = _tf_buffer->lookupTransform("camera_link", "base_link", ros::Time(0));
                tf2::fromMsg(base2cam.transform, base2cam_tf);
                ROS_INFO("Got robot to camera transform...");
                break;
            }
            catch(tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        // Initial guess for transformation between world and map frames
        map2odom_tf.setIdentity();
        relative_odom_fail.data = false;
    }

    bool localize();

};