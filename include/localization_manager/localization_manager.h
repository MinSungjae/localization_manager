#include <ros/ros.h>

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

    void relative_loc_cb(nav_msgs::Odometry msg);
    void absolute_loc_cb(nav_msgs::Odometry msg);

    tf2_ros::Buffer* _tf_buffer;

protected:
    nav_msgs::Odometry relative_odom, absolute_odom, managed_odom;
    geometry_msgs::TransformStamped base2cam;

public:
    LocalizationManager(ros::NodeHandle* nh, ros::Rate* rate, tf2_ros::Buffer* buffer): _nh(nh), _rate(rate), _tf_buffer(buffer)
    {
        relative_loc_sub = _nh->subscribe("orb_odom", 1, &LocalizationManager::relative_loc_cb, this);
        absolute_loc_sub = _nh->subscribe("tag_localization", 1, &LocalizationManager::absolute_loc_cb, this);
        managed_loc_pub = _nh->advertise<nav_msgs::Odometry>("odom", 1);

        while (_nh->ok())
        {
            try{
                base2cam = _tf_buffer->lookupTransform("camera_base_link", "base_link", ros::Time(0));
                break;
            }
            catch(tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
    }

    bool localize();

};