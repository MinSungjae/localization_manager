#include "localization_manager/localization_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_manager");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener(tf_buffer);

    LocalizationManager localization_manager(&nh, &rate, &tf_buffer);

    while(ros::ok())
    {
        localization_manager.localize();
        ros::spinOnce();
        rate.sleep();
    }
}