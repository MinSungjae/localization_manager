#include "localization_manager/localization_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_manager");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    LocalizationManager localization_manager;

    while(ros::ok())
    {
        localization_manager.localize();
        ros::spinOnce();
        rate.sleep();
    }
}