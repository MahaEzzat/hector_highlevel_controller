#include <ros/ros.h>
#include "hector_highlevel_controller/HectorHighlevelController.h"

int  main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_highlevel_controller");
    ros::NodeHandle n("~");

 hector_highlevel_controller::HectorHighlevelController hectorighlevelcontroller(n);

    ros::spin();
    return 0;
}