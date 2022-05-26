#include <ros/ros.h>
#include <depth_estimator/depth_estimator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_estimator_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    depth_estimator::DepthEstimator depth_estimator(nh, pnh);


    ros::spin();
    return 0;
}
