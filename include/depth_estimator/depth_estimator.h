#ifndef DEPTH_ESTIMATOR
#define DEPTH_ESTIMATOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <fstream>
#include <sstream>

#include <camera_apps_msgs/BoundingBox.h>
#include <camera_apps_msgs/BoundingBoxes.h>

namespace depth_estimator
{
    class DepthEstimator
    {
        public:
            DepthEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        private:
            void image_callback(const sensor_msgs::ImageConstPtr &msg);
            void set_network();
            void object_detect(cv::Mat &image);
            std::vector<std::string> get_outputs_names(cv::dnn::Net& net);

            std::string camera_topic_name_;
            std::string model_path_;

            cv::Mat input_image_;
            ros::Time input_image_stamp_;
            std_msgs::Header input_image_header_;
            // ros::Time msg_stamp_;
            cv::Mat detection_image_;
            cv::dnn::Net net_;

            image_transport::Subscriber image_sub_;
            image_transport::Publisher image_pub_;
    };
}
#endif
