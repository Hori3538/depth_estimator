#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <depth_estimator/depth_estimator.h>


namespace depth_estimator{
    class DepthEstimatorNodelet : public nodelet::Nodelet
    {
        public:
            DepthEstimatorNodelet() = default;
            ~DepthEstimatorNodelet() {
        if (depth_estimator_) delete depth_estimator_;
            }
        private:
            virtual void onInit() {
                ros::NodeHandle nh;
                ros::NodeHandle pnh("~");
                pnh = getPrivateNodeHandle();
                depth_estimator_ = new depth_estimator::DepthEstimator(nh, pnh);
            }
            depth_estimator::DepthEstimator *depth_estimator_;
    };
}
// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(depth_estimator::DepthEstimatorNodelet, nodelet::Nodelet);
