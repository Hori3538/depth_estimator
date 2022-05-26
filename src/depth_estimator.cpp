#include <depth_estimator/depth_estimator.h>

namespace depth_estimator
{
    DepthEstimator::DepthEstimator(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("camera_topic_name", camera_topic_name_, std::string("/camera/color/image_raw"));
        pnh.getParam("model_path", model_path_);

        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe(camera_topic_name_, 1, &DepthEstimator::image_callback, this);
        image_pub_ = it.advertise("/depth_image", 1);
        // bbox_pub_ = nh.advertise<camera_apps_msgs::BoundingBox>("/bounding_box", 1);

        set_network();
    }

    void DepthEstimator::image_callback(const sensor_msgs::ImageConstPtr &msg)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try{
            // input_image_stamp_ = msg->header.stamp;
            input_image_header_ = msg->header;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            input_image_ = cv_ptr->image;
            object_detect(input_image_);
        }
        catch(cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }


    void DepthEstimator::set_network()
    {
        // std::string proto_path = model_path_ + "/ssd_mobilenet_v2_coco.pbtxt";
        // std::string weight_path = model_path_ + "/frozen_inference_graph.pb";
        // std::string label_path = model_path_ + "/object_detection_classes_coco.txt";
        std::string model = model_path_ + "/model-f6b98070.onnx";

        net_ = cv::dnn::readNet(model);

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        // class_names_ = read_file(label_path);
    }

    void DepthEstimator::object_detect(cv::Mat &image)
    {

        // cv::Mat blob = cv::dnn::blobFromImage(image, 1, cv::Size(300, 300));
        cv::Mat blob = cv::dnn::blobFromImage( image, 1 / 255.f, cv::Size( 384, 384 ), cv::Scalar( 123.675, 116.28, 103.53 ), true, false );
        net_.setInput(blob);
        // cv::Mat pred = net_.forward();
        // cv::Mat pred_mat(pred.size[2], pred.size[3], CV_32F, pred.ptr<float>());
        cv::Mat output = net_.forward(get_outputs_names(net_)[0]);
        std::vector<int32_t> size = { output.size[1], output.size[2] };
        output = cv::Mat( static_cast<int32_t>( size.size() ), &size[0], CV_32F, output.ptr<float>() );
        cv::resize( output, output, image.size() );

        double min, max;
        cv::minMaxLoc( output, &min, &max );
        const double range = max - min;
        output.convertTo( output, CV_32F, 1.0 / range, - ( min / range ) );
        output.convertTo( output, CV_8U, 255.0 );

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", output).toImageMsg();
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", output).toImageMsg();
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", output).toImageMsg();
        // msg->header.stamp = input_image_stamp_;
        msg->header = input_image_header_;
        // msg->header.stamp = input_image_->header.stamp;
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub_.publish(msg);
    }


    std::vector<std::string> DepthEstimator::get_outputs_names(cv::dnn::Net& net)
    {
        std::vector<std::string> names;
        if( names.empty() ){
            std::vector<int32_t> out_layers = net.getUnconnectedOutLayers();
            std::vector<std::string> layers_names = net.getLayerNames();
            names.resize( out_layers.size() );
            for( size_t i = 0; i < out_layers.size(); ++i ){
                names[i] = layers_names[out_layers[i] - 1];
            }
        }
        return names;
    }
}

