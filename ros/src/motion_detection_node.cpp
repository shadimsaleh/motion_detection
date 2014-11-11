#include <motion_detection/motion_detection_node.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

MotionDetectionNode::MotionDetectionNode(ros::NodeHandle &nh): nh_(nh), it_(nh), rng(12345), output_cap_("/home/santosh/test.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30, cv::Size(320, 240), true)
{
    cloud_received_ = false;
    image_received_ = false;
    odom_received_ = false;
    first_image_received_ = false;
    camera_params_set_ = false;
    first_run_ = true;
    nh_.getParam("pixel_step", pixel_step_);
    frame_number_ = 0;

    nh_.param<bool>("use_odom", use_odom_, false);
    nh_.param<bool>("use_pointcloud", use_pointcloud_, false);
    nh_.param<bool>("record_video", record_video_, false);
    nh_.param<bool>("use_all_frames", use_all_frames_, false);
    nh_.param<bool>("write_vectors", write_vectors_, false);
    nh_.param<bool>("include_zeros", include_zeros_, false);

    of_image_publisher_ = it_.advertise("optical_flow_image", 1);
    image1_publisher_ = it_.advertise("image1", 1);
    image2_publisher_ = it_.advertise("image2", 1);
    expected_flow_publisher_ = it_.advertise("expected_flow_image", 1);
    compensated_flow_publisher_ = it_.advertise("compensated_flow_image", 1);
    clustered_flow_publisher_ = it_.advertise("clustered_flow_image", 1);
    background_subtraction_publisher_ = it_.advertise("background_subtraction_image", 1);

    if (use_pointcloud_)
    {
        cloud_subscriber_ = nh_.subscribe("input_pointcloud", 1, &MotionDetectionNode::cloudCallback, this);
    }
    image_subscriber_ = it_.subscribe("input_image", 0, &MotionDetectionNode::imageCallback, this);
    if (use_odom_)
    {
        odom_subscriber_ = nh_.subscribe("/odom", 1, &MotionDetectionNode::odomCallback, this);
    }

    camera_info_subscriber_ = nh_.subscribe("input_camerainfo", 1, &MotionDetectionNode::cameraInfoCallback, this);
}

MotionDetectionNode::~MotionDetectionNode()
{
}

void MotionDetectionNode::runOpticalFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors)
{
    cv::Mat debug_image;
    cv::Mat optical_flow_image;

    optical_flow_vectors = cv::Mat::zeros(image1.rows, image1.cols, CV_32FC4);
    int num_vectors = ofc_.calculateOpticalFlow(image1, image2, optical_flow_vectors, pixel_step_, debug_image);
    ofv_.showOpticalFlowVectors(image1, optical_flow_image, optical_flow_vectors, pixel_step_, CV_RGB(0, 0, 255));

    publishImage(optical_flow_image, of_image_publisher_);
    if (record_video_)
    {
        cv::Mat mat2;
        cv::cvtColor(optical_flow_image, mat2, CV_RGB2BGR);
        output_cap_.write(mat2);
    }
}

void MotionDetectionNode::detectOutliers(const cv::Mat &original_image, const cv::Mat &optical_flow_vectors, cv::Mat &outlier_mask, bool include_zeros)
{
    od_.findOutliers(optical_flow_vectors, outlier_mask, include_zeros, pixel_step_, false);
    cv::Mat outlier_image;
    ofv_.showFlowOutliers(original_image, outlier_image, optical_flow_vectors, outlier_mask, pixel_step_, false); 

    publishImage(outlier_image, compensated_flow_publisher_);
}

void MotionDetectionNode::clusterFlow(const cv::Mat &image, const cv::Mat &flow_vectors, std::vector<std::vector<cv::Vec4d> > &clusters)
{
    double distance_threshold, angular_threshold;
    nh_.getParam("distance_threshold", distance_threshold);
    nh_.getParam("angular_threshold", angular_threshold);

    cv::Mat clustered_flow_image;
    clusters = fc_.getClusters(flow_vectors, pixel_step_, distance_threshold, angular_threshold);
    /*
    std::cout << "clusters_second: " << std::endl;
    for (int i = 0; i < clusters.size(); i++)
    {
        std::cout << clusters.at(i) << std::endl; 
    }
    */

    for (int i = 0; i < clusters.size(); i++)
    {
        cv::Scalar colour;
        cv::Mat temp;
        switch (i)
        {
            case 0: colour = CV_RGB(0, 0, 255); break;
            case 1: colour = CV_RGB(0, 255, 0); break;
            case 2: colour = CV_RGB(255, 0, 0); break;
            case 3: colour = CV_RGB(255, 0, 255); break;
            case 4: colour = CV_RGB(255, 255, 0); break;
            case 5: colour = CV_RGB(0, 255, 255); break;
            default: colour = CV_RGB(0, 0, 255);break;
        }

        if (i == 0)
        {
            ofv_.showFlowClusters(image, clustered_flow_image, clusters.at(i), pixel_step_, colour);
        }
        else
        {
            ofv_.showFlowClusters(clustered_flow_image, temp, clusters.at(i), pixel_step_, colour);
            temp.copyTo(clustered_flow_image);
        }
    }

    publishImage(clustered_flow_image, clustered_flow_publisher_);
}

void MotionDetectionNode::writeVectors(const cv::Mat &flow_vectors, const std::string &filename)
{
    ofc_.writeFlow(flow_vectors, filename, pixel_step_); 
}

void MotionDetectionNode::publishImage(const cv::Mat &image, const image_transport::Publisher &publisher)
{
    cv_bridge::CvImage image_msg;
    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.image = image;
    publisher.publish(image_msg.toImageMsg());
}

void MotionDetectionNode::odomCallback(const nav_msgs::Odometry &odom)
{
    odom_ = odom;
    odom_received_ = true;
}

void MotionDetectionNode::cloudCallback(const sensor_msgs::PointCloud2 &cloud)
{
}

void MotionDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    if (first_image_received_)
    {
        raw_image1_ = raw_image2_;
        raw_image2_ = image;

        image_received_ = true;
    }
    else
    {
        raw_image2_ = image;
        first_image_received_ = true;
    }
    if (use_all_frames_ && image_received_ == true)
    {
        image_received_ = true;
        cv_bridge::CvImagePtr cv_image1;
        cv_bridge::CvImagePtr cv_image2;
        cv_image1 = cv_bridge::toCvCopy(raw_image1_, "rgb8");
        cv_image2 = cv_bridge::toCvCopy(raw_image2_, "rgb8");
        cv::Mat optical_flow_vectors;
        cv::Mat outlier_mask;
        std::vector<std::vector<cv::Vec4d> > clusters;
        runOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_vectors);
        detectOutliers(cv_image1->image, optical_flow_vectors, outlier_mask, include_zeros_); 
        clusterFlow(cv_image1->image, optical_flow_vectors, clusters);
        frame_number_++;
        if (write_vectors_)
        {
            std::stringstream ss;
            ss << frame_number_;
            std::string filename = "/home/santosh/data/frame" + ss.str();
            writeVectors(optical_flow_vectors, filename);
        }
    }
}


void MotionDetectionNode::cameraInfoCallback(const sensor_msgs::CameraInfo &camera_info)    
{
    
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "motion_detection");

    ros::NodeHandle n("~");

    ROS_INFO("[motion_detection] node started");

    MotionDetectionNode mdn(n); 
    ros::spin();
    /*
    while(ros::ok())
    {
    }
    */

    return 0;
}
