#ifndef MOTION_DETECTION_NODE_H_
#define MOTION_DETECTION_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <motion_detection/expected_flow_calculator.h>
#include <motion_detection/optical_flow_calculator.h>
#include <motion_detection/flow_clusterer.h>
#include <motion_detection/flow_difference_calculator.h>
#include <motion_detection/optical_flow_visualizer.h>
#include <motion_detection/background_subtractor.h>
#include <motion_detection/flow_neighbour_similarity_calculator.h>
#include <motion_detection/outlier_detector.h>
#include <motion_detection/trajectory_visualizer.h>

class MotionDetectionNode
{
    public:
        MotionDetectionNode(ros::NodeHandle &nh);
        virtual ~MotionDetectionNode();

        void run();
        void publishImage(const cv::Mat &image, const image_transport::Publisher &publisher);
        void odomCallback(const nav_msgs::Odometry &odom);
        void cloudCallback(const sensor_msgs::PointCloud2 &cloud);
        void imageCallback(const sensor_msgs::ImageConstPtr &image);
        void cameraInfoCallback(const sensor_msgs::CameraInfo &camera_info);
        
    private:
        void writeVectors(const cv::Mat &flow_vectors, const std::string &filename);
        void writeTrajectories(const std::vector<std::vector<cv::Point2f> > &trajectories, const std::string &filename);
        void runOpticalFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors);
        void runOpticalFlowTrajectory(const std::vector<cv::Mat> &images, cv::Mat &optical_flow_vectors, std::vector<std::vector<cv::Point2f> > &trajectories);
        void clusterFlow(const cv::Mat &image, const cv::Mat &flow_vectors, std::vector<std::vector<cv::Vec4d> > &clusters);

        void detectOutliers(const cv::Mat &original_image, const cv::Mat &optical_flow_vectors, cv::Mat &outlier_mask, bool include_zeros);
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        ros::Subscriber cloud_subscriber_;
        ros::Subscriber camera_info_subscriber_;
        ros::Subscriber odom_subscriber_;
        image_transport::Subscriber image_subscriber_;
        image_transport::Publisher of_image_publisher_;
        image_transport::Publisher image1_publisher_;
        image_transport::Publisher image2_publisher_;
        image_transport::Publisher expected_flow_publisher_;
        image_transport::Publisher compensated_flow_publisher_;
        image_transport::Publisher clustered_flow_publisher_;
        image_transport::Publisher background_subtraction_publisher_;
        bool cloud_received_;
        bool image_received_;
        bool odom_received_;        
        bool first_image_received_;
        bool camera_params_set_;
        bool first_run_;
        bool use_odom_;
        bool use_pointcloud_;
        bool record_video_;
        bool use_all_frames_;
        bool write_vectors_;
        bool write_trajectories_;
        bool include_zeros_;
        int pixel_step_;
        double min_vector_size_;
        int trajectory_size_;
        int global_frame_count_;

        sensor_msgs::PointCloud2 cloud_;
        std::list<sensor_msgs::ImageConstPtr> raw_images_;
        sensor_msgs::ImageConstPtr raw_image1_;
        sensor_msgs::ImageConstPtr raw_image2_;
        nav_msgs::Odometry odom_;
        nav_msgs::Odometry prev_odom_;
        OpticalFlowCalculator ofc_;
        ExpectedFlowCalculator efc_;
        FlowClusterer fc_;
        FlowDifferenceCalculator fdc_;
        OpticalFlowVisualizer ofv_;
        TrajectoryVisualizer tv_;
        BackgroundSubtractor bs_;
        FlowNeighbourSimilarityCalculator fs_;
        OutlierDetector od_;

        cv::VideoWriter output_cap_;

        cv::RNG rng;

        int frame_number_;

};

#endif
