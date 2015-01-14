/* motion_detection_node.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

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
    global_frame_count_ = 0;
    write_trajectories_ = false;

    nh_.param<bool>("use_odom", use_odom_, false);
    nh_.param<bool>("use_pointcloud", use_pointcloud_, false);
    nh_.param<bool>("record_video", record_video_, false);
    nh_.param<bool>("use_all_frames", use_all_frames_, false);
    nh_.param<bool>("write_vectors", write_vectors_, false);
    nh_.param<bool>("log_contours", log_contours_, false);
    nh_.param<bool>("egomotion", egomotion_, true);
    std::string log_path;
    nh_.param<std::string>("log_path", log_path, "");
    nh_.param<bool>("include_zeros", include_zeros_, false);
    nh_.param<double>("min_vector_size", min_vector_size_, 1.0);

    of_image_publisher_ = it_.advertise("optical_flow_image", 1);
    image1_publisher_ = it_.advertise("image1", 1);
    image2_publisher_ = it_.advertise("image2", 1);
    expected_flow_publisher_ = it_.advertise("expected_flow_image", 1);
    compensated_flow_publisher_ = it_.advertise("compensated_flow_image", 1);
    clustered_flow_publisher_ = it_.advertise("clustered_flow_image", 1);
    background_subtraction_publisher_ = it_.advertise("background_subtraction_image", 1);

    if (log_contours_)
    {
        ml_.setFileName(log_path);
    }

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
    int num_vectors = ofc_.calculateOpticalFlow(image1, image2, optical_flow_vectors, pixel_step_, debug_image, min_vector_size_);
    ofv_.showOpticalFlowVectors(image1, optical_flow_image, optical_flow_vectors, pixel_step_, CV_RGB(0, 0, 255), min_vector_size_);

    publishImage(optical_flow_image, of_image_publisher_);
    if (record_video_)
    {
        cv::Mat mat2;
        cv::cvtColor(optical_flow_image, mat2, CV_RGB2BGR);
        output_cap_.write(mat2);
    }
}

void MotionDetectionNode::runOpticalFlowTrajectory(const std::vector<cv::Mat> &images, cv::Mat &optical_flow_vectors, std::vector<std::vector<cv::Point2f> > &trajectories, cv::Mat &optical_flow_image)
{
    cv::Mat debug_image;

    optical_flow_vectors = cv::Mat::zeros(images[0].rows, images[0].cols, CV_32FC4);
    int num_vectors = ofc_.calculateOpticalFlowTrajectory(images, optical_flow_vectors, trajectories, pixel_step_, debug_image, min_vector_size_);
//    int num_vectors = ofc_.calculateOpticalFlow(image1, image2, optical_flow_vectors, pixel_step_, debug_image, min_vector_size_);
    ofv_.showOpticalFlowVectors(images.back(), optical_flow_image, optical_flow_vectors, pixel_step_, CV_RGB(0, 0, 255), min_vector_size_);

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

    cv::Mat outlier_vectors;
    od_.getOutlierVectors(optical_flow_vectors, outlier_mask, outlier_vectors, pixel_step_);
    std::vector<std::vector<cv::Vec4d> > clusters;

    double distance_threshold, angular_threshold;
    nh_.getParam("distance_threshold", distance_threshold);
    nh_.getParam("angular_threshold", angular_threshold);
    clusters = fc_.getClusters(outlier_vectors, pixel_step_, distance_threshold, angular_threshold);

    cv::Mat clustered_flow_image;
    original_image.copyTo(clustered_flow_image);

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
            ofv_.showFlowClusters(original_image, clustered_flow_image, clusters.at(i), pixel_step_, colour, min_vector_size_);
        }
        else
        {
            ofv_.showFlowClusters(clustered_flow_image, temp, clusters.at(i), pixel_step_, colour, min_vector_size_);
            temp.copyTo(clustered_flow_image);
        }
    }

    publishImage(clustered_flow_image, clustered_flow_publisher_);

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
            ofv_.showFlowClusters(image, clustered_flow_image, clusters.at(i), pixel_step_, colour, min_vector_size_);
        }
        else
        {
            ofv_.showFlowClusters(clustered_flow_image, temp, clusters.at(i), pixel_step_, colour, min_vector_size_);
            temp.copyTo(clustered_flow_image);
        }
    }

    publishImage(clustered_flow_image, clustered_flow_publisher_);
}

void MotionDetectionNode::writeVectors(const cv::Mat &flow_vectors, const std::string &filename)
{
    ofc_.writeFlow(flow_vectors, filename, pixel_step_); 
}

void MotionDetectionNode::writeTrajectories(const std::vector<std::vector<cv::Point2f> > &trajectories, const std::string &filename)
{
    ofc_.writeTrajectories(trajectories, filename); 
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
    int skip_frames;
    nh_.param<int>("skip_frames", skip_frames, 1);
    int num_motions;
    nh_.param<int>("num_motions", num_motions, 2);
    trajectory_size_ = num_motions * 2 + 1;
    if (!egomotion_)
    {
        trajectory_size_ = 2;
    }

    if (global_frame_count_ % skip_frames != 0) { global_frame_count_++; return;}
    if (raw_images_.size() < trajectory_size_)
    {
        raw_images_.push_back(image);        
        if (raw_images_.size() == trajectory_size_)
        {
            image_received_ = true;
        }
    }
    else
    {
        raw_images_.push_back(image);
        raw_images_.pop_front();
        image_received_ = true;
    }
    if (use_all_frames_ && image_received_ == true)
    {
        nh_.getParam("pixel_step", pixel_step_);
        image_received_ = true;
        std::vector<cv::Mat> cv_images;
        std::list<sensor_msgs::ImageConstPtr>::iterator iter = raw_images_.begin();
        for (; iter != raw_images_.end(); ++iter)            
        {
            cv_bridge::CvImagePtr cv_image;
            cv_image = cv_bridge::toCvCopy(*iter, "rgb8");
            if (cv_image->image.cols > 320)
            {
                /*
                cv::Rect region(200, 0, cv_image->image.cols - 400, cv_image->image.rows);
                cv::Mat resized = cv_image->image(region);
                cv::resize(resized, resized, cv::Size(), 0.5, 0.5);
                cv_images.push_back(resized);
                */
                cv_images.push_back(cv_image->image);

            }
            else
            {
                cv_images.push_back(cv_image->image);
            }
        }
        cv::Mat optical_flow_vectors;
        cv::Mat outlier_mask;
        std::vector<std::vector<cv::Point2f> > trajectories;
        //std::vector<std::vector<cv::Vec4d> > clusters;
        std::vector<std::vector<cv::Point2f> > clusters;
        //runOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_vectors);
        cv::Mat optical_flow_image;
        runOpticalFlowTrajectory(cv_images, optical_flow_vectors, trajectories, optical_flow_image);
        if (trajectories.empty())
        {
            std::cout << "no trajectories found " << std::endl;
            cv::Mat cluster_image;
            cv::Mat trajectory_image;
            cv::Mat optical_flow_image;
            cv_images.back().copyTo(cluster_image);
            cv_images.back().copyTo(trajectory_image);
            cv_images.back().copyTo(optical_flow_image);

            publishImage(trajectory_image, background_subtraction_publisher_);
            publishImage(cluster_image, clustered_flow_publisher_);
            cv::Mat combined_image(2 * cluster_image.rows, cluster_image.cols, CV_8UC3);
            cv::Mat top(combined_image, cv::Rect(0, 0, optical_flow_image.cols, optical_flow_image.rows));
            optical_flow_image.copyTo(top);
            cv::Mat bottom(combined_image, cv::Rect(0, optical_flow_image.rows, cluster_image.cols, cluster_image.rows));
            cluster_image.copyTo(bottom);
            publishImage(combined_image, compensated_flow_publisher_);
            frame_number_++;
            global_frame_count_++;

            return;
        }

        double distance_threshold;
        nh_.getParam("distance_threshold", distance_threshold);

        /*
        int long_trajectories = 0;
        for (int i = 0; i < trajectories.size(); i++)
        {
            std::vector<cv::Point2f> line = trajectories.at(i);
            int length = line.size();
            double traj_length = std::sqrt( (std::pow((line[0].x - line[length-1].x), 2.0) + std::pow((line[0].y - line[length-1].y), 2.0)));
            if (traj_length > distance_threshold / 5)
            {
                long_trajectories++;
            }
            else
            {
                std::cout << "length: " << traj_length << std::endl;
            }
        }
        std::cout << "long trajectories " << long_trajectories << std::endl;
        */
        //if (long_trajectories > (0.5 * cv_images[0].rows * cv_images[0].cols) / (pixel_step_ * pixel_step_))
        if (true)
        {
            std::vector<cv::Point2f> outlier_points;
            double sigma;
            nh_.param<double>("sigma", sigma, 0.5);
            std::vector<std::vector<cv::Point2f> > trajectory_subspace_vectors;
            trajectory_subspace_vectors = od_.fitSubspace(trajectories, outlier_points, num_motions, sigma);

            cv::Mat trajectory_image;
            tv_.showTrajectories(cv_images.back(), trajectory_image, trajectory_subspace_vectors);
            // TODO: rename the publisher
            publishImage(trajectory_image, background_subtraction_publisher_);

            clusters = fc_.clusterEuclidean(outlier_points, distance_threshold);
        }
        else
        {
            ROS_INFO("no egomotion");
            std::vector<cv::Point2f> points;
            for (int i = 0; i < trajectories.size(); i++)
            {
                int length = trajectories.at(i).size();
                std::vector<cv::Point2f> line = trajectories.at(i);
                double traj_length = std::sqrt( (std::pow((line[0].x - line[length-1].x), 2.0) + std::pow((line[0].y - line[length-1].y), 2.0)));
                if (traj_length > distance_threshold / 5)
                {
                    points.push_back(trajectories.at(i).at(length - 1));
                }
            }
            //clusters = fc_.clusterEuclidean(points, distance_threshold);
            std::vector<std::vector<cv::Vec4d> > cluster_vec;
            double angular_threshold;
            nh_.getParam("angular_threshold", angular_threshold);
            cluster_vec = fc_.getClusters(optical_flow_vectors, pixel_step_, distance_threshold, angular_threshold);
            for (int i = 0; i < cluster_vec.size(); i++)
            {
                std::vector<cv::Point2f> cc;
                std::vector<cv::Vec4d> cc_v = cluster_vec.at(i);
                for (int j = 0; j < cc_v.size(); j++)
                {
                    cv::Point2f point(cc_v.at(j)[0], cc_v.at(j)[1]);
                    cc.push_back(point);
                }
                clusters.push_back(cc);
            }
            std::cout << "clusters size " << clusters.size() << std::endl;
            std::cout << "clusters  vecsize " << cluster_vec.size() << std::endl;
        }

        cv::Mat cluster_image;
        std::vector<std::vector<cv::Point> > contours;
        //contours = ofv_.showClusterContours(cv_images.back(), cluster_image, clusters);        
        std::vector<cv::Rect> rectangles;
        rectangles = ofv_.showBoundingBoxes(cv_images.back(), cluster_image, clusters);
        publishImage(cluster_image, clustered_flow_publisher_);

        cv::Mat combined_image(2 * cluster_image.rows, cluster_image.cols, CV_8UC3);
        cv::Mat top(combined_image, cv::Rect(0, 0, optical_flow_image.cols, optical_flow_image.rows));
        optical_flow_image.copyTo(top);
        cv::Mat bottom(combined_image, cv::Rect(0, optical_flow_image.rows, cluster_image.cols, cluster_image.rows));
        cluster_image.copyTo(bottom);
        publishImage(combined_image, compensated_flow_publisher_);

        //detectOutliers(cv_image1->image, optical_flow_vectors, outlier_mask, include_zeros_); 
        //clusterFlow(cv_image1->image, optical_flow_vectors, clusters);
        frame_number_++;
        if (write_vectors_)
        {
            std::stringstream ss;
            ss << frame_number_;
            std::string filename = "/home/santosh/data/frame" + ss.str();
            writeVectors(optical_flow_vectors, filename);
        }
        if (write_trajectories_)
        {
            std::stringstream ss;
            ss << frame_number_;
            std::string filename = "/home/santosh/workspace/rnd/outlier/frame" + ss.str();
            writeTrajectories(trajectories, filename);
            cv::imwrite("/home/santosh/workspace/rnd/outlier/frame.jpg", cv_images.back());
        }
        if (log_contours_)
        {
            /*
            for (int i = 0; i < contours.size(); i++)
            {
                ml_.writeContour(contours.at(i), global_frame_count_, i);
            }
            */
            for (int i = 0; i < rectangles.size(); i++)
            {
                ml_.writeBoundingBox(rectangles.at(i), global_frame_count_, i);
            }
        }
        bool save_frames;
        nh_.param<bool>("save_frames", save_frames, false);
        if (save_frames)
        {
            std::string frames_path;
            nh_.param<std::string>("frames_path", frames_path, "");
            if (!frames_path.empty())
            {
                if (global_frame_count_ % 5 == 0)
                {
                    std::stringstream ss;
                    ss << global_frame_count_;
                    cv::cvtColor(combined_image, combined_image, CV_BGR2RGB);
                    cv::imwrite(frames_path + "frame" + ss.str() + ".jpg", combined_image); 
                }
            }
        }
    }
    global_frame_count_++;
}

void MotionDetectionNode::run()
{
    while ((!cloud_received_ && use_pointcloud_) || !image_received_ || (!camera_params_set_ && use_pointcloud_) || (!odom_received_ && use_odom_))
    {
        ros::Rate(100).sleep();
        ros::spinOnce();
    }
    std::vector<cv::Mat> cv_images;
    std::list<sensor_msgs::ImageConstPtr>::iterator iter = raw_images_.begin();
    for (; iter != raw_images_.end(); ++iter)            
    {
        cv_bridge::CvImagePtr cv_image;
        cv_image = cv_bridge::toCvCopy(*iter, "rgb8");
        if (cv_image->image.cols > 320)
        {
            cv::Mat resized;
            cv::resize(cv_image->image, resized, cv::Size(), 0.5, 0.5);
            cv_images.push_back(resized);
        }
        else
        {
            cv_images.push_back(cv_image->image);
        }
    }
    cv::Mat optical_flow_vectors;
    cv::Mat outlier_mask;
    std::vector<std::vector<cv::Point2f> > trajectories;
    //std::vector<std::vector<cv::Vec4d> > clusters;
    std::vector<std::vector<cv::Point2f> > clusters;
    //runOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_vectors);
    cv::Mat optical_flow_image;
    runOpticalFlowTrajectory(cv_images, optical_flow_vectors, trajectories, optical_flow_image);
    std::vector<cv::Point2f> outlier_points;
    double residual_threshold;
    nh_.param<double>("residual_threshold", residual_threshold, 0.2);
    int num_motions;
    nh_.param<int>("num_motions", num_motions, 2);
    od_.fitSubspace(trajectories, outlier_points, 2, residual_threshold); 
    double distance_threshold;
    nh_.getParam("distance_threshold", distance_threshold);
    clusters = fc_.clusterEuclidean(outlier_points, distance_threshold);
    /*
    std::cout << "clusters" << clusters.size() << std::endl;
    for (int i = 0; i < clusters.size(); i++)
    {
        std::cout << "size: " << clusters.at(i).size() << std::endl;
        for (int j = 0; j < clusters.at(i).size(); j++)
        {
            std::cout << clusters.at(i).at(j) << std::endl;
        }
    }
    */
    cv::Mat cluster_image;
    ofv_.showClusterContours(cv_images.back(), cluster_image, clusters);
    publishImage(cluster_image, clustered_flow_publisher_);
    //detectOutliers(cv_image1->image, optical_flow_vectors, outlier_mask, include_zeros_); 
    //clusterFlow(cv_image1->image, optical_flow_vectors, clusters);
    frame_number_++;
    if (write_vectors_)
    {
        std::stringstream ss;
        ss << frame_number_;
        std::string filename = "/home/santosh/data/frame" + ss.str();
        writeVectors(optical_flow_vectors, filename);
    }
    if (write_trajectories_)
    {
        std::stringstream ss;
        ss << frame_number_;
        std::string filename = "/home/santosh/workspace/rnd/outlier/frame" + ss.str();
        writeTrajectories(trajectories, filename);
        cv::imwrite("/home/santosh/workspace/rnd/outlier/frame.jpg", cv_images.back());
    }
    /*
    image_received_ = false;
    frame_number_++;
    cv_bridge::CvImagePtr cv_image1;
    cv_bridge::CvImagePtr cv_image2;
    cv_image1 = cv_bridge::toCvCopy(raw_image1_, "rgb8");
    cv_image2 = cv_bridge::toCvCopy(raw_image2_, "rgb8");

    cv::Mat optical_flow_vectors;
    cv::Mat outlier_mask;
    std::vector<std::vector<cv::Vec4d> > clusters;
    runOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_vectors);
    detectOutliers(cv_image1->image, optical_flow_vectors, outlier_mask, include_zeros_); 
    //clusterFlow(cv_image1->image, optical_flow_vectors, clusters);
    frame_number_++;
    if (write_vectors_)
    {
        std::stringstream ss;
        ss << frame_number_;
        std::string filename = "/home/santosh/data/frame" + ss.str();
        writeVectors(optical_flow_vectors, filename);
    }
    */
}


void MotionDetectionNode::cameraInfoCallback(const sensor_msgs::CameraInfo &camera_info)    
{
    
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "motion_detection");

    ros::NodeHandle n("~");

    bool use_all_frames;
    n.param<bool>("use_all_frames", use_all_frames, true);

    ROS_INFO("[motion_detection] node started");

    MotionDetectionNode mdn(n); 
    if (use_all_frames)
    {
        ros::spin();
    }
    else
    {
        while(ros::ok())
        {
            mdn.run();
        }
    }

    return 0;
}
