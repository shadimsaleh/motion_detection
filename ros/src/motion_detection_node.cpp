#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <motion_detection/expected_flow_calculator.h>
#include <motion_detection/optical_flow_calculator.h>
#include <motion_detection/flow_clusterer.h>
#include <motion_detection/flow_difference_calculator.h>
#include <motion_detection/optical_flow_visualizer.h>
#include <motion_detection/background_subtractor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MotionDetectionNode
{
    public:
        MotionDetectionNode(ros::NodeHandle &n) : nh_(n), it_(n), rng(12345)
        {
            cloud_received = false;
            image_received = false;
            odom_received = false;
            first_image_received = false;
            camera_params_set = false;
            first_run = true;
            image_publisher = it_.advertise("optical_flow_image", 1);
            image1_publisher = it_.advertise("image1", 1);
            image2_publisher = it_.advertise("image2", 1);
            expected_flow_publisher = it_.advertise("expected_flow_image", 1);
            compensated_flow_publisher = it_.advertise("compensated_flow_image", 1);
            clustered_flow_publisher = it_.advertise("clustered_flow_image", 1);
            background_subtraction_publisher = it_.advertise("background_subtraction_image", 1);

            cloud_subscriber = nh_.subscribe("input_pointcloud", 1, &MotionDetectionNode::cloudCallback, this);
            image_subscriber = it_.subscribe("input_image", 1, &MotionDetectionNode::imageCallback, this);
            odom_subscriber = nh_.subscribe("/odom", 1, &MotionDetectionNode::odomCallback, this);

            camera_info_subscriber = nh_.subscribe("input_camerainfo", 1, &MotionDetectionNode::cameraCallback, this);
        }

        void runExpectedFlow()
        {           
           while (!cloud_received || !image_received || !camera_params_set || !odom_received)
           {
               ros::Rate(100).sleep();
               ros::spinOnce();
           }

           image_received = false;
           cloud_received = false;
           odom_received = false;

           cv_bridge::CvImagePtr cv_image1;
           cv_bridge::CvImagePtr cv_image2;
           cv_image1 = cv_bridge::toCvCopy(raw_image1, "rgb8");
           cv_image2 = cv_bridge::toCvCopy(raw_image2, "rgb8");
           //ROS_INFO("received cloud");

           cv_bridge::CvImagePtr cv_image;

           cv_image = cv_bridge::toCvCopy(raw_image2, "rgb8");
           cv::Mat projected_image;

           if(first_run)
           {
               cv::Mat contour_image;
               bs.getMotionContours(cv_image1->image, contour_image); 
               first_run = false;
           }
#ifdef MANUAL_ODOM
           std::vector<double> odom;
           double x_trans, y_trans, z_trans, roll, pitch, yaw;
           int pixel_step;
           double distance_threshold, angular_threshold;
           nh_.getParam("x_trans", x_trans);
           nh_.getParam("y_trans", y_trans);
           nh_.getParam("z_trans", z_trans);
           nh_.getParam("roll", roll);
           nh_.getParam("pitch", pitch);
           nh_.getParam("yaw", yaw);
           nh_.getParam("pixel_step", pixel_step);
           nh_.getParam("distance_threshold", distance_threshold);
           nh_.getParam("angular_threshold", angular_threshold);
           odom.push_back(x_trans);
           odom.push_back(y_trans);
           odom.push_back(z_trans);
           odom.push_back(roll);
           odom.push_back(pitch);
           odom.push_back(yaw);
#else
           std::vector<double> odom;
           double x_trans, y_trans, z_trans, roll, pitch, yaw;
           int pixel_step;
           double distance_threshold, angular_threshold;

           /*
           tf::Quaternion q;
           tf::quaternionMsgToTF(odom_.pose.pose.orientation, q);
           tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
           */

           z_trans = odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
           y_trans = odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
           x_trans = odom_.pose.pose.position.z - prev_odom_.pose.pose.position.z;
           odom.push_back(x_trans);
           odom.push_back(0.0);
           odom.push_back(z_trans);
           odom.push_back(0.0);
           odom.push_back(0.0);
           odom.push_back(0.0);

           prev_odom_ = odom_;

           nh_.getParam("pixel_step", pixel_step);
           nh_.getParam("distance_threshold", distance_threshold);
           nh_.getParam("angular_threshold", angular_threshold);
#endif

           pcl::PCLPointCloud2 pc2;
           pcl_conversions::toPCL(cloud, pc2);
           
//           cv::Mat empty_image = cv::Mat::ones(cv_image->image.rows, cv_image->image.cols, CV_8UC3);

           cv::Mat expected_flow_vectors = cv::Mat::zeros(cv_image->image.rows, cv_image->image.cols, CV_32FC4);
           efc.calculateExpectedFlow(pc2, odom, projected_image, expected_flow_vectors, pixel_step);
           ofv.showOpticalFlowVectors(cv_image1->image, cv_image->image, expected_flow_vectors, pixel_step, CV_RGB(0, 0, 255));
//           ofv.showOpticalFlowVectors(cv_image1->image, empty_image, expected_flow_vectors, pixel_step);


           cv_bridge::CvImage expected_flow_image_msg;
           expected_flow_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
           expected_flow_image_msg.image = cv_image->image;
//           expected_flow_image_msg.image = empty_image;
           expected_flow_publisher.publish(expected_flow_image_msg.toImageMsg());
           
           


           cv::Mat optical_flow_image;

           cv::Mat optical_flow_vectors = cv::Mat::zeros(cv_image->image.rows, cv_image->image.cols, CV_32FC4);
           int num_vectors = ofc.calculateOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_vectors, pixel_step);
           
           ofv.showOpticalFlowVectors(cv_image1->image, optical_flow_image, optical_flow_vectors, pixel_step, CV_RGB(0, 0, 255));
           //int num_vectors = ofc.superPixelFlow(cv_image1->image, cv_image2->image, optical_flow_image, optical_flow_vectors);
           
           //cv::Mat centers;
           std::vector<cv::Point2f> centers;
#ifdef DRAW_CIRCLES
           if (num_vectors > 0)
           {
               //centers = fc.clusterFlowVectors(optical_flow_vectors);          
               centers = fc.getClusterCenters(optical_flow_vectors, pixel_step, distance_threshold, angular_threshold);
               /*
               for (int i = 0; i < centers.rows; i++)
               {                 
                   cv::circle(optical_flow_image, cv::Point((int)centers.at<float>(i,0), (int)centers.at<float>(i,1)), 5.0, cv::Scalar(0,0,255), -1, 8);
               }
               */
               for (int i = 0; i < centers.size(); i++)
               {
                   cv::circle(optical_flow_image, centers.at(i), 5.0, cv::Scalar(0, 0, 255), -1, 8);
               }
           }
#else
           if (num_vectors > 0)
           {
               cv::Mat clustered_flow_image;
               std::vector<cv::Mat> clusters = fc.getClusters(optical_flow_vectors, pixel_step, distance_threshold, angular_threshold);
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
               //        ofv.showOpticalFlowVectors(cv_image1->image, clustered_flow_image, clusters.at(i), pixel_step, cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0,255)));
                       ofv.showOpticalFlowVectors(cv_image1->image, clustered_flow_image, clusters.at(i), pixel_step, colour);
                   }
                   else
                   {
                       ofv.showOpticalFlowVectors(clustered_flow_image, temp, clusters.at(i), pixel_step, colour);
                       temp.copyTo(clustered_flow_image);
                   }
               }


               cv_bridge::CvImage clustered_flow_image_msg;
               clustered_flow_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
               clustered_flow_image_msg.image = clustered_flow_image;
               clustered_flow_publisher.publish(clustered_flow_image_msg.toImageMsg());
           }
#endif
           
           cv::Mat difference_vectors = cv::Mat::zeros(cv_image->image.rows, cv_image->image.cols, CV_32FC4);
           fdc.calculateFlowDifference(optical_flow_vectors, expected_flow_vectors, difference_vectors, pixel_step);

           cv::Mat compensated_optical_flow_image;
           ofv.showOpticalFlowVectors(cv_image1->image, compensated_optical_flow_image, difference_vectors, pixel_step, CV_RGB(0, 0, 255)); 
           
           cv_bridge::CvImage compensated_flow_image_msg;
           compensated_flow_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
           compensated_flow_image_msg.image = compensated_optical_flow_image;
           compensated_flow_publisher.publish(compensated_flow_image_msg.toImageMsg());

           cv_bridge::CvImage optical_flow_image_msg;
           optical_flow_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
           optical_flow_image_msg.image = optical_flow_image;
           image_publisher.publish(optical_flow_image_msg.toImageMsg());
           image1_publisher.publish(raw_image1);
           image2_publisher.publish(raw_image2);


           cv::Mat bs_contours;
           bs.getMotionContours(cv_image2->image, bs_contours);
           cv_bridge::CvImage bs_contours_msg;
           bs_contours_msg.encoding = sensor_msgs::image_encodings::RGB8;
           bs_contours_msg.image = bs_contours;
           background_subtraction_publisher.publish(bs_contours_msg.toImageMsg());
        }

        void runOpticalFlow()
        {
           while (!image_received)
           {
               ros::Rate(100).sleep();
               ros::spinOnce();
           }

           //ROS_INFO("received image");
           

           cv_bridge::CvImagePtr cv_image1;
           cv_bridge::CvImagePtr cv_image2;

           cv_image1 = cv_bridge::toCvCopy(raw_image1, "rgb8");
           cv_image2 = cv_bridge::toCvCopy(raw_image2, "rgb8");

           cv::Mat optical_flow_image;

           //ofc.calculateOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_image);
           cv::Mat optical_flow_vectors = cv::Mat::zeros(cv::Size(cv_image1->image.cols, cv_image1->image.rows), CV_32FC4);
           ofc.varFlow(cv_image1->image, cv_image2->image, optical_flow_image, optical_flow_vectors);
           
           cv_bridge::CvImage optical_flow_image_msg;
           optical_flow_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
           optical_flow_image_msg.image = optical_flow_image;
           image_publisher.publish(optical_flow_image_msg.toImageMsg());
           image1_publisher.publish(raw_image1);
           image2_publisher.publish(raw_image2);
           image_received = false;
        }
        
        void odomCallback(const nav_msgs::Odometry &odom)
        {
            odom_ = odom;
            odom_received = true;
        }

        void cloudCallback(const sensor_msgs::PointCloud2 &cloud)
        {
            this->cloud = cloud;
            cloud_received = true;
        }

        void imageCallback(const sensor_msgs::ImageConstPtr &image)
        {
            if (first_image_received)
            {
                raw_image1 = raw_image2;
                raw_image2 = image;

                image_received = true;
            }
            else
            {
                raw_image2 = image;
                first_image_received = true;
            }
        }

        void cameraCallback(const sensor_msgs::CameraInfo &camera_info)
        {
            double matrix[3][3] = { {570.3422241210938, 0.0, 319.5}, {0.0, 570.3422241210938, 239.5}, {0.0, 0.0, 1.0} };
            std::copy(&camera_info.K[0], &camera_info.K[0] + 9, &matrix[0][0]);
            cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, matrix);

            double rotation[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
            std::copy(&camera_info.R[0], &camera_info.R[0] + 9, &rotation[0][0]);
            cv::Mat camera_rotation_matrix = cv::Mat(3, 3, CV_64F, rotation);
            cv::Mat camera_rotation_vector;
            cv::Rodrigues(camera_rotation_matrix, camera_rotation_vector);

            double translation[3] = {camera_info.P[3], camera_info.P[7], camera_info.P[11]};
            cv::Mat camera_translation = cv::Mat(1, 3, CV_64F, translation);
            
            double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
            std::copy(&camera_info.D[0], &camera_info.D[0] + 5, &distortion[0]);
            cv::Mat camera_distortion = cv::Mat(1, 5, CV_64F, distortion);

            efc.setCameraParameters(camera_matrix, camera_translation, camera_rotation_vector, camera_distortion);
            camera_params_set = true;
        }

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        ros::Subscriber cloud_subscriber;
        ros::Subscriber camera_info_subscriber;
        ros::Subscriber odom_subscriber;
        image_transport::Subscriber image_subscriber;
        image_transport::Publisher image_publisher;
        image_transport::Publisher image1_publisher;
        image_transport::Publisher image2_publisher;
        image_transport::Publisher expected_flow_publisher;
        image_transport::Publisher compensated_flow_publisher;
        image_transport::Publisher clustered_flow_publisher;
        image_transport::Publisher background_subtraction_publisher;
        bool cloud_received;
        bool image_received;
        bool odom_received;        
        bool first_image_received;
        bool camera_params_set;
        bool first_run;

        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::ImageConstPtr raw_image1;
        sensor_msgs::ImageConstPtr raw_image2;
        nav_msgs::Odometry odom_;
        nav_msgs::Odometry prev_odom_;
        OpticalFlowCalculator ofc;
        ExpectedFlowCalculator efc;
        FlowClusterer fc;
        FlowDifferenceCalculator fdc;
        OpticalFlowVisualizer ofv;
        BackgroundSubtractor bs;

        cv::RNG rng;

};
int main(int argc, char **argv)
{  
    ros::init(argc, argv, "motion_detection");

    ros::NodeHandle n("~");

    ROS_INFO("[motion_detection] node started");

    MotionDetectionNode mdn(n); 
    while(ros::ok())
    {
   //     mdn.runOpticalFlow();
        double start_time = (double)clock() / CLOCKS_PER_SEC;
        mdn.runExpectedFlow();
        double end_time = (double)clock() / CLOCKS_PER_SEC;
        //std::cout << "Time: " << (end_time - start_time); 
        //ROS_INFO("time %.2f", (end_time - start_time));
    }

    return 0;
}
