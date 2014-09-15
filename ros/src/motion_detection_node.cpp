#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <motion_detection/expected_flow_calculator.h>
#include <motion_detection/optical_flow_calculator.h>
#include <motion_detection/flow_clusterer.h>
#include <motion_detection/flow_difference_calculator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

class MotionDetectionNode
{
    public:
        MotionDetectionNode(ros::NodeHandle &n) : nh_(n), it_(n)
        {
            cloud_received = false;
            image_received = false;
            first_image_received = false;
            camera_params_set = false;
            image_publisher = it_.advertise("optical_flow_image", 1);
            image1_publisher = it_.advertise("image1", 1);
            image2_publisher = it_.advertise("image2", 1);
            expected_flow_publisher = it_.advertise("expected_flow_image", 1);

            cloud_subscriber = nh_.subscribe("input_pointcloud", 1, &MotionDetectionNode::cloudCallback, this);
            image_subscriber = it_.subscribe("input_image", 1, &MotionDetectionNode::imageCallback, this);

            camera_info_subscriber = nh_.subscribe("input_camerainfo", 1, &MotionDetectionNode::cameraCallback, this);
        }

        void runExpectedFlow()
        {
           while (!cloud_received || !image_received || !camera_params_set)
           {
               ros::Rate(100).sleep();
               ros::spinOnce();
           }

           //ROS_INFO("received cloud");

           cv_bridge::CvImagePtr cv_image;

           cv_image = cv_bridge::toCvCopy(raw_image2, "rgb8");
           cv::Mat projected_image;

           std::vector<double> odom;
           double x_trans, y_trans, z_trans, roll, pitch, yaw;
           nh_.getParam("x_trans", x_trans);
           nh_.getParam("y_trans", y_trans);
           nh_.getParam("z_trans", z_trans);
           nh_.getParam("roll", roll);
           nh_.getParam("pitch", pitch);
           nh_.getParam("yaw", yaw);
           odom.push_back(x_trans);
           odom.push_back(y_trans);
           odom.push_back(z_trans);
           odom.push_back(roll);
           odom.push_back(pitch);
           odom.push_back(yaw);


           pcl::PCLPointCloud2 pc2;
           pcl_conversions::toPCL(cloud, pc2);

          // ROS_INFO("cloud width %i", cloud.width);
         //  ROS_INFO("cloud height %i", cloud.height);

         //  ROS_INFO("getting expected flow");
           cv::Mat expected_flow_vectors = cv::Mat::zeros(cv_image->image.rows, cv_image->image.cols, CV_32FC4);
           efc.calculateExpectedFlow(pc2, odom, cv_image->image, projected_image, expected_flow_vectors);


           cv_bridge::CvImage expected_flow_image_msg;
           expected_flow_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
           expected_flow_image_msg.image = cv_image->image;
           expected_flow_publisher.publish(expected_flow_image_msg.toImageMsg());
          // ROS_INFO("got expected flow");
           
           
           cv_bridge::CvImagePtr cv_image1;
           cv_bridge::CvImagePtr cv_image2;

           cv_image1 = cv_bridge::toCvCopy(raw_image1, "rgb8");
           cv_image2 = cv_bridge::toCvCopy(raw_image2, "rgb8");

           cv::Mat optical_flow_image;

           cv::Mat optical_flow_vectors = cv::Mat::zeros(cv_image->image.rows, cv_image->image.cols, CV_32FC4);
           int num_vectors = ofc.calculateOpticalFlow(cv_image1->image, cv_image2->image, optical_flow_image, optical_flow_vectors);
           //std::cout << "test : " << optical_flow_vectors.at<cv::Vec4d>(160, 160) << std::endl;
           //std::cout << "test : " << optical_flow_vectors.at<cv::Vec4d>(140, 120) << std::endl;
          // std::cout << "-----------------------" << std::endl;
           //std::cout << optical_flow_vectors << std::endl;
         //  std::cout << "-----------------------" << std::endl;
           cv::Mat centers;
           if (num_vectors > 0)
           {
               std::cout <<"num_vectors: " << num_vectors << std::endl;
               centers = fc.clusterFlowVectors(optical_flow_vectors);          
           //    std::cout << centers << std::endl;
               for (int i = 0; i < centers.rows; i++)
               {                 
                   //std::cout << centers.at<float>(i,0) << ", " << centers.at<float>(i,1) << ", " << centers.at<float>(i,2) << std::endl; 
                   cv::circle(optical_flow_image, cv::Point((int)centers.at<float>(i,0), (int)centers.at<float>(i,1)), 5.0, cv::Scalar(255,0,0), -1, 8);
               }
           }
           
           cv::Mat difference_vectors = cv::Mat::zeros(cv_image->image.rows, cv_image->image.cols, CV_32FC4);
           fdc.calculateFlowDifference(optical_flow_vectors, expected_flow_vectors, difference_vectors);

           //ofc.varFlow(cv_image1->image, cv_image2->image, optical_flow_image, optical_flow_vectors);
           
           cv_bridge::CvImage optical_flow_image_msg;
           optical_flow_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
           optical_flow_image_msg.image = optical_flow_image;
           image_publisher.publish(optical_flow_image_msg.toImageMsg());
           image1_publisher.publish(raw_image1);
           image2_publisher.publish(raw_image2);

          


           cloud_received = false;
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
        image_transport::Subscriber image_subscriber;
        image_transport::Publisher image_publisher;
        image_transport::Publisher image1_publisher;
        image_transport::Publisher image2_publisher;
        image_transport::Publisher expected_flow_publisher;
        bool cloud_received;
        bool image_received;
        bool first_image_received;
        bool camera_params_set;

        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::ImageConstPtr raw_image1;
        sensor_msgs::ImageConstPtr raw_image2;
        OpticalFlowCalculator ofc;
        ExpectedFlowCalculator efc;
        FlowClusterer fc;
        FlowDifferenceCalculator fdc;

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
        mdn.runExpectedFlow();
    }

    return 0;
}
