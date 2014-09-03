#ifndef EXPECTED_FLOW_CALCULATOR_H_
#define EXPECTED_FLOW_CALCULATOR_H_

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PCLPointCloud2 PointCloud;

class ExpectedFlowCalculator
{

    public:
        
        ExpectedFlowCalculator();
        
        virtual ~ExpectedFlowCalculator();

        std::vector<std::vector<double> > calculateExpectedFlow(PointCloud frame, std::vector<double> odom, cv::Mat &frame_image, cv::Mat &projected_image);        

        void setCameraParameters(cv::Mat camera_matrix, cv::Mat translation, cv::Mat rotation, cv::Mat distortion);

    private:

        cv::Mat camera_matrix;

        cv::Mat camera_translation;
        
        cv::Mat camera_rotation;

        cv::Mat camera_distortion;

    private:

        cv::Mat getOpenCVPoints(pcl::PointCloud<PointT>::Ptr frame);

};

#endif
