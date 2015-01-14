/* expected_flow_calculator.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
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

        void calculateExpectedFlow(PointCloud frame, std::vector<double> odom, cv::Mat &projected_image, cv::Mat &optical_flow_vectors, int pixel_step);

        void setCameraParameters(cv::Mat camera_matrix, cv::Mat translation, cv::Mat rotation, cv::Mat distortion);

    private:

        cv::Mat camera_matrix;

        cv::Mat camera_translation;
        
        cv::Mat camera_rotation;

        cv::Mat camera_distortion;

    private:

        std::vector<cv::Point3f> getOpenCVPoints(pcl::PointCloud<PointT>::Ptr frame);

};

#endif
