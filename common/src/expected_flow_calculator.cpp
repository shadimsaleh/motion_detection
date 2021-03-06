/* expected_flow_calculator.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <motion_detection/expected_flow_calculator.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ExpectedFlowCalculator::ExpectedFlowCalculator()
{

}

ExpectedFlowCalculator::~ExpectedFlowCalculator()
{

}

void ExpectedFlowCalculator::setCameraParameters(cv::Mat camera_matrix, cv::Mat translation, cv::Mat rotation,
                                                 cv::Mat distortion)
{
    this->camera_matrix = camera_matrix.clone();
    this->camera_translation = translation.clone();
    this->camera_rotation = rotation.clone();
    this->camera_distortion = distortion.clone();
}

void ExpectedFlowCalculator::calculateExpectedFlow(PointCloud frame, 
                                                                          std::vector<double> odom,    
                                                                          cv::Mat &projected_image2,
                                                                          cv::Mat &expected_flow_vectors,
                                                                          int pixel_step)
{

    if (odom[2] == 0.0)
    {
        if (odom[0] == 0.0 && odom[1] == 0.0)
        {
            std::cout << "foe: no movement" << std::endl;
        }
        else if (odom[0] == 0.0)
        {
            if (odom[1] > 0.0)
            {
                std::cout << "foe: positive y infinity" << std::endl;
            }
            else
            {
                std::cout << "foe: negative y infinity" << std::endl;
            }
        }
        else if (odom[1] == 0)
        {
            if (odom[0] > 0.0)
            {
                std::cout << "foe: positive x infinity" << std::endl;
            }
            else
            {
                std::cout << "foe: negative x infinity" << std::endl;
            }
        }
    }
    else
    {
        double foe_x = odom[0] / odom[2];
        double foe_y = odom[1] / odom[2];
        std::cout << "foe: " << foe_x << ", " << foe_y << std::endl;
    }
    pcl::PointCloud<PointT>::Ptr frame1(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(frame, *frame1);

    cv::Mat projected_image1;
    std::vector<cv::Point3f> objectPoints = getOpenCVPoints(frame1);
    if (objectPoints.empty())
    {
        return;
    }

    std::vector<cv::Point2f> projected_points1(objectPoints.size());

    cv::projectPoints(objectPoints, camera_rotation, camera_translation, camera_matrix, camera_distortion, projected_points1);

    projected_image1 = cv::Mat(projected_points1);

    std::vector<cv::Point2f> projected_points2(objectPoints.size());
    double translation[3] = {odom[0], odom[1], odom[2]};
    cv::Mat camera_translation2 = cv::Mat(1, 3, CV_64F, translation);
    double rotation[3] = {odom[3], odom[4], odom[5]};
    cv::Mat camera_rotation2 = cv::Mat(1, 3, CV_64F, rotation);
    cv::projectPoints(objectPoints, camera_rotation2, camera_translation2, camera_matrix, camera_distortion, projected_points2);
    projected_image2 = cv::Mat(projected_points2);

    for (int i = 0; i < projected_points1.size(); i++)
    {

        cv::Point2f start_point(projected_points1[i]);
        cv::Point2f end_point(projected_points2[i]);        

        if ((int)start_point.x % pixel_step != 0 || (int)start_point.y % pixel_step != 0)
            continue;
        cv::Vec4d &elem = expected_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
        float x_diff = end_point.x - start_point.x;
        float y_diff = end_point.y - start_point.y;
        elem[0] = start_point.x;
        elem[1] = start_point.y;
        elem[2] = x_diff;
        elem[3] = y_diff;
        
      //  elem[2] = atan2(y_diff, x_diff);
      //  elem[3] = sqrt(x_diff*x_diff + y_diff*y_diff);
    }
}


std::vector<cv::Point3f> ExpectedFlowCalculator::getOpenCVPoints(pcl::PointCloud<PointT>::Ptr frame)
{
    std::vector<cv::Point3f> points;
    //int m = 0;
    for (int i = 0; i < frame->points.size(); i++)
    {
        if (!pcl_isfinite (frame->points[i].x) || 
            !pcl_isfinite (frame->points[i].y) || 
            !pcl_isfinite (frame->points[i].z))
        {
            continue;
        }
        cv::Point3f p;
        p.x = frame->points[i].x;
        p.y = frame->points[i].y;
        p.z = frame->points[i].z;
        points.push_back(p);
    }
    return points;
}

