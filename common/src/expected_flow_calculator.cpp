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
                                                                          cv::Mat &frame_image,
                                                                          cv::Mat &projected_image2,
                                                                          cv::Mat &expected_flow_vectors)
{
//   Eigen::Transform<Scalar, 3, Eigen::Affine> t;
//   pcl::getTransformation(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, t);

    pcl::PointCloud<PointT>::Ptr frame1(new pcl::PointCloud<PointT>);
 //   pcl::PointCloud<PointT>::Ptr frame2(new pcl::PointCloud<PointT>);

//    std::cout << "converting to pointcloud" << std::endl;
    pcl::fromPCLPointCloud2(frame, *frame1);
//    std::cout << "done converting to pointcloud" << std::endl;

  //  pcl::copyPointCloud(*frame1, *frame2);

    /*
    for (int i = 0; i < frame1->points.size(); i++)
    {
        std::cout << frame1->points[i].x << ", " << frame1->points[i].y << ", " << frame1->points[i].z << std::endl;
    }
    */

//    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//    transform_2.translation() << 0.0, 0.0, 0.0;
//    std::cout << "transforming pointcloud" << std::endl;
//    pcl::transformPointCloud (*frame1, *frame2, transform_2);
//    std::cout << "done transforming pointcloud" << std::endl;
    
//    std::cout << "frame 1 pc: " << frame1->points.size() << std::endl;
    cv::Mat projected_image1;
    std::vector<cv::Point3f> objectPoints = getOpenCVPoints(frame1);
    //cv::Mat cv_frame1 = getOpenCVPoints(frame1);
 //   std::cout << "frame 1 size: " << objectPoints.size() << std::endl;
//    std::vector<cv::Point2f> projected_points1(cv_frame1.cols * cv_frame1.rows);
    std::vector<cv::Point2f> projected_points1(objectPoints.size());
    cv::projectPoints(objectPoints, camera_rotation, camera_translation, camera_matrix, camera_distortion, projected_points1);

    /*
    for (int i = 0; i < 1000; i++)
    {
        std::cout << projected_points1[i].x << ", " << projected_points1[i].y << std::endl;
    }

    */
    projected_image1 = cv::Mat(projected_points1);

//    cv::Mat cv_frame2 = getOpenCVPoints(frame2);
//    std::cout << "frame 2 size: " << cv_frame2.cols << ", " << cv_frame2.rows << std::endl;
    std::vector<cv::Point2f> projected_points2(objectPoints.size());
    double translation[3] = {odom[0], odom[1], odom[2]};
    cv::Mat camera_translation2 = cv::Mat(1, 3, CV_64F, translation);
    double rotation[3] = {odom[3], odom[4], odom[5]};
    cv::Mat camera_rotation2 = cv::Mat(1, 3, CV_64F, rotation);
    cv::projectPoints(objectPoints, camera_rotation2, camera_translation2, camera_matrix, camera_distortion, projected_points2);
    projected_image2 = cv::Mat(projected_points2);


    /*
    std::cout << "Camera Matrix: " << std::endl;
    for (int i = 0; i < camera_matrix.cols; i++)
    {
        for (int j = 0; j < camera_matrix.rows; j++)
        {
            std::cout << camera_matrix.at<double>(i,j) << ", ";
        }
        std::cout << std::endl;
    }
*/

    for (int i = 0; i < projected_points1.size(); i++)
    {
        //int col = (i % frame_image.cols);
        //int row = (i / frame_image.cols);       

        cv::Point2f start_point(projected_points1[i]);
        cv::Point2f end_point(projected_points2[i]);        

        if ((int)start_point.x % 20 != 0 || (int)start_point.y % 20 != 0)
            continue;
        cv::Vec4d &elem = expected_flow_vectors.at<cv::Vec4d> ((int)start_point.x, (int)start_point.y);
        float x_diff = end_point.x - start_point.x;
        float y_diff = end_point.y - start_point.y;
        elem[0] = start_point.x;
        elem[1] = start_point.y;
        elem[2] = atan2(y_diff, x_diff);
        elem[3] = sqrt(x_diff*x_diff + y_diff*y_diff);
        if (i < 1000)
        {
            //std::cout << "start point: " << start_point.x << ", " << start_point.y << "    :    end point: " << end_point.x << ", " << end_point.y << std::endl;
       //     std::cout << "start point: " << projected_points1[i].x << ", " << projected_points1[i].y << "    :    end point: " << projected_points2[i].x << ", " << projected_points2[i].y << std::endl;
        }

        cv::line(frame_image, start_point, end_point, CV_RGB(255,0,0), 1, CV_AA, 0);        
    }






/*   pcl::io::PointCloudImageExtractorFromRGBField<PointT> pcie;
   pcie.setPaintNaNsWithBlack(true);
   pcl::PCLImage image;
   pcl::PCLImage image2;
   bool extracted = pcie.extract(frame, image);
   extracted = pcie.extract(frame2, image2);
   */
    
    /*
    pcl::PointCloud<PointT> projected_frame2;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;


    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (frame2);
    proj.setModelCoefficients (coefficients);
    proj.setCopyAllData(true);
    proj.filter (projected_frame2);
    
    std::cout << "frame origin " << projected_frame2.sensor_origin_[0] << std::endl;
    std::cout << "frame origin " << projected_frame2.sensor_origin_[1] << std::endl;
    std::cout << "frame origin " << projected_frame2.sensor_origin_[2] << std::endl;
    std::cout << "frame origin " << projected_frame2.sensor_origin_[3] << std::endl;   
   // projected_frame2.sensor_origin_[2] = 5.0;

    std::cout << "converting to pointcloud2" << std::endl;
    PointCloud tframe;
    pcl::toPCLPointCloud2(projected_frame2, tframe);
    std::cout << "done converting to pointcloud2" << std::endl;

    */


    /*
    pcl::PointCloud<pcl::RGB> image;
    pcl::PointCloud<pcl::RGB> image2;

    std::cout << "converting to image" << std::endl;
    fromPCLPointCloud2 (frame, image);
    fromPCLPointCloud2 (tframe, image2);
    std::cout << "done converting to image" << std::endl;

    
    std::cout << "saving png file" << std::endl;
    pcl::io::savePNGFile("one.png", image);
    pcl::io::savePNGFile("two.png", image2);
    std::cout << "done saving png file" << std::endl;
    pcl::io::savePCDFileASCII("one.pcd", frame1);
    pcl::io::savePCDFileASCII("two.pcd", projected_frame2);
    */
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
            //std::cout << pcl_isfinite (frame->points[i].x) << ", " << pcl_isfinite (frame->points[i].y) << ", " << pcl_isfinite (frame->points[i].z) << std::endl;
            continue;
        }
       // m++;
        cv::Point3f p;
        p.x = frame->points[i].x;
        p.y = frame->points[i].y;
        p.z = frame->points[i].z;
        //if (m < 100)
        //    std::cout << p.x << ", " << p.y << ", " << p.z << std::endl;
        points.push_back(p);
    }
//    std::cout << "points size: " << points.size() << std::endl;
    return points;
    //cv::Mat m_points = cv::Mat(points);
    //return m_points;
}

