#include <motion_detection/optical_flow_calculator.h>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

OpticalFlowCalculator::OpticalFlowCalculator()
{

}

OpticalFlowCalculator::~OpticalFlowCalculator()
{

}

std::vector<std::vector<double> > OpticalFlowCalculator::calculateOpticalFlow(cv::Mat image1, cv::Mat image2, cv::Mat &optical_flow_image)
{
    const int MAX_LEVEL = 2;
    cv::Size winSize(40, 40);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);

    image1.copyTo(optical_flow_image);
    
    cv::Mat gray_image1;
    cv::Mat gray_image2;

    cvtColor(image1, gray_image1, CV_BGR2GRAY);
    cvtColor(image2, gray_image2, CV_BGR2GRAY);

    std::vector<cv::Point2f> points_image1;
    std::vector<cv::Point2f> points_image2;

    for (int i = 0; i < image1.cols; i = i + 40)
    {
        for (int j = 0; j < image1.rows; j = j + 40)
        {
            cv::Point2f point(i, j);
            //std::cout << "adding point " << i << ", " << j << std::endl;
            points_image1.push_back(point);
        }
    }

    cv::calcOpticalFlowPyrLK(gray_image1, gray_image2, points_image1, points_image2, status, err, winSize, MAX_LEVEL, termcrit, 0, 0.001);


    for (int i = 0; i < points_image2.size(); i++)
    {
        if (status[i])
        {
            cv::Point2f start_point = points_image1.at(i);
            cv::Point2f end_point = points_image2.at(i);
            //std::cout << "Point " << start_point.x << ", " << start_point.y << " moved to " << end_point.x << ", " << end_point.y << std::endl;
            cv::line(optical_flow_image, start_point, end_point, CV_RGB(255,0,0), 1, CV_AA, 0);
        }
    }

    std::vector<std::vector<double> > toreturn;
    return toreturn;
}
