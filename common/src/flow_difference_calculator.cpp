#include <motion_detection/flow_difference_calculator.h>
#include <iostream>

FlowDifferenceCalculator::FlowDifferenceCalculator()
{
}

FlowDifferenceCalculator::~FlowDifferenceCalculator()
{
}

void FlowDifferenceCalculator::calculateFlowDifference(const cv::Mat &first, const cv::Mat &second, cv::Mat &diff, int pixel_step)
{
    for (int i = 0; i < first.rows; i = i + pixel_step)
    {
        for (int j = 0; j < first.cols; j = j + pixel_step)
        {
            cv::Vec4d &elem = diff.at<cv::Vec4d>(i,j);
            elem[0] = first.at<cv::Vec4d>(i,j)[0];
            elem[1] = first.at<cv::Vec4d>(i,j)[1];
            elem[2] = first.at<cv::Vec4d>(i,j)[2] - second.at<cv::Vec4d>(i,j)[2];
            elem[3] = first.at<cv::Vec4d>(i,j)[3] - second.at<cv::Vec4d>(i,j)[3];
        }
    }    
}
