/* background_subtractor.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <motion_detection/background_subtractor.h>

BackgroundSubtractor::BackgroundSubtractor()
{
}

BackgroundSubtractor::~BackgroundSubtractor()
{

}

void BackgroundSubtractor::getMotionContours(const cv::Mat &frame, cv::Mat &motion_contours)
{

    frame.copyTo(motion_contours);
    cv::Mat foreground_mask;
    std::vector<std::vector<cv::Point> > contours;

    background_subtractor.operator ()(frame, foreground_mask);
    background_subtractor.getBackgroundImage(background);


    cv::erode(foreground_mask, foreground_mask, cv::Mat());
    cv::dilate(foreground_mask, foreground_mask, cv::Mat());


    cv::findContours(foreground_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::drawContours(motion_contours, contours, -1, cv::Scalar(0,0,255), 2); 
}
