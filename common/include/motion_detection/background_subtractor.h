/* background_subtractor.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef BACKGROUND_SUBTRACTOR_H_
#define BACKGROUND_SUBTRACTOR_H_

#include <opencv2/opencv.hpp>

class BackgroundSubtractor
{
    public:
        BackgroundSubtractor();
        virtual ~BackgroundSubtractor();

        void getMotionContours(const cv::Mat &frame, cv::Mat &motion_contours);

    private:
        cv::BackgroundSubtractorMOG2 background_subtractor;
        cv::Mat background;
};

#endif
