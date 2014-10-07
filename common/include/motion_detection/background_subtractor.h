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
