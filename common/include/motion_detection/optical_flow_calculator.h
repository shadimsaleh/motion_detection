#ifndef OPTICAL_FLOW_CALCULATOR_H_
#define OPTICAL_FLOW_CALCULATOR_H_

#include <opencv2/core/core.hpp>

class OpticalFlowCalculator
{
    public:
        OpticalFlowCalculator();
        virtual ~OpticalFlowCalculator();
        
        int calculateOpticalFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors, int pixel_step);

        int superPixelFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_image, cv::Mat &optical_flow_vectors);

        void varFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow, cv::Mat &optical_flow_vectors);

        void drawMotionField(IplImage* imgU, IplImage* imgV, IplImage* imgMotion, int xSpace, int ySpace, float cutoff, int multiplier, CvScalar color);
};

#endif
