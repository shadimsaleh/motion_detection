#ifndef OPTICAL_FLOW_CALCULATOR_H_
#define OPTICAL_FLOW_CALCULATOR_H_

#include <opencv2/core/core.hpp>

class OpticalFlowCalculator
{
    public:
        OpticalFlowCalculator();
        virtual ~OpticalFlowCalculator();
        
        int calculateOpticalFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors, int pixel_step, cv::Mat &comp, double min_vector_size);

        int calculateOpticalFlowTrajectory(const std::vector<cv::Mat> &images, cv::Mat &optical_flow_vectors, std::vector<std::vector<cv::Point2f> > &trajectories, int pixel_step, cv::Mat &comp, double min_vector_size);

        int calculateCompensatedFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors, int pixel_step);

        int superPixelFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_image, cv::Mat &optical_flow_vectors);

        void varFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow, cv::Mat &optical_flow_vectors);

        void drawMotionField(IplImage* imgU, IplImage* imgV, IplImage* imgMotion, int xSpace, int ySpace, float cutoff, int multiplier, CvScalar color);

        void writeFlow(const cv::Mat &flow_vectors, const std::string &filename, int pixel_step);
        void writeTrajectories(const std::vector<std::vector<cv::Point2f> > &trajectories, const std::string &filename);
};

#endif
