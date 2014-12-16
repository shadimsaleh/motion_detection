#ifndef OPTICAL_FLOW_VISUALIZER_H_
#define OPTICAL_FLOW_VISUALIZER_H_

#include <opencv2/core/core.hpp>

class OpticalFlowVisualizer
{
    public:
        OpticalFlowVisualizer();
        virtual ~OpticalFlowVisualizer();

        void showOpticalFlowVectors(const cv::Mat &original, cv::Mat &optical_flow_image, const cv::Mat &optical_flow_vectors, int pixel_step, cv::Scalar colour, double min_vector_size);        
        void showFlowClusters(const cv::Mat &original, cv::Mat &optical_flow_image, const std::vector<cv::Vec4d> &optical_flow_vectors, int pixel_step, cv::Scalar colour, double min_vector_size);        
        void showFlowOutliers(const cv::Mat &original, cv::Mat &outlier_image, const cv::Mat &outlier_mask, const cv::Mat &optical_flow_vectors, int pixel_step, bool print);

};
#endif
