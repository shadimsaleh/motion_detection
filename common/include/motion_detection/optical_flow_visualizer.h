#ifndef OPTICAL_FLOW_VISUALIZER_H_
#define OPTICAL_FLOW_VISUALIZER_H_

#include <opencv2/core/core.hpp>

class OpticalFlowVisualizer
{
    public:
        OpticalFlowVisualizer();
        virtual ~OpticalFlowVisualizer();

        void showOpticalFlowVectors(const cv::Mat &original, cv::Mat &optical_flow_image, const cv::Mat &optical_flow_vectors, int pixel_step, cv::Scalar colour);        

};
#endif
