#ifndef OPTICAL_FLOW_CALCULATOR_H_
#define OPTICAL_FLOW_CALCULATOR_H_

#include <opencv2/core/core.hpp>

class OpticalFlowCalculator
{
    public:
        OpticalFlowCalculator();
        virtual ~OpticalFlowCalculator();
        
        std::vector<std::vector<double> > calculateOpticalFlow(cv::Mat image1, cv::Mat image2, cv::Mat &optical_flow_image);   
};

#endif
