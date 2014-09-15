#ifndef FLOW_DIFFERENCE_CALCULATOR_H_
#define FLOW_DIFFERENCE_CALCULATOR_H_

#include <opencv2/core/core.hpp>

class FlowDifferenceCalculator
{
    public:
        FlowDifferenceCalculator();
        virtual ~FlowDifferenceCalculator();

        void calculateFlowDifference(const cv::Mat &first, const cv::Mat &second, cv::Mat &diff);
};
#endif
