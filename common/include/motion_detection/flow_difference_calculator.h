/* flow_difference_calculator.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#ifndef FLOW_DIFFERENCE_CALCULATOR_H_
#define FLOW_DIFFERENCE_CALCULATOR_H_

#include <opencv2/core/core.hpp>

class FlowDifferenceCalculator
{
    public:
        FlowDifferenceCalculator();
        virtual ~FlowDifferenceCalculator();

        void calculateFlowDifference(const cv::Mat &first, const cv::Mat &second, cv::Mat &diff, int pixel_step);
};
#endif
