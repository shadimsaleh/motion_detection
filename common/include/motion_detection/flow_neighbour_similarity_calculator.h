/* flow_neighbour_similarity_calculator.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef FLOW_NEIGHBOUR_SIMILARITY_CALCULATOR_H
#define FLOW_NEIGHBOUR_SIMILARITY_CALCULATOR_H

#include <opencv2/core/core.hpp>

class FlowNeighbourSimilarityCalculator
{
    public:
        FlowNeighbourSimilarityCalculator();
        virtual ~FlowNeighbourSimilarityCalculator();

        void calculateNeighbourhoodSimilarity(const cv::Mat &vectors, cv::Mat &similarity, int pixel_step);

        double getSimilarityMeasure(const cv::Vec4d &one, const cv::Vec4d &two);

        double getAngularDistance(const cv::Vec4d &one, const cv::Vec4d &two);

        double getAngle(const cv::Vec4d &vector);

        double getMagnitude(const cv::Vec4d &vector);


};
#endif
