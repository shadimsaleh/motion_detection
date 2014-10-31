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
