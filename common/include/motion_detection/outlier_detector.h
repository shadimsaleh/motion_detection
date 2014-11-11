#ifndef OUTLIER_DETECTOR_H
#define OUTLIER_DETECTOR_H

#include <opencv2/core/core.hpp>

class OutlierDetector
{
    public:
        OutlierDetector();
        virtual ~OutlierDetector();

        void findOutliers(const cv::Mat &optical_flow_vectors, cv::Mat &outlier_probabilities, bool include_zeros, int pixel_step, bool print);

    private:
        void createMask(const cv::Mat &optical_flow_vectors, const cv::Mat &values, cv::Mat &mask, bool include_zeros, int pixel_step, bool print);
        void createAngleMatrix(const cv::Mat &optical_flow_vectors, cv::Mat &angle_matrix, int pixel_step);
        void createMagnitudeMatrix(const cv::Mat &optical_flow_vectors, cv::Mat &magnitude_matrix, int pixel_step);
        double getMedian(std::vector<double> vals, bool print);
        
};
#endif
