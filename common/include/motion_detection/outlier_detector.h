/* outlier_detector.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */
#ifndef OUTLIER_DETECTOR_H_
#define OUTLIER_DETECTOR_H_

#include <opencv2/core/core.hpp>

class OutlierDetector
{
    public:
        OutlierDetector();
        virtual ~OutlierDetector();

        void findOutliers(const cv::Mat &optical_flow_vectors, cv::Mat &outlier_probabilities, bool include_zeros, int pixel_step, bool print);
        void getOutlierVectors(const cv::Mat &optical_flow_vectors, const cv::Mat &outlier_probabilities, cv::Mat &outlier_vectors, int pixel_step);
        std::vector<std::vector<cv::Point2f> > fitSubspace(const std::vector<std::vector<cv::Point2f> > &trajectories, std::vector<cv::Point2f> &outlier_points, int num_motions, double sigma);

    private:
        void createMask(const cv::Mat &optical_flow_vectors, const cv::Mat &values, cv::Mat &mask, bool include_zeros, int pixel_step, bool print);
        void createAngleMatrix(const cv::Mat &optical_flow_vectors, cv::Mat &angle_matrix, int pixel_step);
        void createMagnitudeMatrix(const cv::Mat &optical_flow_vectors, cv::Mat &magnitude_matrix, int pixel_step);
        double getMedian(std::vector<double> vals, bool print);

    private:
        std::vector<std::vector<double> > chi_square_table;
        
};
#endif
