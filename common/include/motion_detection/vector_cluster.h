#ifndef VECTOR_CLUSTER_H_
#define VECTOR_CLUSTER_H_

#include <opencv2/core/core.hpp>

class VectorCluster
{
    public:
        VectorCluster();
        virtual ~VectorCluster();

        void addVector(const cv::Vec4d &vector);
        double getClosestDistance(const cv::Vec4d &vector);
        double getClosestOrientation(const cv::Vec4d &vector);
        cv::Point2f getCentroid();
        double getMeanOrientation();
        
        std::vector<cv::Vec4d> getCluster();
        std::vector<cv::Point2f> getClusterPoints();
        int size();


    private:
        double getDistance(const cv::Vec4d &one, const cv::Vec4d &two);
        double getAngularDistance(const cv::Vec4d &one, const cv::Vec4d &two);
        double getAngle(const cv::Vec4d &vector);


    private:
        std::vector<cv::Vec4d> cluster_;
};
#endif
