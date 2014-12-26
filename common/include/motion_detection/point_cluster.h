#ifndef POINT_CLUSTER_H_
#define POINT_CLUSTER_H_

#include <opencv2/core/core.hpp>

class PointCluster
{
    public:
        PointCluster();
        virtual ~PointCluster();

        void addPoint(const cv::Point2f &point);
        double getClosestDistance(const cv::Point2f &point);
        cv::Point2f getCentroid();
        
        std::vector<cv::Point2f> getCluster();
        int size();

    private:
        double getDistance(const cv::Point2f &one, const cv::Point2f &two);
        void getMinMax(double &min_x, double &max_x, double &min_y, double &max_y);

    private:
        std::vector<cv::Point2f> cluster_;
};
#endif
