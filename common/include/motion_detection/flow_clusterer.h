#ifndef FLOW_CLUSTERER_H_
#define FLOW_CLUSTERER_H_

#include <opencv2/core/core.hpp>

class FlowClusterer
{

    public:
        FlowClusterer();
        virtual ~FlowClusterer();

        cv::Mat clusterFlowVectors(const cv::Mat &flow_vectors);

        std::vector<cv::Point2f> getClustersCenters(const cv::Mat &flow_vectors, int pixel_step, double distance_threshold, double angular_threshold);

        std::vector<cv::Mat> getClusters(const cv::Mat &flow_vectors, int pixel_step, double distance_threshold, double angular_threshold);
};
#endif
