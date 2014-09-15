#ifndef FLOW_CLUSTERER_H_
#define FLOW_CLUSTERER_H_

#include <opencv2/core/core.hpp>

class FlowClusterer
{

    public:
        FlowClusterer();
        virtual ~FlowClusterer();

        cv::Mat clusterFlowVectors(const cv::Mat &flow_vectors);
};
#endif
