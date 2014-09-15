#include <motion_detection/flow_clusterer.h>
#include <opencv2/flann/flann_base.hpp>
#include <opencv2/features2d/features2d.hpp>

FlowClusterer::FlowClusterer()
{
}

FlowClusterer::~FlowClusterer()
{
}

cv::Mat FlowClusterer::clusterFlowVectors(const cv::Mat &flow_vectors)
{
    //std::cout << "started" << std::endl;

//    std::cout << "hmm 1" << flow_vectors.rows << ", " << flow_vectors.cols << std::endl;
    cv::Mat samples = cv::Mat::zeros(flow_vectors.rows * flow_vectors.cols, 2, CV_32F);    
  //  std::cout << "hmm " << std::endl;
    int num_samples = 0;
    for (int i = 0; i < flow_vectors.rows; i = i + 20)
    {
        for (int j = 0; j < flow_vectors.cols; j = j + 20)
        {
            //std::cout << "yep " << std::endl;
           // std::cout << flow_vectors.at<cv::Vec4d>(i,j)[3] << std::endl;
            if(flow_vectors.at<cv::Vec4d>(i,j)[3] > 0.0)
            {
               // std::cout << "reached here (" << i <<", " << j << "), "<< num_samples << ": ";          
                for (int f = 0; f < 2; f++)
                {
                    samples.at<float>(num_samples, f) = flow_vectors.at<cv::Vec4d>(i,j)[f];
                    //std::cout << samples.at<float>(num_samples, f) << ", ";
                }
                //std::cout << std::endl;
                num_samples++;
            }
        }
    }
    cvflann::KMeansIndexParams kmeans_params(3000, 100, cvflann::FLANN_CENTERS_KMEANSPP);
 //   std::cout << "done collecting samples" << std::endl;

    samples = samples.rowRange(cv::Range(0,num_samples));
    cvflann::Matrix<float> samplesMatrix((float*)samples.data, samples.rows, samples.cols);
//    std::cout << "done setting samples matrix" << std::endl;
    cv::Mat centers(10, 2, CV_32F);
    cvflann::Matrix<float> centersMatrix((float*)centers.data, centers.rows, centers.cols);


   // std::cout << "started clustering " << std::endl;
    int num_clusters = cvflann::hierarchicalClustering<cvflann::L2<float> >(samplesMatrix, centersMatrix, kmeans_params);
   // std::cout << "finished clustering " << std::endl;

    centers = centers.rowRange(cv::Range(0, num_clusters));
    std::cout << "centers: " << std::endl;
    for(int i = 0; i < num_clusters; i++)
    {
        for (int j = 0; j < centersMatrix.cols; j++)
        {
            std::cout << centersMatrix[i][j] << ", ";
        }
        std::cout << std::endl;
    }
    std::cout <<"num_clusters: " << num_clusters << std::endl;
    return centers;
}
