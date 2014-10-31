#include <motion_detection/flow_neighbour_similarity_calculator.h>
#include <iostream>

FlowNeighbourSimilarityCalculator::FlowNeighbourSimilarityCalculator()
{
}

FlowNeighbourSimilarityCalculator::~FlowNeighbourSimilarityCalculator()
{
}

void FlowNeighbourSimilarityCalculator::calculateNeighbourhoodSimilarity(const cv::Mat &vectors, cv::Mat &similarity, int pixel_step)
{
    for (int i = 0; i < vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < vectors.cols; j = j + pixel_step)
        {           
            if (j != 0)
            {
            //    std::cout << ", ";
            }
            cv::Vec4d vec = vectors.at<cv::Vec4d>(i, j);
            if (abs(vec[2]) < 1.0 && abs(vec[3]) < 1.0)
            {
            //    std::cout << "0.0";
                continue;
            }
            double sim = similarity.at<double>(i,j);
            double similarityMeasure = 0.0;
            int num_neighbours = 0;
            if (i != 0)
            {
                cv::Vec4d top_neighbour = vectors.at<cv::Vec4d>(i - pixel_step, j);
                similarityMeasure += getSimilarityMeasure(vec, top_neighbour);
                num_neighbours++;                
            }
            if (j != 0)
            {
                cv::Vec4d left_neighbour = vectors.at<cv::Vec4d>(i, j - pixel_step);
                similarityMeasure += getSimilarityMeasure(vec, left_neighbour);
                num_neighbours++;
            }
            if (i < vectors.rows - pixel_step)
            {
                cv::Vec4d bottom_neighbour = vectors.at<cv::Vec4d>(i + pixel_step, j);
                similarityMeasure += getSimilarityMeasure(vec, bottom_neighbour);
                num_neighbours++;                
            }
            if (j < vectors.cols - pixel_step)
            {
                cv::Vec4d right_neighbour = vectors.at<cv::Vec4d>(i, j + pixel_step);
                similarityMeasure += getSimilarityMeasure(vec, right_neighbour);
                num_neighbours++;
            }

            similarityMeasure /= num_neighbours;
            sim = similarityMeasure;
           // std::cout << sim;           
        }
       // std::cout << std::endl;
    }
  //  std::cout << std::endl << std::endl << std::endl << std::endl;
}

double FlowNeighbourSimilarityCalculator::getSimilarityMeasure(const cv::Vec4d &one, const cv::Vec4d &two)
{
    double magnitude_diff = abs(getMagnitude(one) - getMagnitude(two));

    double angle_diff = getAngularDistance(one, two);
    
    return (5*angle_diff);
   
}

double FlowNeighbourSimilarityCalculator::getMagnitude(const cv::Vec4d &vector)
{
    return sqrt(pow(vector[2],2) + pow(vector[3],2));
}

double FlowNeighbourSimilarityCalculator::getAngularDistance(const cv::Vec4d &one, const cv::Vec4d &two)
{

    double angle1 = getAngle(one);
    double angle2 = getAngle(two);
    return atan2(sin(angle1 - angle2), cos(angle1 - angle2));    
}

double FlowNeighbourSimilarityCalculator::getAngle(const cv::Vec4d &vector)
{
    double ang = atan2(vector[3], vector[2]);
    if (ang < 0.0)
    {
        ang += 2 * M_PI;
    }
    return ang;
}

