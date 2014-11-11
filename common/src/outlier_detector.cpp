#include <motion_detection/outlier_detector.h>
#include <iostream>

OutlierDetector::OutlierDetector()
{

}

OutlierDetector::~OutlierDetector()
{
}

void OutlierDetector::findOutliers(const cv::Mat &optical_flow_vectors, cv::Mat &outlier_probabilities, bool include_zeros, int pixel_step, bool print)
{
    cv::Mat angle_matrix = cv::Mat::zeros(optical_flow_vectors.rows, optical_flow_vectors.cols, CV_64F);
    cv::Mat magnitude_matrix = cv::Mat::zeros(optical_flow_vectors.rows, optical_flow_vectors.cols, CV_64F);
    outlier_probabilities = cv::Mat::zeros(optical_flow_vectors.rows, optical_flow_vectors.cols, CV_64F);
    
    createAngleMatrix(optical_flow_vectors, angle_matrix, pixel_step);    
    //if (print) std::cout << " Angle matrix " << std::endl << angle_matrix << std::endl;
    createMagnitudeMatrix(optical_flow_vectors, magnitude_matrix, pixel_step);
    //if (print) std::cout << " Magnitude matrix " << std::endl << magnitude_matrix << std::endl;
    if (print ) std::cout << "angles " << std::endl;
    createMask(optical_flow_vectors, angle_matrix, outlier_probabilities, include_zeros, pixel_step, print);
    //if (print) std::cout << " Mask angle " << std::endl << outlier_probabilities << std::endl;
    if (print ) std::cout << "lenghts " << std::endl;
    createMask(optical_flow_vectors, magnitude_matrix, outlier_probabilities, include_zeros, pixel_step, print);
    //if (print) std::cout << " Mask magn " << std::endl << outlier_probabilities << std::endl;
}

void OutlierDetector::createAngleMatrix(const cv::Mat &optical_flow_vectors, cv::Mat &angle_matrix, int pixel_step)
{
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            cv::Vec4d elem = optical_flow_vectors.at<cv::Vec4d>(i, j);
            angle_matrix.at<double>(i, j) = std::atan2(elem[3], elem[2]);
        }
    }
}

void OutlierDetector::createMagnitudeMatrix(const cv::Mat &optical_flow_vectors, cv::Mat &magnitude_matrix, int pixel_step)
{
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            cv::Vec4d elem = optical_flow_vectors.at<cv::Vec4d>(i, j);
            magnitude_matrix.at<double>(i, j) = std::sqrt(elem[3]*elem[3] + elem[2]*elem[2]);
        }
    }
}

double OutlierDetector::getMedian(std::vector<double> vals, bool print)
{
    sort(vals.begin(), vals.end());
    if (print)
    {
        std::cout << "Calculating median for " << std::endl;
        for (int i = 0; i < vals.size(); i++)
        {
            std::cout << vals.at(i) << ", ";
        }
        std::cout << std::endl;
    }
    double median = 0.0;
    if (vals.size() % 2 == 0)
    {
        median = (vals[(vals.size() / 2) - 1] + vals[vals.size() / 2]) / 2.0;        
    }
    else
    {
        median = vals[(vals.size()-1) / 2];
    }
    return median;
}
void OutlierDetector::createMask(const cv::Mat &optical_flow_vectors, const cv::Mat &values, cv::Mat &mask, bool include_zeros, int pixel_step, bool print)
{
    std::vector<double> vals;
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            double elem = values.at<double>(i, j);
            cv::Vec4d e = optical_flow_vectors.at<cv::Vec4d>(i, j);
            if (print) std::cout << e << ", " << elem << std::endl;
            if (include_zeros)
            {
                vals.push_back(elem);
            }
            else if (std::abs(e[2]) > 0.0 || std::abs(e[3]) > 0.0)
            {
                vals.push_back(elem);
            }
        }
    }    
    if (vals.empty())
    {
        return;
    }
    double median = getMedian(vals, print);
    if (print) std::cout << "median: " << median  << std::endl;
    std::vector<double> diff;
    for (int i = 0; i < vals.size(); i++)
    {
        diff.push_back(std::abs(vals.at(i) - median));
       // std::cout << fabs(vals.at(i) - median) << std::endl;
    }    
    double median_absolute_deviation = getMedian(diff, print);
    if (print) std::cout << "MAD: " << median_absolute_deviation  << std::endl;
    double threshold = 3.5;
    int index = 0;

    if (print) std::cout << "diff: " << std::endl;
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            double val = values.at<double>(i, j);
            cv::Vec4d e = optical_flow_vectors.at<cv::Vec4d>(i, j);
            if (include_zeros)
            {
                double modified_z_score = 0.6745 * diff.at(index) / median_absolute_deviation;
                if (modified_z_score > threshold)
                {
                    mask.at<double>(i, j) = 1.0;
                }
                index++;
            }
            else if (std::abs(e[2]) > 0.0 || std::abs(e[3]) > 0.0)
            {
                double modified_z_score = 0.6745 * diff.at(index) / median_absolute_deviation;
                if (print) std::cout << modified_z_score << std::endl;
                if (modified_z_score > threshold)
                {
                    mask.at<double>(i, j) = 1.0;
                    if (print) std::cout << i << ", " << j << ", " << mask.at<double>(i, j) << std::endl;
                }
                index++;
            }
        }
    }
}
