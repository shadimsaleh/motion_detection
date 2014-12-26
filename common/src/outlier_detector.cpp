#include <motion_detection/outlier_detector.h>
#include <iostream>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>

OutlierDetector::OutlierDetector()
{
    srand (time(NULL));
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

void OutlierDetector::getOutlierVectors(const cv::Mat &optical_flow_vectors, const cv::Mat &outlier_probabilities, cv::Mat &outlier_vectors, int pixel_step)
{

    outlier_vectors = cv::Mat::zeros(optical_flow_vectors.rows, optical_flow_vectors.cols, CV_32FC4);
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            if (outlier_probabilities.at<double>(i, j) > 0.5)
            {
                cv::Vec4d orig_vec = optical_flow_vectors.at<cv::Vec4d>(i, j);
                cv::Vec4d &out_vec = outlier_vectors.at<cv::Vec4d>(i, j);
                for (int i = 0; i < 4; i++) out_vec[i] = orig_vec[i];                    
            }
        }
    }
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
                if (std::abs(modified_z_score) > threshold)
                {
                    mask.at<double>(i, j) = 1.0;
                }
                index++;
            }
            else if (std::abs(e[2]) > 0.0 || std::abs(e[3]) > 0.0)
            {
                double modified_z_score = 0.6745 * diff.at(index) / median_absolute_deviation;
                if (print) std::cout << modified_z_score << std::endl;
                if (std::abs(modified_z_score) > threshold)
                {
                    mask.at<double>(i, j) = 1.0;
                    if (print) std::cout << i << ", " << j << ", " << mask.at<double>(i, j) << std::endl;
                }
                index++;
            }
        }
    }
}

void fillMatrix(const std::vector<std::vector<cv::Point2f> > &trajectories, Eigen::MatrixXf &data)
{
    // TODO: find a better way to do this
    for (int i = 0; i < trajectories.size(); i++)
    {
        std::vector<cv::Point2f> trajectory = trajectories.at(i);
        for (int j = 0; j < trajectory.size(); j++)
        {
            data(j * 2, i) = trajectory.at(j).x;     
            data((j * 2) + 1, i) = trajectory.at(j).y;     
        }
    }
}

void meanSubtract(Eigen::MatrixXf &data)
{
    double x_sum = data.row(0).sum();
    double y_sum = data.row(1).sum();
    x_sum /= data.cols();
    y_sum /= data.cols();
    Eigen::MatrixXf xsum = Eigen::MatrixXf::Constant(1, data.cols(), x_sum); 
    Eigen::MatrixXf ysum = Eigen::MatrixXf::Constant(1, data.cols(), y_sum); 
    for (int i = 0; i < data.rows(); i++)
    {
        if (i % 2 == 0)
        {
            data.row(i) = data.row(i) - xsum;
        }
        else
        {
            data.row(i) = ysum - data.row(i);
        }
    }
}

std::vector<int> fillSubset(const Eigen::MatrixXf &data, Eigen::MatrixXf &subset, int num_columns)
{
    std::vector<int> column_indices;
    for(int i = 0; i < num_columns; i++)
    {
        int column_number = rand() % data.cols();
        column_indices.push_back(column_number);
        subset.col(i) = data.col(column_number);        
        //subset.col(i) = data.col(100 + i);        
    }
    return column_indices;
}

std::vector<std::vector<cv::Point2f> > OutlierDetector::fitSubspace(const std::vector<std::vector<cv::Point2f> > &trajectories, std::vector<cv::Point2f> &outlier_points, int num_motions, double residual_threshold)
{
    bool print = false;
    int subspace_dimensions = trajectories[0].size() * 2; // n
    int num_trajectories = trajectories.size();
    // each column in data represents one trajectory
    // even rows are x coordinates, odd rows are y coordinates
    Eigen::MatrixXf data(subspace_dimensions, num_trajectories);
    if (print) std::cout << "fill matrix " << std::endl;
    fillMatrix(trajectories, data);
    if (print) std::cout << "mean subtract " << std::endl;
    meanSubtract(data);

    int num_sample_points = 4 * num_motions; // d
    int num_iterations = 50;
    double sigma = 0.5;
    
    Eigen::VectorXf final_residual;
    std::vector<int> final_columns;
    int max_points = 0;

    if (print) std::cout << " start iterations " << std::endl;
    for (int i = 0; i < num_iterations; i++)
    {
        Eigen::MatrixXf subset(subspace_dimensions, num_sample_points); 
        if (print) std::cout << "file subset " << std::endl;
        std::vector<int> column_indices;
        column_indices = fillSubset(data, subset, num_sample_points);

        if (print) std::cout << "svd calculation " << std::endl;
//        Eigen::JacobiSVD<Eigen::MatrixXf> svd(subset, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Eigen::MatrixXf, Eigen::FullPivHouseholderQRPreconditioner> svd(subset,  Eigen::ComputeFullU | Eigen::ComputeFullV);

        if (print) std::cout << "init Pnd " << std::endl;
        Eigen::MatrixXf Pnd = Eigen::MatrixXf::Zero(subspace_dimensions, subspace_dimensions);
        //std::cout << "U" << std::endl;
        //std::cout << svd.matrixU() << std::endl;
        //std::cout << svd.singularValues() << std::endl;
        
        if (print) std::cout << "calc M " << std::endl;
        for (int idx = 0; idx < num_sample_points; idx++)
        {
            Eigen::VectorXf u = svd.matrixU().col(idx);
            Eigen::MatrixXf M = u*u.transpose();
            Pnd = Pnd + M;
        }
        if (print) std::cout << "calc Pnd " << std::endl;
        Pnd = Eigen::MatrixXf::Identity(subspace_dimensions, subspace_dimensions) - Pnd;

        Eigen::MatrixXf data_T = data.transpose();
        if (print) std::cout << "calc residual " << std::endl;
        Eigen::VectorXf residual = (data_T * (Pnd * data)).diagonal();
        if (print) std::cout << "residual : " << std::endl;
        if (print) std::cout << residual << std::endl;
        residual = residual.cwiseAbs();
        int num_points = 0;
        for (int idx = 0; idx < residual.size(); idx++)
        {
            if (print) std::cout << "threshold " << (subspace_dimensions - num_sample_points) * sigma * sigma << std::endl;
            if (residual(idx) < (subspace_dimensions - num_sample_points) * sigma * sigma)
            {
                if (print) std::cout << "adding  " << std::endl;
                num_points++;
            }
        }
        if (num_points > max_points)
        {
            if (print) std::cout << num_points << " to " << max_points << std::endl;
            if (print) std::cout << "copy residual" << residual << std::endl;
            max_points = num_points;
            final_residual = residual;
            final_columns = column_indices;
            if (print) std::cout << "copy final residual " << final_residual << std::endl;
        }
    }
    if (print) std::cout << "final residual " << std::endl;
    if (print) std::cout << final_residual << std::endl;

    for (int idx = 0; idx < final_residual.size(); idx++)
    {
        if (final_residual(idx) > residual_threshold)
        {
            outlier_points.push_back(trajectories.at(idx).at(trajectories.at(idx).size() - 2));
        }
    }
    std::vector<std::vector<cv::Point2f> > trajectory_subspace_vectors;
    for (int i = 0; i < final_columns.size(); i++)
    {
        trajectory_subspace_vectors.push_back(trajectories.at(final_columns.at(i)));
    }
    return trajectory_subspace_vectors;
}
