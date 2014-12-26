#include <motion_detection/trajectory_visualizer.h>

TrajectoryVisualizer::TrajectoryVisualizer() : rng(12345)
{

}

TrajectoryVisualizer::~TrajectoryVisualizer()
{

}

void TrajectoryVisualizer::showTrajectories(const cv::Mat &original_image, cv::Mat &output_image, const std::vector<std::vector<cv::Point2f> > &trajectories)
{
    original_image.copyTo(output_image);
    std::vector<cv::Scalar> colours;    
    for (int i = 0; i < trajectories.size(); i++)
    {
        colours.push_back(cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)));       
    }
    showTrajectories(output_image, trajectories, colours);
}

void TrajectoryVisualizer::showTrajectories(cv::Mat &output_image, const std::vector<std::vector<cv::Point2f> > &trajectories, const std::vector<cv::Scalar> &colours)
{
    for (int i = 0; i < trajectories.size(); i++)
    {
        std::vector<cv::Point2f> points = trajectories.at(i);
        for (int j = 1; j < points.size(); j++)
        {
            cv::line(output_image, points.at(j), points.at(j - 1), colours.at(i), 2, 8);
        }
    }
}

