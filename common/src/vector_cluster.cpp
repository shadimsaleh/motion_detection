#include <motion_detection/vector_cluster.h>

VectorCluster::VectorCluster()
{
}

VectorCluster::~VectorCluster()
{
}

void VectorCluster::addVector(const cv::Vec4d & vector)
{
    cluster_.push_back(vector);
}

double VectorCluster::getClosestDistance(const cv::Vec4d &vector)
{
    double min_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < cluster_.size(); i++)
    {
        double distance = getDistance(vector, cluster_.at(i));
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    return min_distance;
}
double VectorCluster::getClosestOrientation(const cv::Vec4d &vector)
{
    double min_angular_distance = std::numeric_limits<double>::max();
    for (int i = 0; i < cluster_.size(); i++)
    {
        double angular_distance = abs(getAngularDistance(vector, cluster_.at(i)));
        if (angular_distance < min_angular_distance)
        {
            min_angular_distance = angular_distance;
        }
    }
    return min_angular_distance;
}

cv::Point2f VectorCluster::getCentroid()
{
    double x_sum = 0.0;
    double y_sum = 0.0;
    for (int i = 0; i < cluster_.size(); i++)
    {
        x_sum += cluster_.at(i)[0];
        y_sum += cluster_.at(i)[1];
    }
    x_sum /= cluster_.size();
    y_sum /= cluster_.size();
    cv::Point2f centroid(x_sum, y_sum);
    return centroid;
}

double VectorCluster::getMeanOrientation()
{
    double angle_sum = 0.0;
    for (int i = 0; i < cluster_.size(); i++)
    {
        angle_sum += getAngle(cluster_.at(i));
    }
    angle_sum /= cluster_.size();
    return angle_sum;
}

std::vector<cv::Vec4d> VectorCluster::getCluster()
{
    return cluster_;
}

std::vector<cv::Point2f> VectorCluster::getClusterPoints()
{
    std::vector<cv::Point2f> cluster_points;
    for (int i = 0; i < cluster_.size(); i++)
    {
        cv::Point2f p(cluster_.at(i)[0], cluster_.at(i)[1]);
        cluster_points.push_back(p);
    }
    return cluster_points;
}

int VectorCluster::size()
{
    return cluster_.size();
}
double VectorCluster::getDistance(const cv::Vec4d &one, const cv::Vec4d &two)
{
    return sqrt((one[0] - two[0])*(one[0] - two[0]) + (one[1] - two[1]) * (one[1] - two[1]));
}

double VectorCluster::getAngularDistance(const cv::Vec4d &one, const cv::Vec4d &two)
{

    double angle1 = getAngle(one);
    double angle2 = getAngle(two);
    return atan2(sin(angle1 - angle2), cos(angle1 - angle2));    
}

double VectorCluster::getAngle(const cv::Vec4d &vector)
{
    double ang = atan2(vector[3], vector[2]);
    if (ang < 0.0)
    {
        ang += 2 * M_PI;
    }
    return ang;
}
