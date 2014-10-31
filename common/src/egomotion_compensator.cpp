#include <motion_detection/egomotion_compensator.h>

EgomotionCompensator::EgomotionCompensator()
{
}

EgomotionCompensator::~EgomotionCompensator()
{
}

void EgomotionCompensator::calculateCompensatedVectors(const cv::Mat &optical_flow_vectors, const std::vector<double> &odom, cv::Mat &compensated_vectors, int pixel_step)
{
    double foe_x, foe_y;
    getFOE(odom, foe_x, foe_y);
    
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
                cv::Vec4d elem = optical_flow_vectors.at<cv::Vec4d>(i, j);
                if (elem[0] != -1.0)
                {
                    double expected = getExpectedOrientation(elem[0], elem[1], foe_x, foe_y);
                    double seen = atan2(elem[3], elem[2]);

                    std::cout << "expected: " << expected << " seen: " << seen << std::endl;

                    if (abs(expected - seen) > 0.1)
                    {
                        cv::Vec4d &elem_copy = compensated_vectors.at<cv::Vec4d>(i, j);
                        elem_copy[0] = elem[0];
                        elem_copy[1] = elem[1];
                        elem_copy[2] = elem[2];
                        elem_copy[3] = elem[3];
                    }
                }
        }
    }
}

void EgomotionCompensator::getFOE(const std::vector<double> &odom, double &foe_x, double &foe_y)
{
    foe_x = odom[0] / odom[2];
    foe_y = odom[1] / odom[2];
}

double EgomotionCompensator::getExpectedOrientation(double x, double y, double foe_x, double foe_y, int image_width, int image_height)
{
    int x_offset = image_width / 2;
    int y_offset = image_height / 2;

    double x_uv = x - x_offset;
    double y_uv = y - y_offset;

    return atan2(y_uv - foe_y, x_uv - foe_x);
}
