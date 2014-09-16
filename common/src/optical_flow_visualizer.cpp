#include <motion_detection/optical_flow_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

OpticalFlowVisualizer::OpticalFlowVisualizer()
{
}

OpticalFlowVisualizer::~OpticalFlowVisualizer()
{    
}

void OpticalFlowVisualizer::showOpticalFlowVectors(const cv::Mat &original_image, cv::Mat &optical_flow_image, const cv::Mat &optical_flow_vectors, int pixel_step)
{

    original_image.copyTo(optical_flow_image);

    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            cv::Point2f start_point;
            cv::Point2f end_point;

            cv::Vec4d elem = optical_flow_vectors.at<cv::Vec4d>(i, j);

            start_point.x = elem[0];
            start_point.y = elem[1];

            end_point.x = start_point.x + elem[3] * cos(elem[2]);
            end_point.y = start_point.y + elem[3] * sin(elem[2]);

            cv::line(optical_flow_image, start_point, end_point, CV_RGB(0, 0, 255), 1, CV_AA, 0);

            double backward_angle = atan2(start_point.y - end_point.y, start_point.x - end_point.x);
            double arrow1_angle = (backward_angle + M_PI / 4.0);
            double arrow2_angle = (backward_angle - M_PI / 4.0);

            cv::Point2f arrow1_end;
            arrow1_end.x = end_point.x + 3.0 * cos(arrow1_angle);
            arrow1_end.y = end_point.y + 3.0 * sin(arrow1_angle);

            cv::Point2f arrow2_end;
            arrow2_end.x = end_point.x + 3.0 * cos(arrow2_angle);
            arrow2_end.y = end_point.y + 3.0 * sin(arrow2_angle);

            cv::line(optical_flow_image, end_point, arrow1_end, CV_RGB(0, 0, 255), 1, CV_AA, 0);
            cv::line(optical_flow_image, end_point, arrow2_end, CV_RGB(0, 0, 255), 1, CV_AA, 0);
        }
    }
}
