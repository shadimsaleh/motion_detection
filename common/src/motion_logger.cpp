/* motion_logger.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <motion_detection/motion_logger.h>

MotionLogger::MotionLogger(const bfs::path &log_filepath) : file_out_(log_filepath.string().c_str())
{

}

MotionLogger::MotionLogger()
{

}

MotionLogger::~MotionLogger()
{
    file_out_.close();
}

void MotionLogger::setFileName(const bfs::path &log_filepath)
{
    file_out_.open(log_filepath.string().c_str());
}

void MotionLogger::writeContour(const std::vector<cv::Point> &points, int frame_number, int contour_id)
{
    file_out_ << frame_number << ", " << contour_id;
    for (int i = 0; i < points.size(); i++)
    {
        cv::Point p = points.at(i);
        file_out_ <<", " << p.x << ", " << p.y;
    }
    file_out_ << std::endl;
}


void MotionLogger::writeBoundingBox(const cv::Rect &rect, int frame_number, int contour_id)
{
    file_out_ << frame_number << ", " << contour_id;
    file_out_ << ", " << rect.tl().x << ", " << rect.tl().y << ", " << rect.br().x << ", " << rect.br().y << std::endl;
}
