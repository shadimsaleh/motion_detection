#include <motion_detection/optical_flow_calculator.h>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <motion_detection/VarFlow.h>
#include <motion_detection/slic.h>
#include <fstream>

OpticalFlowCalculator::OpticalFlowCalculator()
{

}

OpticalFlowCalculator::~OpticalFlowCalculator()
{

}

int OpticalFlowCalculator::calculateOpticalFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors, int pixel_step, cv::Mat &comp, double min_vector_size)
{
    /*
    const int MAX_LEVEL = 2;
    cv::Size winSize(40, 40);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);
    */

    const int MAX_LEVEL = 5;
    cv::Size winSize(40, 40);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);

    
    cv::Mat gray_image1;
    cv::Mat gray_image2;

    cvtColor(image1, gray_image1, CV_BGR2GRAY);
    cvtColor(image2, gray_image2, CV_BGR2GRAY);

    std::vector<cv::Point2f> points_image1;
    std::vector<cv::Point2f> points_image2;

    for (int i = 0; i < image1.cols; i = i + pixel_step)
    {
        for (int j = 0; j < image1.rows; j = j + pixel_step)
        {
            cv::Point2f point(i, j);
            //std::cout << "adding point " << i << ", " << j << std::endl;
            points_image1.push_back(point);
        }
    }

    std::vector<cv::Mat> pyramids;
    cv::buildOpticalFlowPyramid(gray_image1, pyramids, winSize, MAX_LEVEL, true);

    //cv::calcOpticalFlowPyrLK(gray_image1, gray_image2, points_image1, points_image2, status, err, winSize, MAX_LEVEL, termcrit, 0, 0.001);

    cv::calcOpticalFlowPyrLK(pyramids, gray_image2, points_image1, points_image2, status, err, winSize, MAX_LEVEL, termcrit, 0, 0.001);

    int num_vectors = 0;

    std::vector<cv::Point2f> src_points;
    std::vector<cv::Point2f> dst_points;

    for (int i = 0; i < points_image2.size(); i++)
    {
        if (status[i])
        {
            cv::Point2f start_point = points_image1.at(i);
            cv::Point2f end_point = points_image2.at(i);
            float x_diff = end_point.x - start_point.x;
            float y_diff = end_point.y - start_point.y;
            if (std::abs(x_diff) > min_vector_size || std::abs(y_diff) > min_vector_size) // THIS USED TO BE 1.0
            {
                cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                elem[0] = start_point.x;
                elem[1] = start_point.y;
                elem[2] = x_diff;
                elem[3] = y_diff;
                //elem[2] = atan2(y_diff, x_diff);
                //elem[3] = sqrt(x_diff*x_diff + y_diff*y_diff);
                src_points.push_back(start_point);
                dst_points.push_back(end_point);
                num_vectors++;
            }
            else
            {
                cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                elem[0] = start_point.x;
                elem[1] = start_point.y;
                elem[2] = 0.0;
                elem[3] = 0.0;
            }
        }
        else
        {
            cv::Point2f start_point = points_image1.at(i);
            cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
            elem[0] = -1.0;
            elem[1] = -1.0;
            elem[2] = 0.0;
            elem[3] = 0.0;
        }
    }
    if (num_vectors > 0)
    {
        cv::Mat pers_transform = cv::getPerspectiveTransform(&src_points[0], &dst_points[0]);
        cv::Mat compensated;
        gray_image2.copyTo(compensated);
        gray_image2.copyTo(comp);
        cv::warpPerspective(gray_image1, compensated, pers_transform, cv::Size(gray_image1.cols, gray_image1.rows));
        cv::absdiff(compensated, gray_image2, comp);
        //std::cout << comp << std::endl;
        cv::threshold(comp, comp, 190, 255, CV_THRESH_BINARY);
    }
    return num_vectors;
}


int OpticalFlowCalculator::calculateOpticalFlowTrajectory(const std::vector<cv::Mat> &images, cv::Mat &optical_flow_vectors, 
                                        std::vector<std::vector<cv::Point2f> > &trajectories, int pixel_step, cv::Mat &comp, double min_vector_size)
{
    
    const int MAX_LEVEL = 5;
    cv::Size winSize(40, 40);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);

    std::vector<std::vector<cv::Point2f> > init_traj_list; 

    // initialize points we want to track
    std::vector<cv::Point2f> points_image1;
    std::vector<cv::Point2f> points_image2;
    for (int i = 0; i < images[0].cols; i = i + pixel_step)
    {
        for (int j = 0; j < images[0].rows; j = j + pixel_step)
        {
            cv::Point2f point(i, j);
            points_image1.push_back(point);
            std::vector<cv::Point2f> traj;
            traj.push_back(point);
            init_traj_list.push_back(traj);
        }
    }

    int num_vectors = 0;
    for (int j = 0; j < images.size() - 1; j++)
    {
        cv::Mat gray_image1;
        cv::Mat gray_image2;

        cvtColor(images.at(j), gray_image1, CV_BGR2GRAY);
        cvtColor(images.at(j+1), gray_image2, CV_BGR2GRAY);

        std::vector<cv::Mat> pyramids;
        cv::buildOpticalFlowPyramid(gray_image1, pyramids, winSize, MAX_LEVEL, true);

        cv::calcOpticalFlowPyrLK(pyramids, gray_image2, points_image1, points_image2, status, err, winSize, MAX_LEVEL, termcrit, 0, 0.001);


        std::vector<cv::Point2f> temp;
        
        int found_vectors = 0;
        for (int i = 0; i < points_image2.size(); i++)
        {
            if (status[i])
            {
                found_vectors++;
                if (j == images.size() - 2)
                {
                    cv::Point2f start_point = points_image1.at(i);
                    cv::Point2f end_point = points_image2.at(i);
                    float x_diff = end_point.x - start_point.x;
                    float y_diff = end_point.y - start_point.y;
                    if (std::abs(x_diff) > min_vector_size || std::abs(y_diff) > min_vector_size) // THIS USED TO BE 1.0
                    {
                        cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                        elem[0] = start_point.x;
                        elem[1] = start_point.y;
                        elem[2] = x_diff;
                        elem[3] = y_diff;
                        num_vectors++;
                    }
                    else
                    {
                        cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                        elem[0] = start_point.x;
                        elem[1] = start_point.y;
                        elem[2] = 0.0;
                        elem[3] = 0.0;
                    }
                }
                if (points_image2.at(i).x > 10.0 && points_image2.at(i).y > 10.0
                    && points_image2.at(i).x < images[0].cols-10 && points_image2.at(i).y < images[0].rows-10)
                {
                    temp.push_back(points_image2.at(i));
                    init_traj_list.at(i).push_back(points_image2.at(i));
                }
                else
                {
                    temp.push_back(points_image1.at(i));
                }
            }
            else
            {
                if (j == images.size() - 2)
                {
                    //std::cout << "in here " << std::endl;
                    cv::Point2f start_point = points_image1.at(i);
                    //std::cout << "start_point " << start_point.x << ", " << start_point.y << std::endl;
                    cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                    elem[0] = -1.0;
                    elem[1] = -1.0;
                    elem[2] = 0.0;
                    elem[3] = 0.0;
                }
                if (points_image1.at(i).x < 0.0 || points_image1.at(i).y < 0.0)
                {
                }
                temp.push_back(points_image1.at(i));
            }
        }
        points_image1.clear();
        for (int xyz=0;xyz<temp.size();xyz++)
        {
            points_image1.push_back(temp.at(xyz));
        }
        points_image2.clear();
    }
    for (int i = 0; i < init_traj_list.size(); i++)
    {
        if (init_traj_list.at(i).size() == images.size())
        {
            trajectories.push_back(init_traj_list.at(i));
        }
        else 
        {
            //std::cout << "size: " << init_traj_list.at(i).size() << std::endl;
        }
    }
    
    return num_vectors;
}






int OpticalFlowCalculator::calculateCompensatedFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_vectors, int pixel_step)
{
    const int MAX_LEVEL = 2;
    cv::Size winSize(40, 40);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);

    
    cv::Mat gray_image1;
    cv::Mat gray_image2;

    cvtColor(image1, gray_image1, CV_BGR2GRAY);
    cvtColor(image2, gray_image2, CV_BGR2GRAY);

    std::vector<cv::Point2f> points_image1;
    std::vector<cv::Point2f> points_image2;

    for (int i = 0; i < image1.cols; i = i + pixel_step)
    {
        for (int j = 0; j < image1.rows; j = j + pixel_step)
        {
            cv::Point2f point(i, j);
            //std::cout << "adding point " << i << ", " << j << std::endl;
            points_image1.push_back(point);
        }
    }

    cv::calcOpticalFlowPyrLK(gray_image1, gray_image2, points_image1, points_image2, status, err, winSize, MAX_LEVEL, termcrit, 0, 0.001);


    int num_vectors = 0;

    for (int i = 0; i < points_image2.size(); i++)
    {
        if (status[i])
        {
            cv::Point2f start_point = points_image1.at(i);
            cv::Point2f end_point = points_image2.at(i);
            float x_diff = end_point.x - start_point.x;
            float y_diff = end_point.y - start_point.y;
            if (std::abs(x_diff) > 1.0 || std::abs(y_diff) > 1.0)
            {
                cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                elem[0] = start_point.x;
                elem[1] = start_point.y;
                elem[2] = x_diff;
                elem[3] = y_diff;
                //elem[2] = atan2(y_diff, x_diff);
                //elem[3] = sqrt(x_diff*x_diff + y_diff*y_diff);
                num_vectors++;
            }
            else
            {
                cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                elem[0] = start_point.x;
                elem[1] = start_point.y;
                elem[2] = 0.0;
                elem[3] = 0.0;
            }
        }
        else
        {
            cv::Point2f start_point = points_image1.at(i);
            cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
            elem[0] = -1.0;
            elem[1] = -1.0;
            elem[2] = 0.0; 
            elem[3] = 0.0;
        }
    }
}

int OpticalFlowCalculator::superPixelFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow_image, cv::Mat &optical_flow_vectors)
{
    int width = image1.cols;
    int height = image1.rows;
    

    IplImage *im1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U,3); 
    im1->imageData = (char *)image1.data;

    
    int number_of_superpixels = 50;
    int nc = 40;

    double step = sqrt((width * height) / (double) number_of_superpixels);


    Slic slic;
    IplImage *im1_lab = cvCloneImage(im1);
    cvCvtColor(im1, im1_lab, CV_BGR2Lab);
    slic.generate_superpixels(im1_lab, step, nc);

    std::vector<std::vector<double> > centers = slic.get_centers();

    const int MAX_LEVEL = 2;
    cv::Size winSize(40, 40);
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);

    image1.copyTo(optical_flow_image);
    
    cv::Mat gray_image1;
    cv::Mat gray_image2;

    cvtColor(image1, gray_image1, CV_BGR2GRAY);
    cvtColor(image2, gray_image2, CV_BGR2GRAY);

    std::vector<cv::Point2f> points_image1;
    std::vector<cv::Point2f> points_image2;

    std::cout << "adding " << centers.size() << " points" << std::endl;
    for (int i = 0; i < centers.size(); i++)
    {
        cv::Point2f point(centers[i][3], centers[i][4]);
        //std::cout << "adding point " << i << ", " << j << std::endl;
        points_image1.push_back(point);
    }

    cv::calcOpticalFlowPyrLK(gray_image1, gray_image2, points_image1, points_image2, status, err, winSize, MAX_LEVEL, termcrit, 0, 0.001);


    int num_vectors = 0;
    for (int i = 0; i < points_image2.size(); i++)
    {
        if (status[i])
        {
            cv::Point2f start_point = points_image1.at(i);
            cv::Point2f end_point = points_image2.at(i);
            float x_diff = end_point.x - start_point.x;
            float y_diff = end_point.y - start_point.y;
            if (x_diff > 1.0 || y_diff > 1.0)
            {
                cv::Vec4d &elem = optical_flow_vectors.at<cv::Vec4d> ((int)start_point.y, (int)start_point.x);
                elem[0] = start_point.x;
                elem[1] = start_point.y;
               // elem[2] = end_point.x - start_point.x;
              //  elem[3] = end_point.y - start_point.y;
              //  elem[4] = sqrt(elem[2] * elem[2] + elem[3] * elem[3]);
              //  elem[5] = atan2(elem[3], elem[2]);
                elem[2] = atan2(y_diff, x_diff);
                elem[3] = sqrt(x_diff*x_diff + y_diff*y_diff);
                //std::cout << "elem is : " << optical_flow_vectors.at<cv::Vec4d>((int)start_point.y, (int)start_point.x) << std::endl;
                //std::cout << "Point " << start_point.x << ", " << start_point.y << " moved to " << end_point.x << ", " << end_point.y << std::endl;
                cv::line(optical_flow_image, start_point, end_point, CV_RGB(255,0,0), 1, CV_AA, 0);
                num_vectors++;
            }
        }
    }
    return num_vectors;
}
void OpticalFlowCalculator::varFlow(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &optical_flow, cv::Mat &optical_flow_vectors)
{

    int width = image1.cols;
    int height = image1.rows;
    int max_level = 4;
    int start_level = 0;
    int n1 = 2;
    int n2 = 2;

    float rho = 2.8;
    float alpha = 1400;
    float sigma = 1.5;    

    //std::cout << "before var flow" << std::endl;
    VarFlow var_flow(width, height, max_level, start_level, n1, n2, rho, alpha, sigma);


    IplImage *im1 = cvCreateImage(cvSize(width, height), 8,1); 
    im1->imageData = (char *)image1.data;

    IplImage *im2 = cvCreateImage(cvSize(width, height), 8,1); 
    im2->imageData = (char *)image2.data;



//    IplImage *im1 = cvCloneImage(&(IplImage)image1);
//    IplImage *im2 = cvCloneImage(&(IplImage)image2);
    IplImage* imgU = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    IplImage* imgV = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    //std::cout << "created IplImages" << std::endl;
    cvZero(imgU);
    cvZero(imgV);
    //std::cout << "zeroed images " << std::endl;

    var_flow.CalcFlow(im1, im2, imgU, imgV, 0);
    //std::cout << "ran calc flow " << std::endl;

    IplImage* imgMotion = cvCreateImage(cvSize(width, height), 8, 3);
    imgMotion->imageData = (char *)image2.data;
    
    //cvZero(imgMotion);
    drawMotionField(imgU, imgV, imgMotion, 10, 10, 5, 1, CV_RGB(255,0,0));
    

    cv::Mat m = cv::cvarrToMat(imgMotion);
    m.copyTo(optical_flow);
}

// Draw a vector field based on horizontal and vertical flow fields
void OpticalFlowCalculator::drawMotionField(IplImage* imgU, IplImage* imgV, IplImage* imgMotion, int xSpace, int ySpace, float cutoff, int multiplier, CvScalar color)
{
     int x, y;
    
     CvPoint p0 = cvPoint(0,0);
     CvPoint p1 = cvPoint(0,0);
     
     float deltaX, deltaY, angle, hyp;
     
     for(y = ySpace; y < imgU->height; y+= ySpace ) {
        for(x = xSpace; x < imgU->width; x+= xSpace ){
         
            p0.x = x;
            p0.y = y;
            
            deltaX = *((float*)(imgU->imageData + y*imgU->widthStep)+x);
            deltaY = -(*((float*)(imgV->imageData + y*imgV->widthStep)+x));
            
            angle = atan2(deltaY, deltaX);
            hyp = sqrt(deltaX*deltaX + deltaY*deltaY);
   
            if(hyp > cutoff){
                   
                p1.x = p0.x + cvRound(multiplier*hyp*cos(angle));
                p1.y = p0.y + cvRound(multiplier*hyp*sin(angle));
                   
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
                
                p0.x = p1.x + cvRound(3*cos(angle-M_PI + M_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-M_PI + M_PI/4));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
                
                p0.x = p1.x + cvRound(3*cos(angle-M_PI - M_PI/4));
                p0.y = p1.y + cvRound(3*sin(angle-M_PI - M_PI/4));
                cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
            }
      
        }
    }
    
}

void OpticalFlowCalculator::writeFlow(const cv::Mat &optical_flow_vectors, const std::string &filename, int pixel_step)
{
    std::string horizontal_f = filename + "_h";
    std::string vertical_f = filename + "_f";
    ofstream hfile(horizontal_f);
    ofstream vfile(vertical_f);
    for (int i = 0; i < optical_flow_vectors.rows; i = i + pixel_step)
    {
        for (int j = 0; j < optical_flow_vectors.cols; j = j + pixel_step)
        {
            if (j != 0)
            {
                hfile << ", ";
                vfile << ", ";
            }
            cv::Vec4d elem = optical_flow_vectors.at<cv::Vec4d>(i, j);
            if (elem[0] == -1.0)
            {
                hfile << 0.0;
                vfile << 0.0;
            }
            else
            {
                hfile << elem[2];
                vfile << elem[3];
            }
        }
        hfile << endl;
        vfile << endl;
    }
    hfile.close();
    vfile.close();
}

void OpticalFlowCalculator::writeTrajectories(const std::vector<std::vector<cv::Point2f> > &trajectories, const std::string &filename)
{

    ofstream tfile(filename);

    for (int i = 0; i < trajectories.size(); i++)
    {
        std::vector<cv::Point2f> traj = trajectories.at(i);
        for (int j = 0; j < traj.size(); j++)
        {
            if (j != 0)
            {
                tfile << ", ";                
            }
            tfile << traj.at(j).x << ", " << traj.at(j).y;
        }
        tfile << std::endl;
    }
    tfile.close();
}
