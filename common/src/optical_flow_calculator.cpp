#include <motion_detection/optical_flow_calculator.h>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <motion_detection/VarFlow.h>

OpticalFlowCalculator::OpticalFlowCalculator()
{

}

OpticalFlowCalculator::~OpticalFlowCalculator()
{

}

int OpticalFlowCalculator::calculateOpticalFlow(cv::Mat image1, cv::Mat image2, cv::Mat &optical_flow_image, cv::Mat &optical_flow_vectors)
{
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

    for (int i = 0; i < image1.cols; i = i + 20)
    {
        for (int j = 0; j < image1.rows; j = j + 20)
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
