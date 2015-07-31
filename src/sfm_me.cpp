//______________________________________________________________________________________
// Trying to set up a Sfm algortihm
//______________________________________________________________________________________

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

Mat img1,img2;

int main ( int argc, char **argv )
{
  


  img1 = cv::imread("im01.jpg");
  img2 = cv::imread("im02.jpg");

  cv::imshow("img1", img1);
  cv::imshow("img2", img2);



  return 0;
}
