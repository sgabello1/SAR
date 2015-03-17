#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


   Mat mImage, mImage1;
   Mat mRotated(1000, 1000, CV_8UC3), mRotated1, mRotated2;
   Mat mNormal;
   vector<Point2f> vStraight, vStraight45; 

/** @function main */
 int main()
 {
    Mat mResult;
    Mat mImage = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/toproj.png", CV_LOAD_IMAGE_COLOR);

    vStraight45.push_back(Point2f(float(199), float(257))); // A
    vStraight45.push_back(Point2f(float(193), float(868))); //B
    vStraight45.push_back(Point2f(float(1388), float(738))); // C
    vStraight45.push_back(Point2f(float(1383), float(148))); // D

    vStraight.push_back(Point2f(float(576), float(133))); // A
    vStraight.push_back(Point2f(float(579), float(726))); //B
    vStraight.push_back(Point2f(float(1358), float(722))); // C
    vStraight.push_back(Point2f(float(1356), float(128))); // D

    Mat H;
    H = findHomography(vStraight45,vStraight,0 ); // from drone image to rectified image
    warpPerspective(mImage, mResult, H, mResult.size() );
    imshow("Homography",mResult);
   
   // Mat mResult;
   // mImage = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/straight_drone_pic.png", CV_LOAD_IMAGE_COLOR);

   //    // Push the 4 corners of the logo image as the 4 points for correspondence to calculate homography.
   //  vStraight.push_back(Point2f(float(200), float(42))); // A
   //  vStraight.push_back(Point2f(float(195), float(238))); //B
   //  vStraight.push_back(Point2f(float(422), float(238))); // C
   //  vStraight.push_back(Point2f(float(418), float(47))); // D

   //  Mat A = (Mat_<double>(2,1) <<
   //            200, 
   //            42 );
   //  Mat B = (Mat_<double>(2,1) <<
   //            195, 
   //            238 );
   //  Mat C = (Mat_<double>(2,1) <<
   //            422, 
   //            238 );
   //  Mat D = (Mat_<double>(2,1) <<
   //            418, 
   //            47 );


   //  Mat Aff = (Mat_<double>(2,2) <<
   //            1, 0,
   //            1, 1
   //                );

   //  Mat Ap = Aff*A;

   //  cout << "Ap = " << Ap << "\n";

   //  Mat Bp = Aff*B;

   //  cout << "Bp = " << Bp << "\n";

   //  Mat Cp = Aff*C;

   //  cout << "Cp = " << Cp << "\n";

   //  Mat Dp = Aff*D;    

   //  cout << "Dp = "  << Dp << "\n";

   //  vAffine.push_back(Point2f(double(Ap.at<double>(0)),double(Ap.at<double>(1) - 400))); // A
   //  vAffine.push_back(Point2f(double(Bp.at<double>(0)),double(Bp.at<double>(1) - 400))); // A
   //  vAffine.push_back(Point2f(double(Cp.at<double>(0)),double(Cp.at<double>(1) - 400))); // A
   //  vAffine.push_back(Point2f(double(Dp.at<double>(0)),double(Dp.at<double>(1) - 400))); // A

   //  circle(mImage, vAffine[0], 7, Scalar(0,0,255),-1); //red
   //  circle(mImage, vAffine[1], 7, Scalar(0,255,0),-1); // green
   //  circle(mImage, vAffine[2], 7, Scalar(255,0,0),-1);
   //  circle(mImage, vAffine[3], 7, Scalar(255,0,255),-1);

   //  // circle(mImage, vStraight[0], 7, Scalar(255,0,0),-1);
   //  // circle(mImage, vStraight[1], 7, Scalar(255,0,0),-1);
   //  // circle(mImage, vStraight[2], 7, Scalar(255,0,0),-1);
   //  // circle(mImage, vStraight[3], 7, Scalar(255,0,0),-1);     

   // imshow("Affine Transformation",mImage);

   waitKey(0);

   return 0;
  }