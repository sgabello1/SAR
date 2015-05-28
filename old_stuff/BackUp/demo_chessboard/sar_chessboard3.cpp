//______________________________________________________________________________________
// Taken by  SimplAR 2 - OpenCV Simple Augmented Reality Program with Chessboard
// SAR demo with chessboard and projector
// it calulates two homographies Hcb and Hpc. Hpc is calculated by clicking on the picture projected on the wall. Combining the two homograpies 
// we have sar effect as long as the camera and the projector and on the same plane.
// AUTOCAILBRATION ADDED!
//______________________________________________________________________________________

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <iostream>

using namespace std;
using namespace cv;


#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 5
//The pattern actually has 6 x 5 squares, but has 5 x 4 = 20 'ENCLOSED' corners

vector<Point2f> vClick,vDisplay;    // stores 4 points that the user clicks(mouse left click) in the main image.
Mat image,displayWarped,displayWarped_t;
Mat display;
Mat  imageDesktop(800,1280, CV_8UC3,Scalar(0,0,255)), imageChessBoard;

Mat Hpc, Hcb, Htot;
vector<Point2f> dst,dst_copy;      // Destination Points to transform overlay image  
vector<Point2f> corners;  
cv_bridge::CvImagePtr cv_ptr;
Mat imageCamera;
Mat imageChessBoardGray;

vector<Point2f> cbCornersProjector;
vector<Point2f> cornersTemplate;

bool token = false;
bool go = false;
bool iWantCorners = false;

Size board_size(CHESSBOARD_WIDTH-1, CHESSBOARD_HEIGHT-1);


bool bTemplate,bTemplate2;

vector<Point2f> projectedPoints(4);      
Mat imageRect;


void findCamera2ProjectorHomography(){
   
    iWantCorners = true;
 
    cvtColor(imageChessBoard, imageChessBoardGray, CV_BGR2GRAY);


    bTemplate2 = findChessboardCorners(imageChessBoard, board_size, cornersTemplate, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (bTemplate2 == 1){

    cout << "Founded corners in the template "<< cornersTemplate.size() << "\n";

    if(cornersTemplate.size() > 1)
    {
    cornerSubPix(imageChessBoardGray, cornersTemplate, Size(11,11), Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    cout << "Subpixel founded\n";
    }
      

    // corners detected in the projector cess board (template image)
    cbCornersProjector.push_back(cornersTemplate[0]);
    cbCornersProjector.push_back(cornersTemplate[CHESSBOARD_WIDTH-2]);
    cbCornersProjector.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
    cbCornersProjector.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

    cout << "Finded template corners...\n";
    cout << cbCornersProjector;
    cout << "\n";

    if(dst_copy.size() == 4) {
    Hpc = findHomography(dst_copy,cbCornersProjector,0); //src dst -> chessboard u see, chessboard you want to project

    
    go = true;

    cout << "Hpc found " << Hpc << "\n now going for online warping\n";
    }
    else
    {

        cout << "dst is " << dst_copy << "\n";

        cout << "Projected chessboard (dst) NOT found "<< dst_copy.size() << " and cbCornersProjector " << cbCornersProjector.size() << "\n";
        cbCornersProjector.clear();

    }
    }
    else{

        cout << "Corners in the template NOT founded return "<< bTemplate2 << "\n";

    }

  }



void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
         
         cout <<" just go for calibration "<<endl;
         findCamera2ProjectorHomography();   

    }  
}

  void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{
    if(token ){
    try
    {

    token = false;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    imageCamera=cv_ptr->image; // fire fly

    Mat cpy_image(imageCamera.rows, imageCamera.cols, imageCamera.type());
    Mat neg_image(imageCamera.rows, imageCamera.cols, imageCamera.type());
    Mat gray;
    Mat blank(display.rows, display.cols, display.type());

    cvtColor(imageCamera, gray, CV_BGR2GRAY);

    bool flag = findChessboardCorners(imageCamera, board_size, corners);


    if(flag == 1)
    {            

      // This function identifies the chessboard pattern from the gray image, saves the valid group of corners
      cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

      // chess board corners in the camera space
      dst.push_back(corners[0]);
      dst.push_back(corners[CHESSBOARD_WIDTH-2]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

      if(iWantCorners)
      {
        dst_copy = dst;
        cout <<" dst copy  " << dst_copy <<endl;
        iWantCorners = false;
      }


    if(go)
    {
      if(dst.size() == 4 && vDisplay.size() == 4)
      {
      
      Mat Hcb = findHomography(vDisplay, dst,0);

      blank = Scalar(0);
      neg_image = Scalar(0);                // Image is white when pixel values are zero
      cpy_image = Scalar(0);                // Image is white when pixel values are zero

      bitwise_not(blank,blank);

      warpPerspective(display, displayWarped_t, Hcb, imageDesktop.size());

      Mat image2;
      warpPerspective(display, neg_image, Hcb, Size(neg_image.cols, neg_image.rows)); // Transform overlay Image to the position  - [ITEM1]
      warpPerspective(blank, cpy_image, Hcb, Size(cpy_image.cols, neg_image.rows));   // Transform a blank overlay image to position  
      bitwise_not(cpy_image, cpy_image);              // Invert the copy paper image from white to black
      bitwise_and(cpy_image, imageCamera, cpy_image);           // Create a "hole" in the Image to create a "clipping" mask - [ITEM2]           
      bitwise_or(cpy_image, neg_image, image2);            // Finally merge both items [ITEM1 & ITEM2]

      imshow("Camera 2", image2);

      //cout <<" Hcb " << Hcb << " and Hpc" << Hpc <<endl;

      Htot = Hpc*Hcb;
      warpPerspective(display, displayWarped, Htot, imageChessBoard.size()); //imageChessBoard, because we calibrated in this image
      imshow("Final",displayWarped);

      
      }

      }

      dst.clear();
      token = true;
    }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
     
    imshow("Camera", imageCamera);
    setMouseCallback("Camera",on_mouse, NULL );
    token = true;


 }
    

}

int main ( int argc, char **argv )
{

  display = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png");

  // this is the image that has to be reconized (cessboard projected) cb1280_800.png
  imageChessBoard = imread("/home/sgabello/0catkin_ws/src/spatially_ar/src/cb652.png");


  if( !imageChessBoard.data )
  { 
    std::cout<< " --(!) Error reading images " << std::endl;  
    
    return -1;
    }
  else
  {
    std::cout<< "cb652.png ok " << std::endl;
    imshow("Final", imageChessBoard);
  }
 
  // corners of the image to be projected
  vDisplay.push_back(Point2f(float(0),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(display.rows)));
  vDisplay.push_back(Point2f(float(0),float(display.rows)));

  cout << vDisplay << " corners vDisplay " << endl;

  ros::init(argc, argv, "spatialar_ros");

  ros::NodeHandle n;

  ros::Subscriber sub_image = n.subscribe("/image_rect_color", 3, receivedImage);
  
  int key = '0';

  cout <<" now click on camera window and start automatic calibration "<<endl;

  token = true;

  while(key != 'q')
  {
    key = cvWaitKey(10);
    ros::spinOnce();
  }

  if(display.empty())
  {
    cerr << "ERR: Unable to find overlay image.\n" << endl;
    return -1;
  }
  
    if(imageDesktop.empty())
  {
    cerr << "ERR: Unable to find overlay image.\n" << endl;
    return -1;
  }


  return 0;
}

