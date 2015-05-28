//______________________________________________________________________________________
// Taken by  SimplAR 2 - OpenCV Simple Augmented Reality Program with Chessboard
// SAR demo with chessboard and projector
// it calulates two homographies Hcb and Hpc. Hpc is calculated by clicking on the picture projected on the wall. Combining the two homograpies 
// we have sar effect as long as the camera and the projector and on the same plane.
//
// Laser Projector - Calibration
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
#include "std_msgs/Float32MultiArray.h"
#include <iostream>

using namespace std;
using namespace cv;


#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 5

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
//The pattern actually has 6 x 5 squares, but has 5 x 4 = 20 'ENCLOSED' corners

vector<Point2f> vClick,vDisplay, vLaser, vLaserWarped;    // stores 4 points that the user clicks(mouse left click) in the main image.
Mat image,displayWarped,displayWarped_t;
Mat display;
Mat  imageDesktop(800,1280, CV_8UC3,Scalar(0,0,255) );

Mat Hpc, Hcb, Htot;
vector<Point2f> dst;      // Destination Points to transform overlay image  
vector<Point2f> corners;  
cv_bridge::CvImagePtr cv_ptr;
Mat imageCamera;
std_msgs::Float32MultiArray msg;
int key ;

//laser origin measuerd in camera space
double Xl_image;
double Yl_image;

// laser init points
float Ax = 0;
float Ay = 0;

float Bx = 1;
float By = -1;

float Cx = -1;
float Cy = -1;

float Dx = -1;
float Dy = 1;

bool token = false;
bool go = false;
bool stop;

Size board_size(CHESSBOARD_WIDTH-1, CHESSBOARD_HEIGHT-1);


bool bTemplate,bTemplate2;

vector<Point2f> projectedPoints(4);      
Mat imageRect;


void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
 
            //laser origin in camera space
            Xl_image = corners.at(0).x; 
            Yl_image = corners.at(0).y;

            //Ax = (double(x) - Xl_image)*36/IMAGE_WIDTH;            
            //Ay = (-1*double(y) + Yl_image)*24/IMAGE_HEIGHT;

            Ax = 0;            
            Ay = 0;

            Bx = (corners.at(4).x - Xl_image)*36/IMAGE_WIDTH;            
            By = (-1*corners.at(4).y + Yl_image)*24/IMAGE_HEIGHT;

            Cx = (corners.at(19).x - Xl_image)*36/IMAGE_WIDTH;            
            Cy = (-1*corners.at(19).y + Yl_image)*24/IMAGE_HEIGHT;

            Dx = (corners.at(12).x - Xl_image)*36/IMAGE_WIDTH;            
            Dy = (-1*corners.at(12).y + Yl_image)*24/IMAGE_HEIGHT;


            cout << " Xl " << Xl_image << " Yl " << Yl_image <<endl;
            cout << " x " << x << " y " << y <<endl;
            cout << " Bx " << Bx << " By " << By <<endl;
            
            msg.data.clear();

            msg.data.push_back(4); // size of the vector
            msg.data.push_back(Ax); //0
            msg.data.push_back(Ay); //1
            msg.data.push_back(Bx); //2
            msg.data.push_back(By); //3
            msg.data.push_back(Cx); //4
            msg.data.push_back(Cy); //5
            msg.data.push_back(Dx); //6
            msg.data.push_back(Dy); //7


            cout << " Projecting the point" <<endl;
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

      //cout << "left corner is " << corners[0] << endl;

    }

    setMouseCallback("Camera",on_mouse, NULL );

    imshow("Camera", imageCamera);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    token = true;
 }


}

int main ( int argc, char **argv )
{


  token = true;
  stop = false;
  display = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png");
  //imageDesktop = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png", CV_LOAD_IMAGE_COLOR);

  // corners of the image to be projected
  vDisplay.push_back(Point2f(float(0),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(display.rows)));
  vDisplay.push_back(Point2f(float(0),float(display.rows)));

  cout << vDisplay << "  vDisplay " << endl;

  ros::init(argc, argv, "spatialar_ros");

  ros::NodeHandle n;

  ros::Subscriber sub_image = n.subscribe("/image_rect_color", 3, receivedImage);
  ros::Publisher pub_laser = n.advertise<std_msgs::Float32MultiArray>("coordinates", 1);


  ros::Rate loop_rate(100);


  // project the origin
  msg.data.push_back(8); // size of the vector
  msg.data.push_back(0); //0
  msg.data.push_back(0); //1
  msg.data.push_back(0); //2
  msg.data.push_back(0); //3
  msg.data.push_back(0); //4
  msg.data.push_back(0); //5
  msg.data.push_back(0); //6
  msg.data.push_back(0); //7


  while((char) key != 'q')
  {
    key = cvWaitKey(15); // 100 Hz

    //cout << "stop" << stop << endl; 

    pub_laser.publish(msg);

    //loop_rate.sleep();

    ros::spinOnce();

    token = true;

  }

  ROS_INFO("Stop request");

  token = false; //stop receiving images

  msg.data.clear();

  while(pub_laser.getNumSubscribers() == 0)
      ROS_INFO("nobody is listen to me");
      
  for(int i=0; i< 100; i++){

      // for finishing
      msg.data.push_back(16); // size of the vector
      msg.data.push_back(10); //00
      msg.data.push_back(10); //10
      msg.data.push_back(10); //2
      msg.data.push_back(10); //3
      msg.data.push_back(10); //4
      msg.data.push_back(10); //5
      msg.data.push_back(10); //6
      msg.data.push_back(10); //7
      msg.data.push_back(10); //00
      msg.data.push_back(10); //10
      msg.data.push_back(10); //2
      msg.data.push_back(10); //3
      msg.data.push_back(10); //4
      msg.data.push_back(10); //5
      msg.data.push_back(10); //6
      msg.data.push_back(10); //7

      //ROS_INFO("publishing the final message");

      pub_laser.publish(msg);

      msg.data.clear();


      ros::spinOnce();


  }

    ROS_INFO("Ended");

  return 0;
}
