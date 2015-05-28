//______________________________________________________________________________________
// Taken by  SimplAR 2 - OpenCV Simple Augmented Reality Program with Chessboard
// SAR demo with chessboard and projector
// it calulates two homographies Hcb and Hpc. Hpc is calculated by clicking on the picture projected on the wall. Combining the two homograpies 
// we have sar effect as long as the camera and the projector and on the same plane.
//
// Laser Projector
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

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 800
//The pattern actually has 6 x 5 squares, but has 5 x 4 = 20 'ENCLOSED' corners

vector<Point2f> vClick,vTemplate,vTemplateWarped,vDisplay, vLaser, vLaserWarped;    // stores 4 points that the user clicks(mouse left click) in the main image.
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

// points in laser frame
float Ax = -0.5;
float Ay = 0.5;

float Bx = 0.5;
float By = 0.5;

float Cx = 0.5;
float Cy = -0.5;

float Dx = -0.5;
float Dy = -0.5;

// points in template frame
float Axt;
float Ayt;

float Bxt;
float Byt;

float Cxt;
float Cyt;

float Dxt;
float Dyt;

// scale factor gammax = (Otx - Btx)/(Olx - Blx) gammay = (Oty - Bty)/(Oly - Bly) 
// i put the origin Ol = (500,500) in template frame. so i put gammax = gammay = (500-750)/(0-1)=250; Bxt i want it 750
float gammax, gammay;

// translation vector from Ot to Ol tx=ty=500 because i put Ol in the middle of the template frame
float tx, ty;

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
    
    // STEP 3 - Observe points in image frame and compute Hpc with points in template frame
    if(vClick.size() < 4 )
        {
 
            vClick.push_back(Point2f(float(x),float(y))); //points in Ri
            cout << "points in image frame " << x << " "<< y <<endl;
        }
        else
        {

               
            Hpc = findHomography(vClick,vTemplate,0);
            cout << " All right, we have Projector to Camera Homography Hpc " <<endl;

            vClick.clear();
            go = true;
          
        }
     
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
      dst.push_back(corners[CHESSBOARD_WIDTH-2]); //4
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

      // perspectiveTransform(displayCorners,projectedPoints, Hcalib); 
      if(dst.size() == 4 && vDisplay.size() == 4){
        Mat Hcb = findHomography(vDisplay, dst,0);
      // Htot = findHomography( projectedPoints,dst,0);

      blank = Scalar(0);
      neg_image = Scalar(0);                // Image is white when pixel values are zero
      cpy_image = Scalar(0);                // Image is white when pixel values are zero

      bitwise_not(blank,blank);

      warpPerspective(display, displayWarped_t, Hcb, imageDesktop.size());
      //imshow("Warped t", displayWarped_t);

      Mat image2;
      warpPerspective(display, neg_image, Hcb, Size(neg_image.cols, neg_image.rows)); // Transform overlay Image to the position  - [ITEM1]
      warpPerspective(blank, cpy_image, Hcb, Size(cpy_image.cols, neg_image.rows));   // Transform a blank overlay image to position  
      bitwise_not(cpy_image, cpy_image);              // Invert the copy paper image from white to black
      bitwise_and(cpy_image, imageCamera, cpy_image);           // Create a "hole" in the Image to create a "clipping" mask - [ITEM2]           
      bitwise_or(cpy_image, neg_image, image2);            // Finally merge both items [ITEM1 & ITEM2]

      imshow("Camera 2", image2);
      if(go)
      {


        //STEP 4 - Transform points in template frame with the Htot= Hcb*Hpc homography
        Htot = Hcb*Hpc;
        perspectiveTransform(vTemplate,vTemplateWarped, Htot); 
        
        //STEP 5 - transform back points from template frame to laser frame
        Ax = (vTemplateWarped.at(0).x - tx)/gammax;
        Bx = (vTemplateWarped.at(1).x - tx)/gammax;
        Cx = (vTemplateWarped.at(2).x - tx)/gammax;
        Dx = (vTemplateWarped.at(3).x - tx)/gammax;

        Ay = (ty - vTemplateWarped.at(0).y)/gammay;
        By = (ty - vTemplateWarped.at(1).y)/gammay;
        Cy = (ty - vTemplateWarped.at(2).y)/gammay;
        Dy = (ty - vTemplateWarped.at(3).y)/gammay;

        msg.data.clear();

        msg.data.push_back(2); // size of the vector
        msg.data.push_back(Ax); //0
        msg.data.push_back(Ay); //1
        /*msg.data.push_back(Bx); //2
        msg.data.push_back(By); //3
        msg.data.push_back(Cx); //4
        msg.data.push_back(Cy); //5
        msg.data.push_back(Dx); //6
        msg.data.push_back(Dy); //7*/

      }

      dst.clear();
      }
    }

    setMouseCallback("Camera",on_mouse, NULL );

    imshow("Camera", imageCamera);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
     
    
 }


}

int main ( int argc, char **argv )
{

  token = true;
  stop = false;
  gammax = gammay = 250;

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


  ros::Rate loop_rate(60);

  // STEP 1 - project a square (1,1)
  msg.data.push_back(8); // size of the vector
  msg.data.push_back(Ax); //0
  msg.data.push_back(Ay); //1
  msg.data.push_back(Bx); //2
  msg.data.push_back(By); //3
  msg.data.push_back(Cx); //4
  msg.data.push_back(Cy); //5
  msg.data.push_back(Dx); //6
  msg.data.push_back(Dy); //7


  // STEP 2 - transform points in Template Reference (proj-to-template)
  Axt = Ax*gammax + tx;
  Bxt = Bx*gammax + tx;
  Cxt = Cx*gammax + tx;
  Dxt = Dx*gammax + tx;

  Ayt = ty - gammay*Ay;
  Byt = ty - gammay*By;
  Cyt = ty - gammay*Cy;
  Dyt = ty - gammay*Dy;

  // put points in the vector vTemplate for next step
  vTemplate.push_back(Point2f(Axt,Ayt));
  vTemplate.push_back(Point2f(Bxt,Byt));
  vTemplate.push_back(Point2f(Cxt,Cyt));
  vTemplate.push_back(Point2f(Dxt,Dyt));

  while((char) key != 'q')
  {
    key = cvWaitKey(1); // 100 Hz

    //cout << "stop" << stop << endl; 

    pub_laser.publish(msg);

    loop_rate.sleep();

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
