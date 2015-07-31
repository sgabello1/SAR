//______________________________________________________________________________________
// Taken by  SimplAR 2 - OpenCV Simple Augmented Reality Program with Chessboard
// SAR demo with chessboard and projector
// it calulates two homographies Hcb and Hcp. Hcp is calculated by clicking on the picture projected on the wall. Combining the two homograpies 
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

vector<Point2d> vClick,vCenter,vCenterWarped, vLaser, vLaserWarped;    // stores 4 points that the user clicks(mouse left click) in the main image.
std::vector<Point2d> vDisplay;
Mat image,displayWarped,displayWarped_t;
Mat display;
Mat  imageDesktop(800,1280, CV_8UC3,Scalar(0,0,255) );

Mat Hcp, Hcb,Hcb_laser, Htot;
vector<Point2d> dst;      // Destination Points to transform overlay image  
vector<Point2f> corners;  
cv_bridge::CvImagePtr cv_ptr;
Mat imageCamera;
std_msgs::Float32MultiArray msg;
int key ;

//laser origin measuerd in camera space
double Xl_image;
double Yl_image;

// points in laser frame
double Ax = -0.5;
double Ay = 0.5;

double Bx = 0.5;
double By = 0.5;

double Cx = 0.5;
double Cy = -0.5;

double Dx = -0.5;
double Dy = -0.5 ;

// scale factor gammax = (Otx - Btx)/(Olx - Blx) gammay = (Oty - Bty)/(Oly - Bly) 
// i put the origin Ol = (500,500) in template frame. so i put gammax = gammay = (500-750)/(0-1)=250; Bxt i want it 750
double gammax, gammay;

// translation vector from Ot to Ol tx=ty=500 because i put Ol in the middle of the template frame
double tx = -2.5;
double ty= 2.5;
bool token = false;
bool go = false;
bool stop;


Size board_size(CHESSBOARD_WIDTH-1, CHESSBOARD_HEIGHT-1);


bool bTemplate,bTemplate2;

vector<Point2d> projectedPoints(4);      
Mat imageRect;


void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
    
    // STEP 3 - Observe points in image frame and compute Hcp with points in the camera frame
    if(vClick.size() < 4 )
        {
 
            vClick.push_back(Point2d(double(x),double(y))); //points in Ri
            cout << "points in image frame " << x << " "<< y <<endl;
        }
        else
        {

            cout << " vLaser " << vLaser << " vClick " << vClick <<endl;   
            Hcp = findHomography(vClick,vLaser,0);
            cout << " Hcp" << Hcp <<endl;   

            cout << " All right, we have Projector to Camera Homography Hcp " <<endl;

            //vClick.clear();
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
        Hcb_laser = findHomography(vLaser, dst,0); //H of the 4 points that I want to project ;)
        
        /*if(!go){
        cout << " dst "<< dst << " \n " << endl;
        cout << " Hcb_laser "<< Hcb_laser << " \n " << endl;
       }*/
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


        //STEP 4 - Transform points in template frame with the Htot= Hcb*Hcp homography
        //Htot = Hcb_laser*Hcp;

        perspectiveTransform(dst,vLaserWarped, Hcp); 
        //perspectiveTransform(vCenter,vCenterWarped, Htot);       
        
        //STEP 5 - transform back points from camera frame to laser frame (cam-to-proj)

        Ax = vLaserWarped.at(0).x; //is just A the origin in (0,0)
        Bx = vLaserWarped.at(1).x;
        Cx = vLaserWarped.at(2).x;
        Dx = vLaserWarped.at(3).x;

        Ay = vLaserWarped.at(0).y; //is just A the origin in (0,0)
        By = vLaserWarped.at(1).y; 
        Cy = vLaserWarped.at(2).y; 
        Dy = vLaserWarped.at(3).y;

        cout << "STEP 5 - transform back points from template frame to laser frame" << endl;

        cout << "\n vLaserWarped " << vLaserWarped << "\n vLaser " <<  vLaser << endl;
        
        cout << " Ax " << Ax << " Ay " << Ay << endl;
        cout << " Bx " << Bx << " By " << By << endl;
        cout << " Cx " << Cx << " Cy " << Cy << endl;
        cout << " Dx " << Dx << " Dy " << Dy << endl;

        msg.data.clear();
        msg.data.push_back(2); // size of the vector
        msg.data.push_back(Ax); //0
        msg.data.push_back(Ay); //1
        /*msg.data.push_back(Bx); //2
        msg.data.push_back(By); //3
        msg.data.push_back(Cx); //4
        msg.data.push_back(Cy); //5
        msg.data.push_back(Dx); //6
        msg.data.push_back(Dy); //7
        */
      }

        dst.clear();

      }
    }

    setMouseCallback("Camera",on_mouse, NULL );

    //draw points in the template
    /*circle(imageCamera, vTemplate[0], 7, Scalar(0,0,255),-1); //BGR
    circle(imageCamera, vTemplate[1], 7, Scalar(0,0,255),-1);
    circle(imageCamera, vTemplate[2], 7, Scalar(0,0,255),-1);
    circle(imageCamera, vTemplate[3], 7, Scalar(0,0,255),-1);
*/
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
  gammax = 106.666667;
  gammay = 80;

  display = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png");
  //imageDesktop = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png", CV_LOAD_IMAGE_COLOR);

  // corners of the image to be projected
  vDisplay.push_back(Point2d(float(0),double(0)));
  vDisplay.push_back(Point2d(double(display.cols),double(0)));
  vDisplay.push_back(Point2d(double(display.cols),double(display.rows)));
  vDisplay.push_back(Point2d(double(0),double(display.rows)));

  //center
  vCenter.push_back(Point2d(double(0),double(0))); 

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


  //laser coordinates
  vLaser.push_back(Point2d(Ax,Ay));
  vLaser.push_back(Point2d(Bx,By));
  vLaser.push_back(Point2d(Cx,Cy));
  vLaser.push_back(Point2d(Dx,Dy));


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