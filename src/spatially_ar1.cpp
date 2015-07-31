#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <sstream>
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <limits>
#include <numeric>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <limits>
#include <numeric>

using namespace cv;
using namespace std;
 
// We need 4 corresponding 2D points(x,y) to calculate homography.
vector<Point2f> vStraight,vStraight45, vStraightAffine;      // Stores 4 points(x,y) of the image. Here the four points are 4 corners of image.
vector<Point2f> vDrone,vDrone2;    // stores 4 points that the user clicks(mouse left click) in the main image.
vector<Point2f> vRotatedPlane;
Mat A( 2, 3, CV_32FC1 ), H, H2;
// Mat imageRect;
// Mat imageRect1;
// Mat imageRect2;
Mat trans;


Mat imgToproject(774, 1026, CV_8UC3);
Mat imgToproject0(774, 1026, CV_8UC3);
Mat imgToproject1(774, 1026, CV_8UC3);

Mat imgDrone(774, 1026, CV_8UC3);
Mat imgStraight(1000, 1200, CV_8UC3);
Mat imgRotated, imgRotated1, imgRotated2;

cv_bridge::CvImagePtr cv_ptr;

bool token = true;
bool first = true;
 
 void rotateImage(const Mat &input, Mat &output, double alpha, double beta, double gamma, double dx, double dy, double dz, double f)
  {
    // [562.8848951932215, 0, 308.5376677969403, 0, 560.3996970461961, 179.4825534829967, 0, 0, 1]
    double cx = 308.5376677969403;
    double cy = 179.4825534829967;
    double fx = 562.8848951932215;
    double fy = 560.3996970461961;
    
    alpha = (alpha - 90.)*CV_PI/180.;
    beta = (beta - 90.)*CV_PI/180.;
    gamma = (gamma - 90.)*CV_PI/180.;
    // get width and height for ease of use in matrices
    double w = (double)input.cols;
    double h = (double)input.rows;
    // Projection 2D -> 3D matrix
    Mat A1 = (Mat_<double>(4,3) <<
              1/fx, 0, -cx/fx,
              0, 1/fy, -cy/fy,
              0, 0,    0,
              0, 0,    1);
    // Rotation matrices around the X, Y, and Z axis
    Mat RX = (Mat_<double>(4, 4) <<
              1,          0,           0, 0,
              0, cos(alpha), -sin(alpha), 0,
              0, sin(alpha),  cos(alpha), 0,
              0,          0,           0, 1);
    Mat RY = (Mat_<double>(4, 4) <<
              cos(beta), 0, -sin(beta), 0,
              0, 1,          0, 0,
              sin(beta), 0,  cos(beta), 0,
              0, 0,          0, 1);
    Mat RZ = (Mat_<double>(4, 4) <<
              cos(gamma), -sin(gamma), 0, 0,
              sin(gamma),  cos(gamma), 0, 0,
              0,          0,           1, 0,
              0,          0,           0, 1);
    // Composed rotation matrix with (RX, RY, RZ)
    Mat R = RX * RY * RZ;
    // Translation matrix
    Mat T = (Mat_<double>(4, 4) <<
             1, 0, 0, dx,
             0, 1, 0, dy,
             0, 0, 1, dz,
             0, 0, 0, 1);
    // 3D -> 2D matrix
    Mat A2 = (Mat_<double>(3,4) <<
              fx, 0, cx, 0,
              0, fy, cy, 0,
              0, 0,   1, 0);
    // Final transformation matrix
    trans = A2 * (T * (R * A1));
    // Apply matrix transformation
    warpPerspective(input, output, trans, imgToproject.size(), INTER_LANCZOS4);
  }



// Here we get four points from the user with left mouse clicks.
// On 5th click it computes the homography -- remeber that is anti-clockwise
void on_mouse( int e, int x, int y, int d, void *ptr )
{
  if (e == EVENT_LBUTTONDOWN )  
  {
    if(first)
    {
        
        if(vDrone.size() < 4 )
        {
 
            vDrone.push_back(Point2f(float(x),float(y)));
            cout << x << " "<< y <<endl;
        }
        else
        {

            // calculating homography
          vStraight45.push_back(Point2f(float(199), float(257))); // A
    vStraight45.push_back(Point2f(float(193), float(868))); //B
    vStraight45.push_back(Point2f(float(1388), float(738))); // C
    vStraight45.push_back(Point2f(float(1383), float(148))); // D

    vStraight.push_back(Point2f(float(576), float(133))); // A
    vStraight.push_back(Point2f(float(579), float(726))); //B
    vStraight.push_back(Point2f(float(1358), float(722))); // C
    vStraight.push_back(Point2f(float(1356), float(128))); // D
              
            H = findHomography(vStraight45,vStraight,0 ); // from drone image to rectified image
            warpPerspective(imgToproject0, imgToproject, H, imgToproject.size() );
            cv::imshow("Homography",imgToproject);

            setMouseCallback("Homography",on_mouse, NULL );
          
          cout << " ok first image\n";
            first = false;
            vDrone.clear();

        }
      }
        else
        {
          if(vDrone2.size() < 4 )
          {

            vDrone2.push_back(Point2f(float(x),float(y)));
            cout << x << " "<< y <<endl;

          }else{

            // vStraightAffine.push_back(Point2f(float(0), float(0))); // A
            // vStraightAffine.push_back(Point2f(float(0), float(imgToproject.rows))); //B
            // vStraightAffine.push_back(Point2f(float(imgToproject.cols), float(imgToproject.rows))); // C
            
             // calculating homography 2
              
            H2 = findHomography(vDrone2,vDrone,0 ); // from drone image to rectified image
            warpPerspective(imgToproject, imgToproject1, H2, imgToproject.size() );
            cv::imshow("Homography 1",imgToproject1);

            setMouseCallback("Homography",on_mouse, NULL );
            
            first = false;
          }
        }

    
        
  }  
}

 
void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{

    if(token){
  // ROS_INFO("Receive an Image from AR drone");

    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      imgDrone=cv_ptr->image; //  scene viewed by the drone (background + desktop)

    setMouseCallback("Display window",on_mouse, NULL );
    // setMouseCallback("rotated on y +45°",on_mouse, NULL );


    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    namedWindow( "Display window" );// Create a window
    imshow( "Display window",imgDrone);

    // rotate the image to project by 45 on y axis 
    // rotateImage(imgToproject0, imgRotated2, 90, 120, 90,0,0,200, 200); 
    // cv::imshow("rotated of -30°",imgRotated2);
    // cv::imshow("straighgt",imgToproject0);

            
    }
}
 
int main( int argc, char** argv )
{


    // this is the image that has to be seen by the observer straight
    imgStraight = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/straight_drone_pic.png", CV_LOAD_IMAGE_COLOR);
    imgToproject = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/toproj.png", CV_LOAD_IMAGE_COLOR);
    imgToproject0 = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/toproj.png", CV_LOAD_IMAGE_COLOR);

    // Push the 4 corners of the logo image as the 4 points for correspondence to calculate homography.
    // vStraight.push_back(Point2f(float(576), float(133))); // A
    // vStraight.push_back(Point2f(float(579), float(726))); //B
    // vStraight.push_back(Point2f(float(1358), float(722))); // C
    // vStraight.push_back(Point2f(float(1356), float(128))); // D



    setMouseCallback("Display window",on_mouse, NULL );
    setMouseCallback("rotated on y +45°",on_mouse, NULL );

 
    ros::init(argc, argv, "listener");
  
    ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("/ardrone/front/image_raw", 3, receivedImage);//  image_rect
    ros::Subscriber sub = n.subscribe("/image_mono", 3, receivedImage);
    ros::Subscriber image;

    if( !imgToproject.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
 
    //  Press "Escape button" to exit
      while(1)
       {
          int key=cvWaitKey(10);
          if(key==27) break;

        
          if(key==32 && !token) //that means space -> that is ok the homography, so save it!
            {
            
            token = true; //ya gimme another image and reset points for right image

            vDrone.clear();

            }

          ros::spinOnce();
       }
     
 
    return 0;
}