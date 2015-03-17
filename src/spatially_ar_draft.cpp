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
vector<Point2f> straight_image,straight_image2,straight_image3;      // Stores 4 points(x,y) of the image. Here the four points are 4 corners of image.
vector<Point2f> from_drone_image,deperspective;    // stores 4 points that the user clicks(mouse left click) in the main image.
vector<Point2f> vPremap(4);
Mat H,H1,H2;
Mat imageRect;
Mat imageRect1;
Mat imageRect2;
Mat mRotated, Mat trans;


Mat imageDesktop(774, 1026, CV_8UC3);
Mat imageDesktop2(1200, 1226, CV_8UC3);
Mat drone_image(774, 1026, CV_8UC3);


Mat img_scene(1080, 1920, CV_8UC3);
Mat img_straight(1000, 1200, CV_8UC3);
// Mat img_reproject2(1500, 1500, CV_8UC3);

Mat img_logo,sceneWarpedEs;
cv_bridge::CvImagePtr cv_ptr;

bool token = true;
bool first = true;
 
 void rotateImage(const Mat &input, Mat &output, double alpha, double beta, double gamma, double dx, double dy, double dz, double f)
  {
    alpha = (alpha - 90.)*CV_PI/180.;
    beta = (beta - 90.)*CV_PI/180.;
    gamma = (gamma - 90.)*CV_PI/180.;
    // get width and height for ease of use in matrices
    double w = (double)input.cols;
    double h = (double)input.rows;
    // Projection 2D -> 3D matrix
    Mat A1 = (Mat_<double>(4,3) <<
              1, 0, -w/2,
              0, 1, -h/2,
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
              f, 0, w/2, 0,
              0, f, h/2, 0,
              0, 0,   1, 0);
    // Final transformation matrix
    trans = A2 * (T * (R * A1));
    // Apply matrix transformation
    warpPerspective(input, output, trans, imageDesktop.size(), INTER_LANCZOS4);
  }



// Here we get four points from the user with left mouse clicks.
// On 5th click it computes the homography -- remeber that is anti-clockwise
void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    
    {
        
        if(from_drone_image.size() < 4 )
        {
 
            from_drone_image.push_back(Point2f(float(x),float(y)));
            cout << x << " "<< y <<endl;
        }
        else
        {

            
            cout << " Calculating Homography " <<endl;
                        
            H1 = findHomography(from_drone_image,straight_image,0 ); // from rectified image to drone image
            
            // perspectiveTransform(straight_image,vPremap, H);
            // circle(drone_image, vPremap[0], 7, Scalar(0,0,255),-1);
            // circle(drone_image, vPremap[1], 7, Scalar(0,0,255),-1);
            // circle(drone_image, vPremap[2], 7, Scalar(0,0,255),-1);
            // circle(drone_image, vPremap[3], 7, Scalar(0,0,255),-1);

            imshow("Display window", drone_image);

            // Warp the  image to rectify the scene and check if everything is correct

            // rotateImage(orignalImage, outputImage, 90, 135, 90, 200, 200);
            rotateImage(imageDesktop, mRotated, 90, 45, 90,0,0,200, 200);
            cv::imshow("rotated on y of 45°",mRotated);

            warpPerspective(mRotated,imageRect,H1,imageDesktop.size());  // warp for the drone's view
            
            cv::imshow("Rectified",imageRect);
            setMouseCallback("Rectified",on_mouse, NULL );

            cout << " Ok. Now the image to project is distorted " <<endl;
            first = false;
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
      drone_image=cv_ptr->image; //  scene viewed by the drone (background + desktop)

      setMouseCallback("Display window",on_mouse, NULL );
    setMouseCallback("Straight",on_mouse, NULL );

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    namedWindow( "Display window" );// Create a window
    imshow( "Display window", drone_image);
    // imshow("Straight", img_straight);

    
    }
}
 
int main( int argc, char** argv )
{


    // this is the image that has to be seen by the observer straight
    img_straight = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/straight_drone_pic.png", CV_LOAD_IMAGE_COLOR);
    imageDesktop = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/toproj.png", CV_LOAD_IMAGE_COLOR);

    // Push the 4 corners of the logo image as the 4 points for correspondence to calculate homography.
    straight_image.push_back(Point2f(float(200), float(42))); // A
    straight_image.push_back(Point2f(float(195), float(238))); //B
    straight_image.push_back(Point2f(float(422), float(238))); // C
    straight_image.push_back(Point2f(float(418), float(47))); // D

    // Desktop image
    straight_image3.push_back(Point2f(float(0), float(0))); // A
    straight_image3.push_back(Point2f(float(0), float(imageDesktop.rows))); //B
    straight_image3.push_back(Point2f(float(imageDesktop.cols), float(imageDesktop.rows))); // C
    straight_image3.push_back(Point2f(float(imageDesktop.cols), float(0))); // D
    
    // drone image
    straight_image2.push_back(Point2f(float(0), float(0))); // A
    straight_image2.push_back(Point2f(float(0), float(drone_image.rows))); //B
    straight_image2.push_back(Point2f(float(drone_image.cols), float(drone_image.rows))); // C
    straight_image2.push_back(Point2f(float(drone_image.cols), float(0))); // D

 
    setMouseCallback("Display window",on_mouse, NULL );
    setMouseCallback("Straight",on_mouse, NULL );

 
    ros::init(argc, argv, "listener");
  
    ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("/ardrone/front/image_raw", 3, receivedImage);//  image_rect
    ros::Subscriber sub = n.subscribe("/image_rect", 3, receivedImage);
    ros::Subscriber image;

    if( !imageDesktop.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
 
    //  Press "Escape button" to exit
      while(1)
       {
          int key=cvWaitKey(10);
          if(key==27) break;

        
          if(  key==32 && !token) //that means space -> that is ok the homography, so save it!
            {
            
            // FileStorage fs("H_h.yml", FileStorage::WRITE);

            // fs << "H_h" << H;

            // fs.release();

            // cout << "witten H matrix now compute the scale\n";

            // setMouseCallback("Rectified",scale, NULL );

            token = true; //ya gimme another image and reset points for right image

            from_drone_image.clear();

            }

        if(key==100 && !token) //that means d -> continue!
            {

            cout << "delete homography\n";
            
            token = true; //ya gimme another image and reset points for right image

            from_drone_image.clear();

            }
          ros::spinOnce();
       }
     
 
    return 0;
}