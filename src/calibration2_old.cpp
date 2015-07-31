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
vector<Point2f> left_image;      // Stores 4 points(x,y) of the image. Here the four points are 4 corners of image.
vector<Point2f> right_image;    // stores 4 points that the user clicks(mouse left click) in the main image.
vector<Point2f> point_scale;
Mat H;
Mat imageDesktop(774, 1026, CV_8UC3);

Mat img_scene(1080, 1920, CV_8UC3);
Mat img_reproject(1000, 1200, CV_8UC3);
Mat img_reproject2(1500, 1500, CV_8UC3);

Mat img_logo,sceneWarpedEs;
cv_bridge::CvImagePtr cv_ptr;

bool token = true;
 
 
// Here we get four points from the user with left mouse clicks.
// On 5th click it computes the homography -- remeber that is anti-clockwise
void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
        if(right_image.size() < 4 )
        {
 
            right_image.push_back(Point2f(float(x),float(y)));
            cout << x << " "<< y <<endl;
        }
        else
        {
            cout << " Calculating Homography " <<endl;
                        
            Mat imageRect;
            H = findHomography(left_image,right_image,0 ); // from little image to a bigger rectified

            // Warp the  image to rectify the scene and check if everything is correct
            warpPerspective(imageDesktop,imageRect,H,imageDesktop.size());
            cv::imshow("Rectified",imageRect);

            token = false;

            cout << " Calculated Homography..wanna keep it? " <<endl;
            

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
      img_scene=cv_ptr->image; //  scene viewed by the drone (background + desktop)

      setMouseCallback("Display window",on_mouse, NULL );

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    namedWindow( "Display window" );// Create a window
    imshow( "Display window", img_scene);

    
    }
}
 
int main( int argc, char** argv )
{

      VideoCapture capture(1);


    // this is the image that has to be reconized (desktop projected)
    imageDesktop = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png", CV_LOAD_IMAGE_COLOR);

    // Push the 4 corners of the logo image as the 4 points for correspondence to calculate homography.
    left_image.push_back(Point2f(float(0),float(0)));
    left_image.push_back(Point2f(float(0),float(imageDesktop.rows)));
    left_image.push_back(Point2f(float(imageDesktop.cols),float(imageDesktop.rows)));
    left_image.push_back(Point2f(float(imageDesktop.cols),float(0)));
 
    setMouseCallback("Display window",on_mouse, NULL );
 
    ros::init(argc, argv, "listener");
  
    ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("/ardrone/front/image_raw", 3, receivedImage);//  image_rect
    ros::Subscriber sub = n.subscribe("/image_raw", 3, receivedImage);
    ros::Subscriber image;

    if( !imageDesktop.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
 
    //  Press "Escape button" to exit
      while(1)
       {

          capture >> img_scene;
          int key=cvWaitKey(10);
          if(key==27) break;

          if(key==32 && !token) //that means space -> that is ok the homography, so save it!
            {
            
            FileStorage fs("H_h.yml", FileStorage::WRITE);

            fs << "H_h" << H;

            fs.release();

            cout << "witten H matrix now compute the scale\n";

            // setMouseCallback("Rectified",scale, NULL );

            token = true; //ya gimme another image and reset points for right image

            right_image.clear();

            }

        if(key==100 && !token) //that means d -> continue!
            {

            cout << "delete homography\n";
            
            token = true; //ya gimme another image and reset points for right image

            right_image.clear();

            }
          ros::spinOnce();

          namedWindow( "Display window" );// Create a window
          imshow( "Display window", img_scene);
          setMouseCallback("Display window",on_mouse, NULL );

       }
     
 
    return 0;
}