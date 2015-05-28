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

// Taken by sar_ye2 stupid demo to test the homography

using namespace cv;

geometry_msgs::Polygon square;
bool token = true;
Mat imageCamera, imageCrop;
cv_bridge::CvImagePtr cv_ptr;
vector<Point2f> redPoint;
int Xpoints[3],Ypoints[3];
Mat imageToPrj,imageSampleWarped, imageToPrjResized;
vector<Point2f> vSampleProj;  
Mat Har;
double zoom = 1;

void SquareARMarkers(const geometry_msgs::Polygon square)
{

  if(!imageCamera.empty()){
    // ROS_INFO("Square: 3 Point (%f , %f) ", square.points[2].x, square.points[2].y);
    
    redPoint.push_back(Point2f(square.points[0].x, square.points[0].y)); //M1
    redPoint.push_back(Point2f(square.points[1].x, square.points[1].y)); //M2
    redPoint.push_back(Point2f(square.points[2].x, square.points[2].y));
    redPoint.push_back(Point2f(square.points[3].x, square.points[3].y));

    circle(imageCamera, redPoint[0], 7, Scalar(0,0,255),-1); //red
    circle(imageCamera,  redPoint[1], 7, Scalar(0,255,0),-1); //blue
    circle(imageCamera,  redPoint[2], 7, Scalar(255,0,0),-1); //green
    circle(imageCamera,  redPoint[3], 7, Scalar(255,0,255),-1); //violet

    namedWindow( "Kinect's image" );// Create a window
    imshow( "Kinect's image",imageCamera);

    // warping
    if(square.points[0].z >= 4 ){ // hopefully markers are 4

      Har = findHomography(vSampleProj,redPoint, 0);
      warpPerspective(imageToPrj, imageSampleWarped, Har, imageSampleWarped.size()); // Transform sample Image to the position of the overlay
      imshow("Warped", imageSampleWarped);
      resize(imageSampleWarped, imageToPrjResized, cvSize(0, 0), zoom, zoom, INTER_NEAREST);

      imshow("Easy", imageToPrjResized);


    }



    redPoint.clear();
  } 
}

void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{

    int key=cvWaitKey(10);

    if((char) key == '+'){

      zoom++;

      ROS_INFO("more.. %f", zoom);


    }
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      imageCamera=cv_ptr->image; // kinect
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
     
    
// }

}

int main(int argc, char **argv)
{

  imageToPrj = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb3.png", CV_LOAD_IMAGE_COLOR);

  // 
  vSampleProj.push_back(Point2f(0,0));
  vSampleProj.push_back(Point2f(imageToPrj.cols,0)); //imageSample.cols,0)
  vSampleProj.push_back(Point2f(imageToPrj.cols,imageToPrj.rows)); //imageSample.cols,imageSample.rows
  vSampleProj.push_back(Point2f(0,imageToPrj.rows)); //0,imageSample.rows

  // vSampleProj.push_back(Point2f(210,67));
  // vSampleProj.push_back(Point2f(209,251)); //imageSample.cols,0)
  // vSampleProj.push_back(Point2f(439,253)); //imageSample.cols,imageSample.rows
  // vSampleProj.push_back(Point2f(442,73)); //0,imageSample.rows


  ros::init(argc, argv, "spatialar");

  ros::NodeHandle n;

  ros::Subscriber kfsub = n.subscribe("/square", 1, SquareARMarkers);
  ros::Subscriber sub_image = n.subscribe("/camera/image_color", 3, receivedImage);
  
  ros::spin();

  return 0;
}
