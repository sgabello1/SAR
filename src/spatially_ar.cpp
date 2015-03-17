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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;


Mat imgScene, imgDesktop, img_reproject;
Mat sceneWarpedCal,sceneWarpedEs, imgSuperImposed;
cv_bridge::CvImagePtr cv_ptr;

bool token = true;

void showFinal(Mat src1,Mat src2)
{
 
    Mat gray,gray_inv,src1final,src2final;
    cvtColor(src2,gray,CV_BGR2GRAY);
    threshold(gray,gray,0,255,CV_THRESH_BINARY);
    //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4);
    bitwise_not ( gray, gray_inv );
    src1.copyTo(src1final,gray_inv);
    src2.copyTo(src2final,gray);
    Mat finalImage = src1final+src2final;
    namedWindow( "output", WINDOW_AUTOSIZE );
    imshow("output",finalImage);

    token = true;
 
}

void homography(){

  token = false;

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( imgScene, keypoints_object );
  detector.detect( imgDesktop, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( imgScene, keypoints_object, descriptors_object );
  extractor.compute( imgDesktop, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 1000;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  // printf("-- Max dist : %f \n", max_dist );
  // printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( imgScene, keypoints_object, imgDesktop, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  Mat H;

  if(obj.size() > 0 && scene.size() > 0)
  H = findHomography( obj, scene, CV_RANSAC );
  // ROS_INFO("H finded");

  Mat imgSuperWarped;
  warpPerspective(imgSuperImposed,imgSuperWarped,H,imgScene.size() );
  imshow( "Distorted",imgSuperWarped);

  showFinal(imgScene,imgSuperWarped);

  // -- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( imgDesktop.cols, 0 );
  obj_corners[2] = cvPoint( imgDesktop.cols, imgDesktop.rows ); obj_corners[3] = cvPoint( 0, imgDesktop.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  // //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( imgScene.cols, 0), scene_corners[1] + Point2f( imgScene.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( imgScene.cols, 0), scene_corners[2] + Point2f( imgScene.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( imgScene.cols, 0), scene_corners[3] + Point2f( imgScene.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( imgScene.cols, 0), scene_corners[0] + Point2f( imgScene.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );
 
            
  //warpPerspective(img_reproject,sceneWarped,H,img_reproject.size() );
  // warpPerspective(imgScene,imgSuperWarped,H,imgDesktop.size() );
  // imshow( "Perspective",sceneWarpedCal);
 
  // warpPerspective(img_reproject,sceneWarpedEs,H,imgDesktop.size() );
  // imshow( "Perspective Exemple",sceneWarpedEs);

 //showFinal(img_reproject,sceneWarped);
  
}


void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{

    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      imgScene=cv_ptr->image; //  scene viewed by the drone (background + desktop)

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    namedWindow( "Display window" );// Create a window
    imshow( "Display window",imgScene);

    // if(token)
    // homography();
    
}



/** @function main */
int main( int argc, char** argv )
{

  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("/ardrone/front/image_raw", 3, receivedImage);//  image_rect
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_rect_color", 3, receivedImage);
  ros::Subscriber image;

  imgDesktop = imread( "src/spatially_ar/src/imac-512.png", CV_LOAD_IMAGE_COLOR );// chessboard or image with lots of features or desktop
  imgSuperImposed = imread( "src/spatially_ar/src/superImposed.png", CV_LOAD_IMAGE_COLOR );// chessboard or image with lots of features or desktop
  
  if( !imgDesktop.data )
   { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

//  Press "Escape button" to exit
      while(1)
       {
          int key=waitKey(10);
          
          cout << key  << endl;

          if((char)key=='q') break;
          ros::spinOnce();
       }
  return 0;
  }

