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
//#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/nonfree/features2d.hpp>


#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;


Mat img_object, img_scene(1080, 1920, CV_8UC3), img_reproject;
Mat sceneWarpedCal,sceneWarpedEs;
cv_bridge::CvImagePtr cv_ptr;


void readme();


void showFinal(Mat src1,Mat src2)
{
 
     Mat gray,gray_inv,src1final,src2final,src2_inv;
    // cvtColor(src2,gray,CV_BGR2GRAY);
    // threshold(gray,gray,0,255,CV_THRESH_BINARY);
    //     ROS_INFO("3");

    // //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4);
    bitwise_not ( src2, src2_inv );
    src1.copyTo(src1final,src2_inv);
    src2.copyTo(src2final,src2);
    Mat finalImage = src1final+src2final;
    namedWindow( "output", WINDOW_AUTOSIZE );
    imshow("output",finalImage);
    
    //cvWaitKey(0);
 
}

void writeMatToFile(Mat& m, const char* filename)
{
    ofstream fout(filename);

    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout<<m.at<float>(i,j)<<"\t";
        }
        fout<<endl;
    }

    fout.close();
}

void homography(){

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
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

  Mat H = findHomography( obj, scene, CV_RANSAC );

    
  // const char* filename = "H.txt";

  // writeMatToFile(H,filename);
 
    FileStorage fs("H_h.yml", FileStorage::WRITE);

    fs << "H_h" << H;

    fs.release();

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  // //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  //imshow( "Good Matches & Object detection", img_matches );
 
            
  //warpPerspective(img_reproject,sceneWarped,H,img_reproject.size() );
  warpPerspective(img_object,sceneWarpedCal,H,img_scene.size() );
  imshow( "Perspective",sceneWarpedCal);
 
  warpPerspective(img_reproject,sceneWarpedEs,H,img_scene.size() );
  imshow( "Perspective Exemple",sceneWarpedEs);

 //showFinal(img_reproject,sceneWarped);
  cvWaitKey(0);

}


void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{

  ROS_INFO("Receive an Image from AR drone");

    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      img_object=cv_ptr->image; // drone
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  //img_object = img->data;

  ROS_INFO("The Image has been saved");

  imshow( "Drone's image",img_object);

  homography();

}



/** @function main */
int main( int argc, char** argv )
{

  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("/ardrone/front/image_raw", 3, receivedImage);//  image_rect
  ros::Subscriber sub = n.subscribe("/image_rect", 3, receivedImage);
  ros::Subscriber image;


  //Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  //Mat img_scene = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
  img_scene = imread( "src/spatially_ar/src/drone_image_straight2.png", CV_LOAD_IMAGE_GRAYSCALE );// chessboard or image with lots of features or desktop
  img_reproject = imread( "src/spatially_ar/src/logo1.jpg", CV_LOAD_IMAGE_GRAYSCALE ); // desktop

  if( !img_scene.data )
   { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  ros::spin();

  return 0;
  }

  /** @function readme */
  void readme()
  { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
