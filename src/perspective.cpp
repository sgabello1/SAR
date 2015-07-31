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

int main( int argc, char** argv )
{

Mat H ; 
FileStorage fs("H.yml",FileStorage::READ);
fs["H"] >> H;
fs.release();

if( !H.data )
   { std::cout<< " --(!) Error reading H " << std::endl; return -1; }

Mat logo = imread( "src/spatially_ar/src/logo1.jpg", CV_LOAD_IMAGE_GRAYSCALE ); // coca cola logo w 462 260 h

if( !logo.data )
   { std::cout<< " --(!) Error reading image logo " << std::endl; return -1; }

Mat sceneWarpedEs;
//Width: 1022 pixels Height: 774 pixels
Mat  img_scene(400, 400, CV_8UC3);;//imread( "src/spatially_ar/src/hipster.png", CV_LOAD_IMAGE_GRAYSCALE );// chessboard or image with lots of features or desktop

if( !img_scene.data )
   { std::cout<< " --(!) Error reading image img_scene " << std::endl; return -1; }

warpPerspective(logo,sceneWarpedEs,H,img_scene.size() );
imshow( "sceneWarpedEs",sceneWarpedEs);
imshow( "original",logo);

cvWaitKey(0);


return 0;
}

  