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


#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 5

vector<Point2f> vSample, vSampleProj;  
vector<Point2f> vTemplate;  
vector<Point2f> vCamera;    
vector<Point2f> cornersCamera;
vector<Point2f> cornersTemplate;
Mat Hcal;
Mat imageCessBoard, imageCessBoardBW;
Mat imageCamera,imageCameraBW,imageSampleWarped;
bool token = true;
bool bTemplate = false;
cv_bridge::CvImagePtr cv_ptr;
Size board_size(CHESSBOARD_WIDTH-1, CHESSBOARD_HEIGHT-1);

Mat imageToBw (Mat img){

  Mat cpy_img(img.rows, img.cols, img.type());
  Mat neg_img(img.rows, img.cols, img.type());
  Mat gray;
  cvtColor(img, gray, CV_BGR2GRAY);
  if (!gray.empty())
      cout << "image gray converted\n";
        
  return  gray;     
    }

// void on_mouse( int e, int x, int y, int d, void *ptr )
// {
//     if (e == EVENT_LBUTTONDOWN )
//         cout << x << " "<< y <<endl;
     
// }

void calibration(){

    cout << "go for calibration...\n";
    imageCameraBW = imageToBw(imageCamera);                

    // Camera image is the chessboard projected (on the wall)
    bool bCamera = findChessboardCorners(imageCamera, board_size, cornersCamera);

    if (bCamera == 1){ //chessboard found

    cornerSubPix(imageCameraBW, cornersCamera, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

    // corners detected in the camera chess board
    vCamera.push_back(cornersCamera[0]);
    vCamera.push_back(cornersCamera[CHESSBOARD_WIDTH-2]);
    vCamera.push_back(cornersCamera[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
    vCamera.push_back(cornersCamera[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

    Hcal = findHomography(vTemplate, vCamera,0);            

    if(!Hcal.empty()){

    FileStorage fs("H_h.yml", FileStorage::WRITE);

    fs << "H_h" << Hcal;

    fs.release();

    cout << "witten H matrix now compute the scale\n";

    // Reproject a sample image to see if calibration is correct

    Mat imageSample = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png", CV_LOAD_IMAGE_COLOR);

    if(imageSample.empty())
    {
    cerr << "ERR: Unable to find overlay image.\n" << endl;

    }

    vSample.push_back(Point2f(0,0));
    vSample.push_back(Point2f(imageSample.cols,0));
    vSample.push_back(Point2f(imageSample.cols,imageSample.rows));
    vSample.push_back(Point2f(0,imageSample.rows));

    perspectiveTransform(vSample,vSampleProj, Hcal);

    // circle(imageCamera, vSampleProj[0], 7, Scalar(0,0,255),-1);
    // circle(imageCamera, vSampleProj[1], 7, Scalar(0,0,255),-1);
    // circle(imageCamera, vSampleProj[2], 7, Scalar(0,0,255),-1);
    // circle(imageCamera, vSampleProj[3], 7, Scalar(0,0,255),-1);

    // namedWindow( "Display window dots" );// Create a window
    // imshow( "Display window dots", imageCamera);


    Mat Hsar = findHomography(vSampleProj, vCamera, 0);
    warpPerspective(imageSample, imageSampleWarped, Hsar, imageSampleWarped.size()); // Transform sample Image to the position of the overlay
    imshow("Warped", imageSampleWarped);

    token = true;   
    }

    }

}


void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{
    if(token){

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
  
    // ROS_INFO("The Image has been saved");
    imshow( "Kinect's image",imageCamera);
    // setMouseCallback("Kinect's image",on_mouse, NULL );

    

}

}
 
int main(int argc, char** argv)
{

    ros::init(argc, argv, "receivedimagekinect");
    
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_rect_color", 3, receivedImage);
    ros::Subscriber image;

    // this is the image that has to be reconized (cessboard projected)
    imageCessBoard = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb652.png", CV_LOAD_IMAGE_GRAYSCALE);

    if( !imageCessBoard.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    else
    {
      std::cout<< " cb652.png ok " << std::endl;
    }

    // Template image is the cessboard stored, to be projected
    bTemplate = findChessboardCorners(imageCessBoard, board_size, cornersTemplate, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (bTemplate == 1){

    cout << "Founded corners in the template "<< cornersTemplate.size() << "\n";

    if(cornersTemplate.size() > 1)
    {
    cornerSubPix(imageCessBoard, cornersTemplate, Size(11,11), Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    cout << "Subpixel founded\n";
    }
      else
        return -1;

    // corners detected in the template cess board
    vTemplate.push_back(cornersTemplate[0]);
    vTemplate.push_back(cornersTemplate[CHESSBOARD_WIDTH-2]);
    vTemplate.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
    vTemplate.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

    cout << "Finded template corners...\n";
    cout << vTemplate;
    cout << "\n";
      
    }
    
    while(1){
    int key=waitKey(10);
    // printf("%d\n",key );
    
    if((char)key==27) break;

    if((char)key==32 && bTemplate == 1 ) //that means space -> go for calibration!
      {

      token = false;
      calibration();

      }
    ros::spinOnce();
    } 
    
 
    return 0;
}