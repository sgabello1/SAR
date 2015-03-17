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


using namespace cv;

geometry_msgs::Polygon square;
bool token = true;
Mat imageCamera, imageCrop;
cv_bridge::CvImagePtr cv_ptr;
vector<Point2f> redPoint;
int Xpoints[3],Ypoints[3];
int minX,maxX,minY,maxY;
cv::Point2f center(0,0);


int maximumValue(int arrayx[3])
{
     int length = 4;  // establish size of array
     int max = arrayx[0];       // start with max = first element

     for(int i = 0; i<length; i++)
     {
          if(arrayx[i] > max)
                max = arrayx[i];
     }
     return max;                // return highest value in array
}

int minimumValue(int array[3])
{
     int length = 4;  // establish size of array
     int min = array[0];       // start with max = first element

     for(int i = 0; i<length; i++)
     {
          if(array[i] < min)
                min = array[i];
     }
     return min;                // return highest value in array
}

bool near(int a,int b,int tol)
{
  if(abs(a-b)<tol) return true; else return false;

}

Point2f computeIntersect2(Vec4i a, 
                             Vec4i b ) 
                     
{
  Point2f o1(a[0],a[1]);
   Point2f p1(a[2],a[3]);
  Point2f o2(b[0],b[1]); Point2f p2(b[2],b[3]);


    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;
  Point2f r;
    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return Point2f(-1, -1);

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return r;
}


void sortCorners(std::vector<Point2f>& corners, 
                 Point2f center)
{
  std::vector<Point2f> top, bot;

  for (int i = 0; i < corners.size(); i++)
  {
    if (corners[i].y < center.y)
      top.push_back(corners[i]);
    else
      bot.push_back(corners[i]);
  }
  corners.clear();
  
  if (top.size() == 2 && bot.size() == 2){
    Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
  
    
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);
  }
}

int findRect(){

  Mat bw;
  cvtColor(imageCrop, bw, CV_BGR2GRAY);
  blur(bw, bw, Size(3, 3));
  Canny(bw, bw, 30, 30, 3);
  imshow("canny", bw);

  std::vector<Vec4i> lines;
  HoughLinesP(bw, lines, 1, CV_PI/180, 70, 30, 10);
  Mat dst = imageCrop.clone();


  // Expand the lines
  for (int i = 0; i < lines.size(); i++)
  {
    Vec4i v = lines[i];
    lines[i][0] = 0;
    lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1]; 
    lines[i][2] = imageCrop.cols; 
    lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (imageCrop.cols - v[2]) + v[3];

    line(dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2],lines[i][3]), CV_RGB(100,0,0));

  }

// remove similar segments
    for (int i = 0; i < lines.size(); i++)
  {
    Vec4i v2 = lines[i];

    for (int j = 0; j < lines.size(); j++)
      if(j!=i)
    {
      Vec4i v = lines[j];
      int tol=40;
      if(
      near(v[0],v2[0],tol) && near(v[1],v2[1],tol) &&  near(v[2],v2[2],tol) && near(v[3],v2[3],tol) ||
      near(v[0],v2[2],tol) && near(v[1],v2[3],tol) &&  near(v[2],v2[0],tol) && near(v[3],v2[1],tol)

      )
      {
        //std::cout << "Removed line\n";
        lines.erase(lines.begin()+j);
      }
      
    }

  }

  // draw the lines (debug)
  for (int i = 0; i < lines.size(); i++)
  {
    line(dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2],lines[i][3]), CV_RGB(255,0,0),4);
  }




    imshow("image", dst);


  // std::cout << "number of lines: " << lines.size() << std::endl;
  std::vector<Point2f> corners;
  corners.clear();
  for (int i = 0; i < lines.size(); i++)
  {
    for (int j = i+1; j < lines.size(); j++)
    {
      Point2f pt = computeIntersect2(lines[i], lines[j]);
      if (pt.x >= 0 && pt.y >= 0)
        {corners.push_back(pt);
          circle(dst, pt, 5, CV_RGB(255,255,0), 2);
        }
    }
  }

  for(int i=0;i<corners.size();i++)
  {
      for(int j=0;j<corners.size();j++)
        if(i!=j &&
          near(corners[i].x,corners[j].x,10) && near(corners[i].y,corners[j].y,10))
            corners.erase(corners.begin()+j);


  }
    std::cout << "number of corners: " << corners.size() << std::endl;

  std::vector<Point2f> approx;
  if(corners.size()==0) 
  {
  	return -1;
  }
  approxPolyDP(Mat(corners), approx, arcLength(Mat(corners), true) * 0.02, true);

  if (approx.size() != 4)
  {
   std::cout << "The object is not quadrilateral!" << std::endl;

   return -1;

  }
  
  // Get mass center
  for (int i = 0; i < corners.size(); i++)
    center += corners[i];
  center *= (1. / corners.size());

  sortCorners(corners, center);
  if (corners.size() == 0){
    std::cout << "The corners were not sorted correctly!" << std::endl;

    return -1;
  }

  // Draw lines
  for (int i = 0; i < lines.size(); i++)
  {
    Vec4i v = lines[i];
    line(dst, Point(v[0], v[1]), Point(v[2], v[3]), CV_RGB(0,255,0));
  }

  // Draw corner points
  circle(dst, corners[0], 3, CV_RGB(255,0,0), 2);
  circle(dst, corners[1], 3, CV_RGB(0,255,0), 2);
  circle(dst, corners[2], 3, CV_RGB(0,0,255), 2);
  circle(dst, corners[3], 3, CV_RGB(255,255,255), 2);

  // Draw mass center
  circle(dst, center, 3, CV_RGB(255,255,0), 2);

  Mat quad = Mat::zeros(300, 220, CV_8UC3);

  std::vector<Point2f> quad_pts;
  quad_pts.push_back(Point2f(0, 0));
  quad_pts.push_back(Point2f(quad.cols, 0));
  quad_pts.push_back(Point2f(quad.cols, quad.rows));
  quad_pts.push_back(Point2f(0, quad.rows));

  Mat transmtx = getPerspectiveTransform(corners, quad_pts);
  warpPerspective(imageCrop, quad, transmtx, quad.size());

  imshow("image", dst);
  imshow("quadrilateral", quad);
  waitKey(10);

  return 0;
};

void SquareARMarkers(const geometry_msgs::Polygon square)
{

	if(!imageCamera.empty()){
    // ROS_INFO("Square: 3 Point (%f , %f) ", square.points[2].x, square.points[2].y);

	for(int i=0;i<4;i++)
	{
		Xpoints[i] = (int) square.points[i].x;
		Ypoints[i] = (int) square.points[i].y;
	}

	maxX = maximumValue(Xpoints);
	maxY = maximumValue(Ypoints);

	minX = minimumValue(Xpoints);
	minY = minimumValue(Ypoints);

	// std::cout << " minx "<< minX << "  maxX " << maxX << "\n";
	// std::cout << " miny "<< minY << "  maxY " << maxY << "\n";

	redPoint.push_back(Point2f(square.points[0].x, square.points[0].y));
	redPoint.push_back(Point2f(square.points[1].x, square.points[1].y));
	redPoint.push_back(Point2f(square.points[2].x, square.points[2].y));
	redPoint.push_back(Point2f(square.points[3].x, square.points[3].y));

    circle(imageCamera, redPoint[0], 7, Scalar(0,0,255),-1); //red
    circle(imageCamera,  redPoint[1], 7, Scalar(0,255,0),-1); //blue
    circle(imageCamera,  redPoint[2], 7, Scalar(255,0,0),-1); //green
    circle(imageCamera,  redPoint[3], 7, Scalar(255,0,255),-1); //violet

    if((int) square.points[0].z >= 4){
    Rect roi(Point(minX,minY),Point(maxX,maxY));

    if(maxX < imageCamera.rows && maxY < imageCamera.cols){
    ROS_INFO("yo");
    imageCrop = imageCamera(roi).clone();

    // namedWindow( "Crop" );// Create a window
    // imshow( "Crop",imageCrop); 

    findRect();
    }
	}

	  namedWindow( "Kinect's image" );// Create a window
    imshow( "Kinect's image",imageCamera);
    redPoint.clear();
	}	
}

void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{
	int key=cvWaitKey(10);

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
  
  ros::init(argc, argv, "spatialar");

  ros::NodeHandle n;

  ros::Subscriber kfsub = n.subscribe("/square", 1, SquareARMarkers);
  ros::Subscriber sub_image = n.subscribe("/camera/rgb/image_rect_color", 3, receivedImage);
  
  ros::spin();

  return 0;
}
