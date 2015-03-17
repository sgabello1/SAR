/**
 * Automatic perspective correction for quadrilateral objects. See the tutorial at
 * http://opencv-code.com/tutorials/automatic-perspective-correction-for-quadrilateral-objects/
 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

cv::Point2f center(0,0);

cv::Point2f computeIntersect(cv::Vec4i a, 
                             cv::Vec4i b)
{
  int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
  float denom;

  if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
  {
    cv::Point2f pt;
    pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
    pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
    return pt;
  }
  else
    return cv::Point2f(-1, -1);
}



cv::Point2f computeIntersect2(cv::Vec4i a, 
                             cv::Vec4i b ) //cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,
                     
{
  cv::Point2f o1(a[0],a[1]);
   cv::Point2f p1(a[2],a[3]);
  cv::Point2f o2(b[0],b[1]); cv::Point2f p2(b[2],b[3]);


    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;
  cv::Point2f r;
    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return cv::Point2f(-1, -1);

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return r;
}



void sortCorners(std::vector<cv::Point2f>& corners, 
                 cv::Point2f center)
{
  std::vector<cv::Point2f> top, bot;

  for (int i = 0; i < corners.size(); i++)
  {
    if (corners[i].y < center.y)
      top.push_back(corners[i]);
    else
      bot.push_back(corners[i]);
  }
  corners.clear();
  
  if (top.size() == 2 && bot.size() == 2){
    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];
  
    
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);
  }
}


bool near(int a,int b,int tol)
{
  if(abs(a-b)<tol) return true; else return false;

}
int main()
{
  
  cv::VideoCapture cap(1);
  

  cv::Mat src;

  cap >> src; cap >> src;

  while(1)
{
  cap >> src;
  if (src.empty())
    return -1;

  cv::Mat bw;
  cv::cvtColor(src, bw, CV_BGR2GRAY);
  cv::blur(bw, bw, cv::Size(3, 3));
  cv::Canny(bw, bw, 30, 30, 3);
  cv::imshow("canny", bw);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(bw, lines, 1, CV_PI/180, 70, 30, 10);
  cv::Mat dst = src.clone();


  // Expand the lines
  for (int i = 0; i < lines.size(); i++)
  {
    cv::Vec4i v = lines[i];
    lines[i][0] = 0;
    lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1]; 
    lines[i][2] = src.cols; 
    lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (src.cols - v[2]) + v[3];

    cv::line(dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2],lines[i][3]), CV_RGB(100,0,0));

  }

// remove similar segments
    for (int i = 0; i < lines.size(); i++)
  {
    cv::Vec4i v2 = lines[i];

    for (int j = 0; j < lines.size(); j++)
      if(j!=i)
    {
      cv::Vec4i v = lines[j];
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
    cv::line(dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2],lines[i][3]), CV_RGB(255,0,0),4);
  }




    cv::imshow("image", dst);


  // std::cout << "number of lines: " << lines.size() << std::endl;
  std::vector<cv::Point2f> corners;
  corners.clear();
  for (int i = 0; i < lines.size(); i++)
  {
    for (int j = i+1; j < lines.size(); j++)
    {
      cv::Point2f pt = computeIntersect2(lines[i], lines[j]);
      if (pt.x >= 0 && pt.y >= 0)
        {corners.push_back(pt);
          cv::circle(dst, pt, 5, CV_RGB(255,255,0), 2);
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

  std::vector<cv::Point2f> approx;
  if(corners.size()==0) {cv::waitKey(10); continue;}
  cv::approxPolyDP(cv::Mat(corners), approx, cv::arcLength(cv::Mat(corners), true) * 0.02, true);

  if (approx.size() != 4)
  {
    // std::cout << "The object is not quadrilateral!" << std::endl; cv::waitKey(10);

    continue;
  }
  
  // Get mass center
  for (int i = 0; i < corners.size(); i++)
    center += corners[i];
  center *= (1. / corners.size());

  sortCorners(corners, center);
  if (corners.size() == 0){
    std::cout << "The corners were not sorted correctly!" << std::endl;
      cv::waitKey(10);

    continue;
  }

  // Draw lines
  for (int i = 0; i < lines.size(); i++)
  {
    cv::Vec4i v = lines[i];
    cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0));
  }

  // Draw corner points
  cv::circle(dst, corners[0], 3, CV_RGB(255,0,0), 2);
  cv::circle(dst, corners[1], 3, CV_RGB(0,255,0), 2);
  cv::circle(dst, corners[2], 3, CV_RGB(0,0,255), 2);
  cv::circle(dst, corners[3], 3, CV_RGB(255,255,255), 2);

  // Draw mass center
  cv::circle(dst, center, 3, CV_RGB(255,255,0), 2);

  cv::Mat quad = cv::Mat::zeros(300, 220, CV_8UC3);

  std::vector<cv::Point2f> quad_pts;
  quad_pts.push_back(cv::Point2f(0, 0));
  quad_pts.push_back(cv::Point2f(quad.cols, 0));
  quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
  quad_pts.push_back(cv::Point2f(0, quad.rows));

  cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);
  cv::warpPerspective(src, quad, transmtx, quad.size());

  cv::imshow("image", dst);
  cv::imshow("quadrilateral", quad);
  cv::waitKey(10);
}
  return 0;
}

