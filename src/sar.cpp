//______________________________________________________________________________________
// Program : SimplAR 2 - OpenCV Simple Augmented Reality Program with Chessboard
// Author  : Bharath Prabhuswamy
//______________________________________________________________________________________

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define CHESSBOARD_WIDTH 6
#define CHESSBOARD_HEIGHT 5
//The pattern actually has 6 x 5 squares, but has 5 x 4 = 20 'ENCLOSED' corners

vector<Point2f> vClick,vDisplay, vTransformed;    // stores 4 points that the user clicks(mouse left click) in the main image.
Mat img,displayWarped,displayWarped_t,cessBoardProjector;
Mat display, imageDesktop;
Mat Hpc, Hcb;
vector<Point2f> dst;      // Destination Points to transform overlay image  
vector<Point2f> corners;  
vector<Point2f> displayCorners;      
vector<Point2f> cbCornersProjector;
vector<Point2f> cornersTemplate;
Size board_size(CHESSBOARD_WIDTH-1, CHESSBOARD_HEIGHT-1);


bool bTemplate,bTemplate2;

vector<Point2f> projectedPoints(4);      
Mat imageRect;


void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
        if(vClick.size() < 4 )
        {
 
            vClick.push_back(Point2f(float(x),float(y)));
            cout << x << " "<< y <<endl;
        }
        else
        {
            cout << " Calculating Homography " <<endl;
            cout << cbCornersProjector.size() << " cbCornersProjector " <<endl;
               
            Hpc = findHomography(vClick,vDisplay,0);
            cout << " All right, we have Projector to Camera Homography " <<endl;

            warpPerspective(displayWarped_t, displayWarped, Hpc, imageDesktop.size());
            imshow("Final",displayWarped);
            vClick.clear();
            // Mat Htot = Hpc*Hcb;
            // warpPerspective(display, displayWarped, Htot, imageDesktop.size());
            // imshow("Final 2",displayWarped);

            // Htot = Hcb*Hpc;
            // warpPerspective(display, displayWarped, Htot, imageDesktop.size());
            // imshow("Final 3",displayWarped);
          
        }
 
    }

   
}

  void fillCbCornersProjector(){
   
    // this is the image that has to be reconized (cessboard projected)
    cessBoardProjector = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb652.png", CV_LOAD_IMAGE_GRAYSCALE);

    if( !cessBoardProjector.data )
    { std::cout<< " --(!) Error reading images " << std::endl;  }
    else
    {
      std::cout<< " cb652.png ok " << std::endl;
    }


    bTemplate2 = findChessboardCorners(cessBoardProjector, board_size, cornersTemplate, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (bTemplate2 == 1){

    cout << "Founded corners in the template "<< cornersTemplate.size() << "\n";

    if(cornersTemplate.size() > 1)
    {
    cornerSubPix(cessBoardProjector, cornersTemplate, Size(11,11), Size(-1,-1),TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
    cout << "Subpixel founded\n";
    }
      

    // corners detected in the projector cess board
    cbCornersProjector.push_back(cornersTemplate[0]);
    cbCornersProjector.push_back(cornersTemplate[CHESSBOARD_WIDTH-2]);
    cbCornersProjector.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
    cbCornersProjector.push_back(cornersTemplate[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

    cout << "Finded template corners...\n";
    cout << cbCornersProjector;
    cout << "\n";
      
    }


  }

int main ( int argc, char **argv )
{

  display = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png");
  imageDesktop = imread("/home/sgabello/catkin_ws/src/spatially_ar/src/cb.png", CV_LOAD_IMAGE_COLOR);

  // corners of the image to be projected
  vDisplay.push_back(Point2f(float(0),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(0)));
  vDisplay.push_back(Point2f(float(display.cols),float(display.rows)));
  vDisplay.push_back(Point2f(float(0),float(display.rows)));

  cout << vDisplay << "  vDisplay " << endl;

  // displayCorners.push_back(Point2f(0,0));
  // displayCorners.push_back(Point2f(display.cols,0));
  // displayCorners.push_back(Point2f(display.cols, display.rows));
  // displayCorners.push_back(Point2f(0, display.rows));

  VideoCapture capture(1);

  if(display.empty())
  {
    cerr << "ERR: Unable to find overlay image.\n" << endl;
    return -1;
  }
  
    if(imageDesktop.empty())
  {
    cerr << "ERR: Unable to find overlay image.\n" << endl;
    return -1;
  }

  if ( !capture.isOpened() )
  {
    cerr << "ERR: Unable to capture frames from device 0" << endl;
    return -1;
  }
    
    int key = 0;
  
  fillCbCornersProjector();

  while((char)key!='q')
  {
    // Query for a frame from Capture device
    capture >> img;

    Mat cpy_img(img.rows, img.cols, img.type());
    Mat neg_img(img.rows, img.cols, img.type());
    Mat gray;
    Mat blank(display.rows, display.cols, display.type());

    cvtColor(img, gray, CV_BGR2GRAY);
        
    bool flag = findChessboardCorners(img, board_size, corners);

    if(flag == 1)
    {            

      // This function identifies the chessboard pattern from the gray image, saves the valid group of corners
      cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

      // chess board corners in the camera space
      dst.push_back(corners[0]);
      dst.push_back(corners[CHESSBOARD_WIDTH-2]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-1)-1]);
      dst.push_back(corners[(CHESSBOARD_WIDTH-1)*(CHESSBOARD_HEIGHT-2)]);

      // perspectiveTransform(displayCorners,projectedPoints, Hcalib); 
      if(dst.size() == 4 && vDisplay.size() == 4){
        Mat Hcb = findHomography(vDisplay, dst,0);
      // Htot = findHomography( projectedPoints,dst,0);

      blank = Scalar(0);
      neg_img = Scalar(0);                // Image is white when pixel values are zero
      cpy_img = Scalar(0);                // Image is white when pixel values are zero

      bitwise_not(blank,blank);

      warpPerspective(display, displayWarped_t, Hcb, imageDesktop.size());
      imshow("Warped t", displayWarped_t);

      Mat img2;
      warpPerspective(display, neg_img, Hcb, Size(neg_img.cols, neg_img.rows)); // Transform overlay Image to the position  - [ITEM1]
      warpPerspective(blank, cpy_img, Hcb, Size(cpy_img.cols, neg_img.rows));   // Transform a blank overlay image to position  
      bitwise_not(cpy_img, cpy_img);              // Invert the copy paper image from white to black
      bitwise_and(cpy_img, img, cpy_img);           // Create a "hole" in the Image to create a "clipping" mask - [ITEM2]           
      bitwise_or(cpy_img, neg_img, img2);            // Finally merge both items [ITEM1 & ITEM2]
      imshow("Camera 2", img2);

      }
      setMouseCallback("Camera",on_mouse, NULL );
  
    }

    imshow("Camera", img);
    key = cvWaitKey(1); 
  }
    
  destroyAllWindows();
  return 0;
}
