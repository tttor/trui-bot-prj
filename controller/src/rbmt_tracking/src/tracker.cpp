#include <rbmt_tracking/tracker.h>

namespace rbmt_tracking{

Tracker::Tracker(ros::NodeHandle nh): nh_(nh) {
  cock_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("cock_pose", 1);
  quad = cv::Mat::zeros(500, 500, CV_8UC3);
  fWidth = 500; //in pixels
  fLength = 500; //in pixels
  rWidth = 180; //in cm
  rLength = 180; //in cm
  dist = 58; //in cm
}
Tracker::~Tracker() {
  
}

/*void Tracker::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    using namespace cv;
    using namespace std;

    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "\n\n\nSET\n\n\n";
        xVal = x;
        yVal = y;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
        else if  ( event == EVENT_MBUTTONDOWN )
    {
        //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        xHover = x;
        yHover = y;
    }
}*/

void Tracker::run(ros::Rate loop_rate) {
  
  using namespace std;
  using namespace cv;

  VideoCapture cap(1); //capture the video from webcam

  if ( !cap.isOpened() )  // if not success, exit program
  {
      cout << "Cannot open the web cam" << endl;
  }

//============================== OBJECT CONTROL =====================================================//
  namedWindow("Object", CV_WINDOW_AUTOSIZE); //create a window called "Object"

  int iLowH = 0;
  int iHighH = 10;

  int iLowS = 135;
  int iHighS = 255;

  int iLowV = 50;
  int iHighV = 255;

  //Create trackbars in "Object" window
  createTrackbar("LowH", "Object", &iLowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "Object", &iHighH, 179);

  createTrackbar("LowS", "Object", &iLowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "Object", &iHighS, 255);

  createTrackbar("LowV", "Object", &iLowV, 255);//Value (0 - 255)
  createTrackbar("HighV", "Object", &iHighV, 255);
//============================== object control =====================================================//

//================================ LEFT CONTROL =====================================================//
  namedWindow("Left", CV_WINDOW_AUTOSIZE); //create a window called "Left"

  int lLowH = 90;
  int lHighH = 115;

  int lLowS = 170;
  int lHighS = 255;

  int lLowV = 120;
  int lHighV = 255;

  //Create trackbars in "Left" window
  createTrackbar("LowH", "Left", &lLowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "Left", &lHighH, 179);

  createTrackbar("LowS", "Left", &lLowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "Left", &lHighS, 255);

  createTrackbar("LowV", "Left", &lLowV, 255);//Value (0 - 255)
  createTrackbar("HighV", "Left", &lHighV, 255);
//================================ left control =====================================================//

//================================ RIGHT CONTROL =====================================================//
  namedWindow("Right", CV_WINDOW_AUTOSIZE); //create a window called "Right"

  int rLowH = 0;
  int rHighH = 10;

  int rLowS = 150;
  int rHighS = 255;

  int rLowV = 50;
  int rHighV = 255;

  //Create trackbars in "Right" window
  createTrackbar("LowH", "Right", &rLowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "Right", &rHighH, 179);

  createTrackbar("LowS", "Right", &rLowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "Right", &rHighS, 255);

  createTrackbar("LowV", "Right", &rLowV, 255);//Value (0 - 255)
  createTrackbar("HighV", "Right", &rHighV, 255);
//================================ right control =====================================================//

/*
//============================== SETUP CORNERS ======================================================//

  cout << "Setup top left...\n Press any key to start, press esc when done \n\n";
  waitKey();

  while(1)
  {
    cap.read(imgOriginal);

    //Create a window
    namedWindow("Camera", 1);

    //set the callback function for any mouse event
    setMouseCallback("Camera", (cv::MouseCallback) &Tracker::CallBackFunc, NULL);

    // show clicked value
    cout << "\t" << xHover << "\t" << yHover << endl;
    tl.x = xVal;
    tl.y = yVal;

    //show the image
    imshow("Camera", imgOriginal);

    if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed, top left set" << endl;
      cout << "top left: " << tl << "\n\n";
        break;
    }
  }

  cout << "Setup top right...\n Press any key to start, press esc when done \n\n";
  waitKey();

  while(1)
  {
      cap.read(imgOriginal);

      //Create a window
      namedWindow("Camera", 1);

      //set the callback function for any mouse event
      setMouseCallback("Camera", (cv::MouseCallback) &Tracker::CallBackFunc, NULL);

      // show clicked value
      cout << "\t" << xHover << "\t" << yHover << endl;
      tr.x = xVal;
      tr.y = yVal;

      //show the image
      imshow("Camera", imgOriginal);

      if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
      {
          cout << "esc key is pressed, top right set" << endl;
          cout << "top right: " << tr << "\n\n";
              break;
      }
  }

  cout << "Setup bottom left...\n Press any key to start, press esc when done \n\n";
  waitKey();

  while(1)
  {
      cap.read(imgOriginal);

      //Create a window
      namedWindow("Camera", 1);

      //set the callback function for any mouse event
      setMouseCallback("Camera", (cv::MouseCallback) &Tracker::CallBackFunc, NULL);

      // show clicked value
      cout << "\t" << xHover << "\t" << yHover << endl;
      bl.x = xVal;
      bl.y = yVal;

      //show the image
      imshow("Camera", imgOriginal);

      if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
      {
          cout << "esc key is pressed, bottom left set" << endl;
          cout << "bottom left: " << bl << "\n\n";
            break;
      }
  }

  cout << "Setup bottom right...\n Press any key to start, press esc when done \n\n";
  waitKey();

  while(1)
  {
      cap.read(imgOriginal);

      //Create a window
      namedWindow("Camera", 1);

      //set the callback function for any mouse event
      setMouseCallback("Camera", (cv::MouseCallback) &Tracker::CallBackFunc, NULL);

      // show clicked value
      cout << "\t" << xHover << "\t" << yHover << endl;
      br.x = xVal;
      br.y = yVal;

      //show the image
      imshow("Camera", imgOriginal);

      if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
      {
          cout << "esc key is pressed, bottom right set" << endl;
          cout << "bottom right: " << br << "\n\n";
            break;
      }
  }
//============================== setup corners ======================================================//
*/
  tl.x = 123;
  tl.y = 310;
  tr.x = 378;
  tr.y = 302;
  bl.x = 24;
  bl.y = 457;
  br.x = 488;
  br.y = 423;
  
  // get mass center
  center.x = (tl.x + tr.x + bl.x + br.x) / 4;
  center.y = (tl.y + tr.y + bl.y + br.y) / 4;

  // input corners to array corners
  corners.clear();
  corners.push_back(tl);
  corners.push_back(tr);
  corners.push_back(br);
  corners.push_back(bl);

  waitKey();
  while (ros::ok()) {
    
    cap.read(imgOriginal);

    // Draw circles on corner and center
    circle( imgOriginal, tl, 3.0, Scalar( 0, 0, 255), 3, 8 );
    circle( imgOriginal, tr, 3.0, Scalar( 0, 0, 255), 3, 8 );
    circle( imgOriginal, bl, 3.0, Scalar( 0, 0, 255), 3, 8 );
    circle( imgOriginal, br, 3.0, Scalar( 0, 0, 255), 3, 8 );
    circle( imgOriginal, center, 3.0, Scalar( 0, 0, 255), 3, 8 );

    // Draw lines
    line(imgOriginal, tl, tr, CV_RGB(0,255,0));
    line(imgOriginal, tl, bl, CV_RGB(0,255,0));
    line(imgOriginal, br, tr, CV_RGB(0,255,0));
    line(imgOriginal, br, bl, CV_RGB(0,255,0));

//================================== draw area ======================================================//


//============================== GET PRESPECTIVE ====================================================//

    cap.read(imgBuffer);

    vector<Point2f> quad_pts;
    quad_pts.push_back(Point2f(0, 0));
    quad_pts.push_back(Point2f(quad.cols, 0));
    quad_pts.push_back(Point2f(quad.cols, quad.rows));
    quad_pts.push_back(Point2f(0, quad.rows));
    Mat transmtx = getPerspectiveTransform(corners, quad_pts);
    warpPerspective(imgBuffer, quadCopy, transmtx, quad.size());
    warpPerspective(imgBuffer, quad, transmtx, quad.size());

//============================== get prespective ====================================================//

//==================== OBJECT DETECTION ===========================================================================//
    Mat imgHSV;
    cvtColor(quad, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //Calculate the moments of the thresholded image
    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;
    int posX, posY;

    // if the area <= 10000, I consider that the there are no object in the image
    //and it's because of the noise, the area is not zero
    if (dArea > 10000)
    {
        //calculate the position of the ball
        posX = dM10 / dArea;
        posY = dM01 / dArea;

        // Draw a circle
        circle( quad, Point(posX,posY), 16.0, Scalar( 0, 0, 255), 3, 8 );

        //cout << posX << "\t";
        //cout << posY << "\n\n";
    }
//==================== object detection ===========================================================================//

//==================== ROBOT LEFT DETECTION =======================================================================//
    Mat imgHSVLeft;
    cvtColor(quad, imgHSVLeft, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgLeft;
    inRange(imgHSVLeft, Scalar(lLowH, lLowS, lLowV), Scalar(lHighH, lHighS, lHighV), imgLeft); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    erode(imgLeft, imgLeft, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgLeft, imgLeft, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (removes small holes from the foreground)
    dilate( imgLeft, imgLeft, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgLeft, imgLeft, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //Calculate the moments of the thresholded image
    Moments oMomentsL = moments(imgLeft);

    double dM01L = oMomentsL.m01;
    double dM10L = oMomentsL.m10;
    double dAreaL = oMomentsL.m00;
    int posXL, posYL;

    // if the area <= 10000, I consider that the there are no object in the image
    //and it's because of the noise, the area is not zero
    if (dAreaL > 10000)
    {
        //calculate the position of the ball
        posXL = dM10L / dAreaL;
        posYL = dM01L / dAreaL;

        // Draw a circle
        circle( quadCopy, Point(posXL,posYL), 6.0, Scalar( 0, 0, 255), 3, 8 );
    }
//==================== robot left detection =======================================================================//

//==================== ROBOT RIGHT DETECTION =======================================================================//
    Mat imgHSVRight;
    cvtColor(quad, imgHSVRight, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgRight;
    inRange(imgHSVRight, Scalar(rLowH, rLowS, rLowV), Scalar(rHighH, rHighS, rHighV), imgRight); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    erode(imgRight, imgRight, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgRight, imgRight, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (removes small holes from the foreground)
    dilate( imgRight, imgRight, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgRight, imgRight, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //Calculate the moments of the thresholded image
    Moments oMomentsR = moments(imgRight);

    double dM01R = oMomentsR.m01;
    double dM10R = oMomentsR.m10;
    double dAreaR = oMomentsR.m00;
    int posXR, posYR;

    // if the area <= 10000, I consider that the there are no object in the image
    //and it's because of the noise, the area is not zero
    if (dAreaR > 10000)
    {
        //calculate the position of the ball
        posXR = dM10R / dAreaR;
        posYR = dM01R / dAreaR;

        // Draw a circle
        circle( quadCopy, Point(posXR,posYR), 6.0, Scalar( 0, 0, 255), 3, 8 );
    }
//==================== robot right detection =======================================================================//


//==================== ROBOT POSE ESTIMATION =======================================================================//

    if(dAreaR > 10000 && dAreaL > 10000)
    {
        if(posYL > posYR)
        {
            h = posYL-posYR;
            l = posXR-posXL;
            deg = 90+(90*(asin(h/sqrt((l*l)+(h*h)))/1.57));
        }
        else if(posYL < posYR)
        {
            h = posYR-posYL;
            l = posXR-posXL;
            deg = 90-(90*(asin(h/sqrt((l*l)+(h*h)))/1.57));
        }
        else deg = 0;
        rCenter.x = (posXL + posXR) / 2;
        rCenter.y = (posYL + posYR) / 2;
        cout << "Center: " << rCenter << "\tDegree: " << deg << "\n\n";
    }
//==================== robot pose estimation =======================================================================//


//========================== CREATE AND SHOW IMAGE ==================================================//

    //show the image
    imshow("Camera", imgOriginal);
    imshow("Perspective", quadCopy);
    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    imshow("Left Bot", imgLeft); //show the thresholded image
    imshow("Right Bot", imgRight); //show the thresholded image

//========================== create and show image ==================================================//

    geometry_msgs::Pose pose;

    pose.position.x = posX;
    pose.position.y = posY;
    pose.position.z = 0.0;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    cock_pose_pub_.publish(pose);
    ros::spinOnce();

    loop_rate.sleep();
    if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed" << endl;
      break;
    }
  }
} 

}
