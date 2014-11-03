#include <rbmt_tracking/tracker.h>

namespace rbmt_tracking{

Tracker::Tracker(ros::NodeHandle nh): nh_(nh) {
  cock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cock_pose", 1);
  quad = cv::Mat::zeros(480, 720, CV_8UC3);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

void Tracker::run_dummy(ros::Rate rate) {
  using namespace std;
  
  marker_init();

  //
  vector<geometry_msgs::Pose> poses;
  geometry_msgs::Pose pose;

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  poses.push_back(pose);

  pose.position.x = 3.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  poses.push_back(pose);

  pose.position.x = 3.0;
  pose.position.y = 3.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  poses.push_back(pose);

  pose.position.x = 1.0;
  pose.position.y = 3.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  poses.push_back(pose);

  //
  std_msgs::Header header;
  header.frame_id = "map";

  marker.lifetime = ros::Duration();

  size_t idx = 0;
  while (ros::ok())  {
    geometry_msgs::PoseStamped spose;
    spose.header = header;
    spose.pose = poses.at(idx%poses.size());

    marker.pose = spose.pose;

    cock_pose_pub_.publish(spose);
    marker_pub_.publish(marker);

    ++idx;
    ros::spinOnce();
    rate.sleep();
  }
}

void Tracker::marker_init() {
 // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
}

void Tracker::run(ros::Rate loop_rate) {
  using namespace std;
  using namespace cv;

  marker_init();

  VideoCapture cap(1); //capture the video from webcam

  if ( !cap.isOpened() )  // if not success, exit program
  {
      cout << "Cannot open the web cam" << endl;
  }

//============================== OBJECT CONTROL =====================================================//
  namedWindow("Object", CV_WINDOW_AUTOSIZE); //create a window called "Object"

  int iLowH = 30;
  int iHighH = 50;

  int iLowS = 65;
  int iHighS = 170;

  int iLowV = 120;
  int iHighV = 255;

  //Create trackbars in "Object" window
  createTrackbar("LowH", "Object", &iLowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "Object", &iHighH, 179);

  createTrackbar("LowS", "Object", &iLowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "Object", &iHighS, 255);

  createTrackbar("LowV", "Object", &iLowV, 255);//Value (0 - 255)
  createTrackbar("HighV", "Object", &iHighV, 255);
//============================== object control =====================================================//

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
  tl.x = 154;
  tl.y = 283;
  tr.x = 504;
  tr.y = 291;
  bl.x = 22;
  bl.y = 387;
  br.x = 620;
  br.y = 401;
  
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


//========================== CREATE AND SHOW IMAGE ==================================================//

    //Create a window
    //namedWindow("Camera", 1);

    //show the image
    imshow("Camera", imgOriginal);

    //Create a window
    //namedWindow("Perspective", 1);

    //show the image
    imshow("Perspective", quad);
    imshow("Thresholded Image", imgThresholded); //show the thresholded image

//========================== create and show image ==================================================//

    geometry_msgs::PoseStamped sPose;

    marker.lifetime = ros::Duration();
    
    sPose.pose.position.x = (double)posX * 0.005;
    sPose.pose.position.y = (480 - (double)posY) * 0.005;
    sPose.pose.position.z = 0.0;

    sPose.pose.orientation.x = 0.0;
    sPose.pose.orientation.y = 0.0;
    sPose.pose.orientation.z = 0.0;
    sPose.pose.orientation.w = 1.0;

    cock_pose_pub_.publish(sPose);
    marker.pose = sPose.pose;
    marker_pub_.publish(marker);

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
