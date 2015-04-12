#include <rbmt_tracking/tracker.h>

namespace rbmt_tracking{

Tracker::Tracker(ros::NodeHandle nh): nh_(nh) {
  fWidth = 480; //in pixels
  fLength = 360; //in pixels
  rWidth = 240; //in cm
  rLength = 180; //in cm
  end_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("end_pose", 1);
  quad = cv::Mat::zeros(fLength, fWidth, CV_8UC3);
  end_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

}
Tracker::~Tracker() {
  
}

void csv_init(const std::string& filepath) {
  using namespace std;
  using namespace boost;

  ofstream csv;
  csv.open(filepath.c_str(),ios::app);
  if ( csv.is_open() ) csv << "\n" << ","; 
  else {
    assert(false && "csv.open(filepath.c_str()): FALSE");
  }
  csv.close();
}

void Tracker::marker_init() {
 // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  marker.header.frame_id = "world";
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

  int iLowH = 0;
  int iHighH = 179;

  int iLowS = 0;
  int iHighS = 51;

  int iLowV = 128;
  int iHighV = 255;

  //Create trackbars in "Object" window
  createTrackbar("LowH", "Object", &iLowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "Object", &iHighH, 179);

  createTrackbar("LowS", "Object", &iLowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "Object", &iHighS, 255);

  createTrackbar("LowV", "Object", &iLowV, 255);//Value (0 - 255)
  createTrackbar("HighV", "Object", &iHighV, 255);
//============================== object control =====================================================//
  
  tl.x = 431;
  tl.y = 297;
  tr.x = 226;
  tr.y = 476;
  bl.x = 269;
  bl.y = 249;
  br.x = 49;
  br.y = 355;

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

//========================== CREATE AND SHOW IMAGE ==================================================//

    //show the image
    imshow("Camera", imgOriginal);
    imshow("Perspective", quadCopy);
    imshow("Thresholded Image", imgThresholded); //show the thresholded image

//========================== create and show image ==================================================//

    geometry_msgs::PoseStamped sPose;

    marker.lifetime = ros::Duration();
    sPose.header.stamp = ros::Time::now();
    
    sPose.pose.position.y = ((fWidth - (double)posX) * 0.005) - 1.2; // right to left
    sPose.pose.position.x = ((fLength - (double)posY) * 0.005) + 0.35; // backward to forward 
    sPose.pose.position.z = 0.0;

    sPose.pose.orientation.x = 0.0;
    sPose.pose.orientation.y = 0.0;
    sPose.pose.orientation.z = 0.0;
    sPose.pose.orientation.w = 1.0;

    sPose.header.stamp = ros::Time::now();
    if(sPose.pose.position.y < 1.2 && sPose.pose.position.y > -1.2 &&
            sPose.pose.position.x > 0.35 && sPose.pose.position.x < 2.15) {
        end_pose_pub_.publish(sPose);
        marker.pose = sPose.pose;
        end_marker_pub_.publish(marker);
    }

    // std::string csv_filepath = "/home/deanzaka/Github/trui-bot-prj/controller/src/rbmt_tracking/bagfiles/test2.csv";
    // csv_write(sPose,csv_filepath);

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
