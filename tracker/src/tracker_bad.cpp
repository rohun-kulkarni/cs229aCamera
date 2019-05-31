#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "../include/cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <cstring>
#include <hiredis/hiredis.h>
#include <cstdlib> 
#include <librealsense2/rsutil.h>
#include "../include/ekf.hpp"
#include "redis/RedisClient.h"



#define MIN_CIRCLE_RADIUS_PXL 20
#define MAX_CIRCLE_RADIUS_PXL 150
#define US_TO_S (1.0/1000000.0)

using namespace std;
using namespace cv;
using namespace rs2;
using namespace Eigen;

string RS435_KEY = "from camera";

static Vec2d currMousePos;
static Mat frame_color_mat;
static Vec3b ballHSV;
static bool ballHSV_SetFlag = false;
static Vec3i lowerHSV;
static Vec3i upperHSV;

static float current_velocity[3];
static float current_position[3] ;
static float previous_position[3];
static double current_pos_timestamp;
static double previous_pos_timestamp;
static float delta_time_us = 1;


static vector<double> timestamps_history;
static vector<double> x_vel_history;
static vector<double> y_vel_history;
static vector<double> z_vel_history;

static vector<double> x_pos_history;
static vector<double> y_pos_history;
static vector<double> z_pos_history;

static bool multiple_camera_flag = false;
  
static VectorXf measurements(6);

std::string MMP_POSE_KEY = "pose of base from t265";
std::string MMP_VELOCITIES_KEY = "velocity of base from t265";
std::string MMP_ACCELERATIONS_KEY = "acceleration of base from t265";
std::string BALL_POSITION_KEY = "sai2::cs225a::ball_position";

const std::string D435_SERIAL_NUM_STR = "832112071449";
const std::string T265_SERIAL_NUM_STR = "905312110116";


/* Function defines */

/* DEFINES*/
#define MAX_HISTORY 100
#define MIN_UPDATE_THRESH .01

void update_current_velocity()
{
    for (int i = 0; i < 3; i++)
    {

      current_velocity[i] = (current_position[i] - previous_position[i])/(delta_time_us*US_TO_S);
    }
}
/* Checks if component of a vector (e.g. position or velocity) exceeds threshold */
bool component_exceeds_thresh(float vector3d[3], float thresh)
{

   int idx = 0;
  while (idx < 2)
  {
    if (fabs(vector3d[idx] - current_position[idx]) >= thresh)
    {
      return true;

    }
    idx++;
  } 
  return false;
}
void update_position(int x_val, int y_val, float z_val, rs2_intrinsics * intrinsic, double timestamp)
{
 /*
  double x_m = (1.0* x_val) / z_val;
  double y_m = (1.0* y_val) / z_val;

  current_pos << x_m, y_, (double) z_val;
  */
  for (int i = 0; i <=2; i++)
  {
    previous_position[i] = current_position[i];
  }

  float pixelxy[2] = {(float) x_val, (float) y_val};
  float tmpPos[3];
  rs2_deproject_pixel_to_point(tmpPos, intrinsic, pixelxy, z_val);

  // Update

  // Basic thresholding to reduce noise... may remove for filtering. 
  bool thresholdExceeded = component_exceeds_thresh(tmpPos, .01);

  //if (thresholdExceeded)
  //{
      // Save the current position. 
  // cout << "current_x_pos before update:   ";
  //   for (int idx = 0; idx < 2; idx++)
  //   {
  //     cout << current_position[idx];
  //     cout << ",  ";
  //   }
  //   cout << endl;
  //   //cout << current_velocity[0] << endl;

      // start redis client
  // auto redis_client = RedisClient();
  // redis_client.connect();

  rs2_deproject_pixel_to_point(current_position, intrinsic, pixelxy, z_val);
  // cout << "Current position" << endl; 
  // cout << current_position[0] << endl;
  // cout << current_position[1] << endl; 
  // cout << current_position[2] << endl; 
  Vector3d ball_position; 
  ball_position << current_position[0], current_position[1], current_position[2];
  // redis_client.setEigenMatrixJSON(BALL_POSITION_KEY, ball_position);

    // cout << "current_position     ";
    // for (int idx = 0; idx < 3; idx++)
    // {
    //   cout << current_position[idx];
    //   cout << ",  ";
    // }
    // cout << endl;

    //  cout << "previous_position     ";
    // for (int idx = 0; idx < 3; idx++)
    // {
    //   cout << previous_position[idx];
    //   cout << ",  ";
    // }
    // cout << endl;

    
      // save the timestamp. 

    //  cout << "current time: " << current_pos_timestamp << endl;
     // cout << "Current Position:  " <<  current_position[0] << "  ," << current_position[1] << "  ,"  << current_position[2] << "  ,"  << endl;
  //}
}

void update_timestamp(double timestamp)
{

    previous_pos_timestamp = current_pos_timestamp;
    current_pos_timestamp = timestamp;

    delta_time_us = current_pos_timestamp - previous_pos_timestamp;
}

void collectMeasurements()
{
  for (int idx = 0; idx < 3; idx++)
  {
    measurements(idx) = current_position[idx];
  }
  for (int idx = 3; idx < 6; idx++)
  {
    measurements(idx) = current_velocity[idx-3];
  }
}

void addLowerHSVBound(Vec3b &hsv);

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}



// Checks if HSV bound is lower or higher than the original bounds. 
void addLowerHSVBound(Vec3b &hsv)
{
  for (int i = 0; i < 3; i++)
  {
    if (hsv[i] < upperHSV[i]) 
    {
      lowerHSV[i]  = hsv[i];
    }
  }
}

// Checks if HSV bound is lower or higher than the original bounds. 
void addUpperHSVBound(Vec3b &hsv)
{
  for (int i = 0; i < 3; i++)
  {
    if (hsv[i] > lowerHSV[i]) 
    {
      lowerHSV[i]  = hsv[i];
    }
  }
}



void colorVal_CB(int event, int x, int y, int flags, void* color_mat)
{
        Vec3b bgr = frame_color_mat.at<Vec3b>(y,x);
        Mat hsv;
        cvtColor(frame_color_mat, hsv, COLOR_BGR2HSV , 0);
        ballHSV = hsv.at<Vec3b>(y,x);   
     if  ( event == EVENT_LBUTTONDOWN )
     {
          ballHSV = hsv.at<Vec3b>(y,x);
          cout << "HSV Value stored for ball" << endl;
          cout << "Ball HSV" << (int) ballHSV[0] << "," << (int)  ballHSV[1] << "," << (int)  ballHSV[2] << endl;
          ballHSV_SetFlag = true;
          addLowerHSVBound(ballHSV);
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {

          cout << "HSV Value stored for ball" << endl;
          cout << "Ball HSV" << (int) ballHSV[0] << "," << (int)  ballHSV[1] << "," << (int)  ballHSV[2] << endl;
          ballHSV_SetFlag = true;
          addUpperHSVBound(ballHSV);
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
        
        
        // cout << (int) hsv.at<Vec3b>(y,x)[0] << "," << (int) hsv.at<Vec3b>(y,x)[1]  << "," << (int) hsv.at<Vec3b>(y,x)[2] << endl;
        //cout << matType <<",  " << rgba.cols << " x " << rgb.rows << endl;
        //int bvalue =         (int)(*rgb).at<Vec3b>(y, x)[0];
        // cout << "did not crash" << endl;
        // int gvalue =         (int)(*rgb).at<Vec3b>(y, x)[1];
        // int rvalue =         (int)(*rgb).at<Vec3b>(y, x)[2];

        //cout << bvalue << endl;
     }
}



int main( int argc, char* argv[] ){

/*********************REDIS ********************/
    // Set up redis connection.
    redisContext *c;    
    redisReply *reply;

    // //start redis client
    // auto redis_client = RedisClient();
    // redis_client.connect();


    const char *hostname = "127.0.0.1";
    int port =  6379;
    struct timeval timeout = {1, 500000}; // 1.5 seconds
    c = redisConnectWithTimeout(hostname, port, timeout);
    if (c == NULL || c-> err)
    {
      if (c) 
      {
        cout << "got here" << endl; 
        printf("Connection error: %s\n", c->errstr);
        redisFree(c);
      }
      else
      {
        printf("Connection error: can't allocate redis conetxt \n");
      }
      exit(1);
    }
  
  /*********************DEVICE AND STREAM SETUP ********************/    
    // Set up context for devices
    rs2::context                ctx;          
    rs2::pipeline d435_pipe(ctx);
    rs2::pipeline t265_pipe(ctx); 
    rs2::config d435_cfg;
    rs2::config t265_cfg;

    rs2::pipeline_profile d435_profile;
    rs2::pipeline_profile t265_profile;

    // get connected devices
    device_list dev_list = ctx.query_devices();
    // find the index with the right camera
    int dev_idx = 0;
    cout << "getting devices " << endl;
    if (dev_list.size() > 1)
    {
      multiple_camera_flag = true;

      cout << "multiple devices found " << endl;
      string first_dev_serial_num =  dev_list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (first_dev_serial_num.compare(D435_SERIAL_NUM_STR) == 0)
      {
        d435_cfg.enable_device(dev_list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        t265_cfg.enable_device(dev_list[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

      }
      else 
      {
        d435_cfg.enable_device(dev_list[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        t265_cfg.enable_device(dev_list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
      }
         
         t265_profile = t265_pipe.start(t265_cfg);

    }
    else // assume only the rgbd camera is connected. TODO: make this publisher more versatile/
    {
      d435_cfg.enable_device(dev_list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));



    }
          d435_profile = d435_pipe.start(d435_cfg);

    
    



    
    // Set up pipeline for t265
    // rs2::pipeline t265_pipe(ctx);
    // rs2::config t265_cfg;

    // // get connected devices
    // device_list dev_list = ctx.query_devices();
    // t265_cfg.enable_device(dev_list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    
    // rs2::pipeline_profile t265_profile = t265_pipe.start(t265_cfg);
    

    // Define colorizer and align processing-blocks
    colorizer colorize;
    rs2::align align_to(RS2_STREAM_COLOR);


    // Start the camera
    //pipeline pipe;
    

    // Create a configuration for configuring the pipeline with a non default profile

    // Add pose stream
    //cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    //d435_cfg.enable_all_streams();


    
    // Set the device to High Accuracy preset
    
    //auto sensor = d435_profile.get_device().first<rs2::depth_sensor>();
    //sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

    auto depth_stream = d435_profile.get_stream(RS2_STREAM_DEPTH)
                         .as<rs2::video_stream_profile>();

    auto intrinsic = depth_stream.get_intrinsics();

    // Gui windows;
    const auto hough_window = "Hough circle tracker";
    namedWindow(hough_window, WINDOW_AUTOSIZE);

    const auto window_name_clr = "Color Window";
    namedWindow(window_name_clr, WINDOW_AUTOSIZE);


    const auto window_name_hsv = "HSV Window";
    namedWindow(window_name_hsv, WINDOW_AUTOSIZE);


    // Set up erosion elements for blurring image
    auto gen_element = [](int erosion_size)
    {
        return getStructuringElement(MORPH_ELLIPSE,
            Size(erosion_size + 1, erosion_size + 1),
            Point(erosion_size, erosion_size));
    };

    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);

    rs2_error* e = 0;

    // Initialize EKF
    EKF estimator(6, 3);

    while (waitKey(1) < 0)
    {
    // auto intrinsic1 = depth_stream.get_intrinsics();
    // float ppx  = intrinsic1.ppx;
    // float ppy = intrinsic1.ppy;
    // float fx = intrinsic1.fx;
    // float fy = intrinsic1.fy;

    //cout << "(ppx, ppy) = " << "( " <<  ppx << ",   " << ppy << ")" ;
    //cout << "(fx, fy) = " << "( " <<  fx << ",   " << fy << ")" ;
    

      /*********************RETRIEVE FRAMES ********************/
      frameset data = d435_pipe.wait_for_frames();
      //rs2_metadata_type frame_timestamp = rs2_get_frame_timestamp(data, &e);

      //
      // Make sure the frameset is spatialy aligned 
      // (each pixel in depth image corresponds to the same pixel in the color image)
      frameset aligned_set = data;//align_to.process(data);

      // get aligned frames 

      depth_frame depth = aligned_set.get_depth_frame();
      frame color_frame = aligned_set.get_color_frame();
      
      if (multiple_camera_flag == true)
      {

      frameset t265_data = t265_pipe.wait_for_frames();
      auto pose_frame = t265_data.first_or_default(RS2_STREAM_POSE);
      
      //Cast the frame to pose_frame and get its data
      
      auto pose_data = pose_frame.as<rs2::pose_frame>().get_pose_data();

        //Create string of x data
        string x = to_string(-pose_data.translation.z) + "," + to_string(-pose_data.translation.x) + "," + to_string(2*acos(pose_data.rotation.w));
        string xd = to_string(-pose_data.velocity.z) + "," + to_string(-pose_data.velocity.x) + "," + to_string(pose_data.angular_velocity.y);
        string xdd = to_string(-pose_data.acceleration.z) + "," + to_string(-pose_data.acceleration.x) + "," + to_string(pose_data.angular_acceleration.y);
   
        // reply = (redisReply *)redisCommand(c, "SET %s %s", MMP_POSE_KEY.c_str(), x.c_str());
        
        //reply = (redisReply *)redisCommand(c, "GET %s", MMP_POSE_KEY.c_str());
        //cout << "x: " << reply->str << endl;
        
        // reply = (redisReply *)redisCommand(c, "SET %s %s", MMP_VELOCITIES_KEY.c_str(), xd.c_str());
        //reply = (redisReply *)redisCommand(c, "GET %s", MMP_VELOCITIES_KEY.c_str());
        //cout << "xd: " << reply->str << endl;
        
        // reply = (redisReply *)redisCommand(c, "SET %s %s", MMP_ACCELERATIONS_KEY.c_str(), xdd.c_str());
        //reply = (redisReply *)redisCommand(c, "GET %s", MMP_ACCELERATIONS_KEY.c_str());
        //cout << "xdd: " << reply->str << endl;

      }

      // get timestamp 
      rs2_metadata_type frame_metadata;
      frame_metadata = color_frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);



      auto color_mat = frame_to_mat(color_frame);
      frame_color_mat = color_mat;
      //cout << color_mat.size() << endl;

      Mat hsv_mat;
      cvtColor(color_mat, hsv_mat, COLOR_BGR2HSV, 0);

      // Set Callback for UI
      setMouseCallback(window_name_clr, colorVal_CB, &color_mat); 

      /*********************COLOR SEGMENTATION ********************/
      // Balloon
      lowerHSV << 40, 25, 100;

      upperHSV << 60, 200, 255;


      // Tennis Ball
      // lowerHSV << 25, 125, 60;
      // upperHSV << 80, 180, 250; 

    
      cv::Mat maskHSV, resultHSV;

      cv::inRange(hsv_mat, lowerHSV, upperHSV, maskHSV);
      cv::bitwise_and(hsv_mat, hsv_mat, resultHSV, maskHSV);

      imshow(window_name_hsv, resultHSV);

      Mat hsv_channels[3];

      // get just the V Channel
      split( resultHSV, hsv_channels );

      Mat grayMat;

      // Reduce noise to avoid false circle detection
      GaussianBlur(hsv_channels[2], grayMat, Size(9,9), 2,2);
      erode(grayMat,grayMat, erode_more);
      dilate(grayMat,grayMat, erode_less);
      /***********************************************************/

      /*********************FIND CONTOURS********************/
      vector<Vec3f> circles;
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours( grayMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
      Mat drawing = Mat::zeros(color_mat.size(), CV_8UC3 );

      // Display contours (all)
      if (contours.size() > 0)
      {
        Scalar color = Scalar(255, 0, 0); 
        vector<double> contour_areas (contours.size());
        // Calculate the area of each contour. 
        for( int i = 0; i < contours.size(); i++ )
         {
          contour_areas[i] = contourArea(contours[i]);

           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         }
         const int N = sizeof(contour_areas) / sizeof(double);

         // Find the contour with the max area. 
         auto max_contour_index = distance(contour_areas.begin(), max_element(contour_areas.begin(), contour_areas.end()));
         vector<Point> max_contour = contours[max_contour_index];
      
        //drawContours( drawing, contours, max_contour, red_color, 2, 8, hierarchy, 0, Point() );
        
        cv::Point2f center;
        float radius = 0;

        minEnclosingCircle(max_contour, center, radius);

        Scalar red_color = Scalar(0, 0,255);  
        circle(drawing, center, radius, red_color, 2, 8, 0);
        circle(color_mat, center, radius, red_color, 2, 8, 0);



        int x_val = (int) center.x;
        int y_val = (int) center.y;

        float depth_at_center = depth.get_distance((int)center.x,(int)center.y);
        
        if (depth_at_center != 0)
        {
          string x_str = to_string(x_val);
          string y_str = to_string(y_val);
          string depth_str = to_string(depth_at_center);
          string comma = ",";

          string pixel_string = x_str +  comma + y_str + comma + depth_str;
          // Send information to redis
          
           reply = (redisReply *)redisCommand(c, "SET %s %s", RS435_KEY.c_str(), pixel_string.c_str());
          
          reply = (redisReply *)redisCommand(c, "GET %s", RS435_KEY.c_str());
          cout << reply->str << endl;
          
          //reply = (redisReply *)redisCommand(c, "GET %s", MMP_POSE_KEY.c_str());
          //cout << "New estimate:  " << state_estimates.back() << endl;


          update_timestamp(frame_metadata);
          update_position(x_val, y_val, depth_at_center, &intrinsic, frame_metadata);
          update_current_velocity();
          collectMeasurements();
          // cout << "x: " << reply->str << endl;

          estimator.update_measurement(measurements, delta_time_us*US_TO_S);
          }

        imshow(window_name_clr,color_mat);  
        imshow ("contour window", drawing);
      
      }


      /* HOUGH CIRCLES
      HoughCircles(grayMat, circles, HOUGH_GRADIENT, 1, 
                  grayMat.rows/8, 100, 30, 10, 150);
      
      Mat hough_mat;
      hough_mat = grayMat.clone();
      Point center;
      for (size_t i = 0; i < circles.size(); i++)
      {
        Vec3i c = circles[i];
        center = Point(c[0],c[1]);
        // circle center
        circle(hough_mat, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(hough_mat, center, radius, Scalar(255,0,255), 3, LINE_AA);
      }



      imshow(hough_window, hough_mat);
      */



    }

    std::cout << "Hello there" << endl;

      return 0;
    }
    