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

// #include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// #include <opencv2/opencv.hpp>   // Include OpenCV API
// #include "../cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV
// #include "opencv2/highgui/highgui.hpp"
// #include <iostream>
// #include <cstring>
// #include <hiredis/hiredis.h>
// #include <cstdlib>
std::string RS435_KEY = "from camera";


#define MIN_CIRCLE_RADIUS_PXL 20
#define MAX_CIRCLE_RADIUS_PXL 150
#define US_TO_S (1.0/1000000.0)
#define S_TO_US (1000000.0)
using namespace std;
using namespace cv;

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
static float final_pos[3] = {-0.1, -0.1, 1.0};

static VectorXf measurements(6);

static bool multiple_camera_flag = false;
void addLowerHSVBound(Vec3b &hsv);

const std::string D435_SERIAL_NUM_STR = "832112071449";
const std::string T265_SERIAL_NUM_STR = "905312110116";

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

void update_current_velocity()
{
    for (int i = 0; i < 3; i++)
    {

      current_velocity[i] = (current_position[i] - previous_position[i])/(delta_time_us*US_TO_S);
    }
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
  //float tmpPos[3];
  //rs2_deproject_pixel_to_point(tmpPos, intrinsic, pixelxy, z_val);


  rs2_deproject_pixel_to_point(current_position, intrinsic, pixelxy, z_val);
  cout << "Current position" << endl; 
  cout << current_position[0] << endl;
  cout << current_position[1] << endl; 
  cout << current_position[2] << endl; 
  Vector3d ball_position; 
  ball_position << current_position[0], current_position[1], current_position[2];
  // redis_client.setEigenMatrixJSON(BALL_POSITION_KEY, ball_position);

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
/**********************************************************
Projectile motion
*********************************************************/
void projectile(float final_height, float final_pos[])
{

  float x0 = current_position[0];
  float y0 = current_position[1];
  float z0 = current_position[2];

  float v0x = current_velocity[0];
  float v0y = current_velocity[1];
  float v0z = current_velocity[2];

  float time_to_height;
  float disc = pow(v0y,2) - 2*(grav_const) * (y0 - final_height);

  if (disc > 0)
  {
    time_to_height = (-pow(v0y,2) + sqrt(disc))/(2*grav_const);
    cout<<"time to height  " << time_to_height << endl;
    final_pos[0] =  (x0 + v0x*time_to_height);
    final_pos[2] =  (z0 + v0z*time_to_height);
    
    final_pos[1] = final_height;
    
  }
  else
  {
      cout << "non zero discrimnant" << endl;
     
  }
  
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
        
        
        cout << (int) hsv.at<Vec3b>(y,x)[0] << "," << (int) hsv.at<Vec3b>(y,x)[1]  << "," << (int) hsv.at<Vec3b>(y,x)[2] << endl;
        //cout << matType <<",  " << rgba.cols << " x " << rgb.rows << endl;
        //int bvalue =         (int)(*rgb).at<Vec3b>(y, x)[0];
        // cout << "did not crash" << endl;
        // int gvalue =         (int)(*rgb).at<Vec3b>(y, x)[1];
        // int rvalue =         (int)(*rgb).at<Vec3b>(y, x)[2];

        //cout << bvalue << endl;
     }
}



int main( int argc, char* argv[] ){
    using namespace std;
    using namespace cv;
    using namespace rs2;
    
    redisContext *c;    
    redisReply *reply;

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
   // Define colorizer and align processing-blocks
   colorizer colorize;
   rs2::align align_to(RS2_STREAM_COLOR);

   rs2::context                ctx; 
  // Start the camera
  pipeline pipe(ctx);
  rs2::pipeline t265_pipe(ctx); 

  rs2::pipeline_profile d435_profile;
  rs2::pipeline_profile t265_profile;  
  
  rs2::config d435_cfg;
  rs2::config t265_cfg;

  device_list dev_list = ctx.query_devices();
  cout << dev_list.size();
  
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
    

    //d435_cfg.enable_device(dev_list[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    d435_profile = pipe.start(d435_cfg);

    // get intrinsics
    auto depth_stream = d435_profile.get_stream(RS2_STREAM_DEPTH)
                         .as<rs2::video_stream_profile>();

    auto intrinsic = depth_stream.get_intrinsics();

    // const auto hough_window = "Hough circle tracker";
    // namedWindow(hough_window, WINDOW_AUTOSIZE);

    const auto window_name_clr = "Color Window";
    namedWindow(window_name_clr, WINDOW_AUTOSIZE);

    // const auto window_name_hsv = "HSV Window";
    // namedWindow(window_name_hsv, WINDOW_AUTOSIZE);

    auto gen_element = [](int erosion_size)
    {
        return getStructuringElement(MORPH_ELLIPSE,
            Size(erosion_size + 1, erosion_size + 1),
            Point(erosion_size, erosion_size));
    };

    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);


    while (waitKey(1) < 0 )
    {
      frameset data = pipe.wait_for_frames();

      if (multiple_camera_flag == true)
      {

      frameset t265_data = t265_pipe.wait_for_frames();
      auto pose_frame = t265_data.first_or_default(RS2_STREAM_POSE);
      
      //Cast the frame to pose_frame and get its data
      
      auto pose_data = pose_frame.as<rs2::pose_frame>().get_pose_data();
      string x = to_string(-pose_data.translation.z) + "," + to_string(-pose_data.translation.x) + "," + to_string(2*acos(pose_data.rotation.w));
      cout << "x string:  " << x << endl;
      }

      // Make sure the frameset is spatialy aligned 
      // (each pixel in depth image corresponds to the same pixel in the color image)
      frameset aligned_set = align_to.process(data);
      depth_frame depth = aligned_set.get_depth_frame();

       frame color_frame = aligned_set.get_color_frame();
      auto color_mat = frame_to_mat(aligned_set.get_color_frame());
      frame_color_mat = color_mat;

      //set the callback function for any mouse event

      /*Testing the HSV contrast method*/

      Mat hsv_mat;
      cvtColor(color_mat, hsv_mat, COLOR_BGR2HSV, 0);

      setMouseCallback(window_name_clr, colorVal_CB, &color_mat);


      lowerHSV << 40, 25, 100;

      upperHSV << 60, 200, 255;


      cv::Mat maskHSV, resultHSV;

      cv::inRange(hsv_mat, lowerHSV, upperHSV, maskHSV);
      cv::bitwise_and(hsv_mat, hsv_mat, resultHSV, maskHSV);

      //imshow(window_name_hsv, resultHSV);

      Mat hsv_channels[3];

      // get just the V Channel
      split( resultHSV, hsv_channels );

      Mat grayMat;

      // Reduce noise to avoid false circle detection
      GaussianBlur(hsv_channels[2], grayMat, Size(9,9), 2,2);
      erode(grayMat,grayMat, erode_more);
      dilate(grayMat,grayMat, erode_less);

      vector<Vec3f> circles;


      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours( grayMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
      Mat drawing = Mat::zeros(color_mat.size(), CV_8UC3 );


      if (contours.size() > 0)
      {
        Scalar color = Scalar(255, 0, 0); 
        
        vector<double> contour_areas (contours.size());
        for( int i = 0; i < contours.size(); i++ )
         {
          contour_areas[i] = contourArea(contours[i]);

           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         }
         const int N = sizeof(contour_areas) / sizeof(double);

         auto max_contour_index = distance(contour_areas.begin(), max_element(contour_areas.begin(), contour_areas.end()));
         vector<Point> max_contour = contours[max_contour_index];
        Scalar red_color = Scalar(0, 0,255);
        //drawContours( drawing, contours, max_contour, red_color, 2, 8, hierarchy, 0, Point() );
        
        cv::Point2f center;
        float radius = 0;

        minEnclosingCircle(max_contour, center, radius);
        circle(drawing, center, radius, red_color, 2, 8, 0);
        circle(color_mat, center, radius, red_color, 2, 8, 0);



        //imshow ("contour window", drawing);

        int x_val = (int) center.x;
        int y_val = (int) center.y;

        float depth_at_center = depth.get_distance((int)center.x,(int)center.y);
        string x_str = to_string(x_val);
        string y_str = to_string(y_val);
        string depth_str = to_string(depth_at_center);
        string comma = ",";

        string pixel_string = x_str +  comma + y_str + comma + depth_str;
        // Send information to redis
        reply = (redisReply *)redisCommand(c, "SET %s %s", RS435_KEY.c_str(), pixel_string.c_str());
        //reply = (redisReply *)redisCommand(c, "GET %s", RS435_KEY.c_str());
        //cout << reply->str << endl;
      
            /*
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
          //cout << reply->str << endl;
          
          //reply = (redisReply *)redisCommand(c, "GET %s", MMP_POSE_KEY.c_str());
          //cout << "New estimate:  " << state_estimates.back() << endl;

          rs2_metadata_type frame_metadata;
          frame_metadata = color_frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);

          update_timestamp(frame_metadata);
          update_position(x_val, y_val, depth_at_center, &intrinsic, frame_metadata);
          update_current_velocity();
          collectMeasurements();
          // cout << "x: " << reply->str << endl;

          //estimator.update_measurement(measurements, delta_time_us*US_TO_S);
          }

          /*********** PREDICTOR OVERLAY *********************/
          float pxl_end[2];
          projectile(0, final_pos);

          rs2_project_point_to_pixel(pxl_end, &intrinsic, final_pos);
          Point final_pos_ctr;
          final_pos_ctr = Point(pxl_end[0], pxl_end[1]);
          circle(color_mat, final_pos_ctr, 1, Scalar(255, 0, 0), 3, LINE_AA);
          int radius_end = 5;
          circle(color_mat, final_pos_ctr, radius_end, Scalar(255, 0, 0), 3, LINE_AA);


          imshow(window_name_clr,color_mat);  


        } 
      }

    

    std::cout << "Hello there" << endl;

      return 0;
    }
