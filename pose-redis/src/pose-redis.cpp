// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <math.h>
#include <hiredis/hiredis.h>

std::string MMP_POSE_KEY = "pose of base from t265";
std::string MMP_VELOCITIES_KEY = "velocity of base from t265";
std::string MMP_ACCELERATIONS_KEY = "acceleration of base from t265";

using namespace std;

int main(int argc, char * argv[]) try
{
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

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Create string of x data
        string x = to_string(-pose_data.translation.z) + "," + to_string(-pose_data.translation.x) + "," + to_string(2*acos(pose_data.rotation.w));
        string xd = to_string(-pose_data.velocity.z) + "," + to_string(-pose_data.velocity.x) + "," + to_string(pose_data.angular_velocity.y);
        string xdd = to_string(-pose_data.acceleration.z) + "," + to_string(-pose_data.acceleration.x) + "," + to_string(pose_data.angular_acceleration.y);
   
      	reply = (redisReply *)redisCommand(c, "SET %s %s", MMP_POSE_KEY.c_str(), x.c_str());
      	reply = (redisReply *)redisCommand(c, "GET %s", MMP_POSE_KEY.c_str());
      	cout << "x: " << reply->str << endl;
      	reply = (redisReply *)redisCommand(c, "SET %s %s", MMP_VELOCITIES_KEY.c_str(), xd.c_str());
      	reply = (redisReply *)redisCommand(c, "GET %s", MMP_VELOCITIES_KEY.c_str());
      	cout << "xd: " << reply->str << endl;
      	reply = (redisReply *)redisCommand(c, "SET %s %s", MMP_ACCELERATIONS_KEY.c_str(), xdd.c_str());
      	reply = (redisReply *)redisCommand(c, "GET %s", MMP_ACCELERATIONS_KEY.c_str());
      	cout << "xdd: " << reply->str << endl;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

