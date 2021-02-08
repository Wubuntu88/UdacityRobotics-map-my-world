#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("sending DriveToTarget message...");
    ball_chaser::DriveToTarget driveToTargetService;
    driveToTargetService.request.linear_x = lin_x;
    driveToTargetService.request.angular_z = ang_z;
    if (!client.call(driveToTargetService)) {
      ROS_ERROR("Failed to call service /ball_chaser/command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_side_counter = 0;
    int front_counter = 0;
    int right_side_counter = 0;
    
    double left_cutoff = img.width / 3.0;
    double right_cutoff = img.width * 2.0 / 3.0;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for(int row = 0; row < img.height; ++row) {
      for(int col = 0; col < img.width; ++col) {
        int index = row * img.step + col * 3;
        int red_pixel = img.data[index];
        int green_pixel = img.data[index + 1];
        int blue_pixel = img.data[index + 2];
        if (red_pixel == white_pixel && 
            green_pixel == white_pixel && 
            blue_pixel == white_pixel) {
          if (col <= left_cutoff) {
            ++left_side_counter;
          }
          if (col > left_cutoff && col < right_cutoff) {
            ++front_counter;
          }
          if (col > right_cutoff) {
            ++right_side_counter;
          }
        }
      }
    }
    
    ROS_INFO("left: %d, center: %d, right: %d", left_side_counter, front_counter, right_side_counter);
    
    if (left_side_counter == 0 && front_counter == 0 && right_side_counter == 0) {
      drive_robot(0.0, 0.0);
      ROS_INFO("STOP; there are no white pixels.");
      return;
    }
    int left_index = 0;
    int front_index = 1;
    int right_index = 2;
    std::vector<int> vec{left_side_counter, front_counter, right_side_counter};
    int maxElementIndex = std::max_element(vec.begin(), vec.end()) - vec.begin();
    if (maxElementIndex == 0) {
      drive_robot(0.2, 0.4);  // Turn Left
      ROS_INFO("LEFT");
    }else if (maxElementIndex == 1) {
      drive_robot(0.4, 0.0);  // Go Straight
      ROS_INFO("CENTER");
    }else if (maxElementIndex == 2) {
      drive_robot(0.2, -0.4);  // Turn Right
      ROS_INFO("RIGHT");
    }else { // if all pixel counters are equal
      drive_robot(0, 0);  // stop
      ROS_INFO("STOP");
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}