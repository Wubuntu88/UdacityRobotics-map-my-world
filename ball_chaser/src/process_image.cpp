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
    double right_cutoff = img.width / 3.0 * 2.0;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for(int row = 0; row < img.height; ++row) {
      for(int col = 0; col < img.step; ++col) {
        int pixel_value_1 = img.data[row * img.step + col];
        int pixel_value_2 = img.data[row * img.step + col + 1];
        int pixel_value_3 = img.data[row * img.step + col + 2];
        if (pixel_value_1 == white_pixel && 
            pixel_value_2 == white_pixel && 
            pixel_value_3 == white_pixel) {
          if (col <= left_cutoff) {
            ++left_cutoff;
          }
          if (col > left_cutoff && col < right_cutoff) {
            ++front_counter;
          }
          if (col > right_cutoff) {
            ++right_cutoff;
          }
        }
      }
    }
    
    if (left_side_counter == 0 && front_counter == 0 && right_side_counter == 0) {
      drive_robot(0.0, 0.0);
    }
    int left_index = 0;
    int front_index = 1;
    int right_index = 2;
    std::vector<int> vec{left_side_counter, front_counter, right_side_counter};
    int maxElementIndex = std::max_element(vec.begin(), vec.end()) - vec.begin();
    if (maxElementIndex == 0) {
      drive_robot(0.25, 0.5);
    }else if (maxElementIndex == 1) {
      drive_robot(0.25, 0.0);
    }else if (maxElementIndex == 2) {
      drive_robot(0.25, -0.5);
    }else { // if all pixel counters are equal
      drive_robot(0, 0);
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