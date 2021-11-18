/*
Pulisher node to publish rgb images from Hikvision camera.
Published topic: "hik_vision/rgb_image"

*/



#include "hik_vision/hik_vision.hpp"
#include <opencv2/opencv.hpp>
#include<iostream>
// #include <ConsumerImplHelper/BlazeCamera.h>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

image_transport::Publisher image_pub_;

bool publish(cv::Mat frame, ros::Time acquisition_time)
{ 


  std::cout<<frame.size()<<'\n';
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  image_pub_.publish(msg);

  return 1;

}


int main(int argc, char* argv[])
{   
    // std::string filepath = argv[2];

       
    int exitCode = EXIT_SUCCESS;
    ros::init(argc, argv, "rgb_camera_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);
    image_pub_=it_.advertise("hik_vision/rgb_image", 1);
    ros::Rate loop_rate(100);

    camera::hik_vision my_camera;
    std::cout<<my_camera.initialize()<<'\n';

    while (ros::ok())
    {
        cv::Mat frame = my_camera.capture();
        
        
        // cv::imshow("Display Image",frame);
        publish(frame,ros::Time::now());
        // Save 3D data
        ros::spinOnce();
        loop_rate.sleep();  
    }
    ros::spin();


    return exitCode;



}