/*
Subscriber node to get RGB and Intensity images from Hikvision (i.e. "hik_vision/rgb_image" topic) camera.
and Basler Blaze-101 (i.e. "Blaze/intensity_image") camera.


Function: Calibrates both RGB and Basler camera and finds the relative tranformation between the cameras
using Stereo calibration from OpenCV.

*/


#include <opencv2/opencv.hpp>
#include<iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace message_filters;


std::vector<std::vector<cv::Point2f>> m_blazePoints;
std::vector<std::vector<cv::Point2f>> m_colorPoints;
std::vector<std::vector<cv::Point3f>> m_objectPoints;

cv::Size        m_patternSize = cv::Size(9, 6); // number of corners
float           m_fieldSize = 87; // mm coner size
int cnt = 0;


bool locateChessboardCorners(const cv::Mat image, std::vector<cv::Point2f>& imagePoints)
{
    double winSize = 0;
    int flags = (cv::CALIB_CB_NORMALIZE_IMAGE		// normalize image gamma
                 | cv::CALIB_CB_FILTER_QUADS		// use additional criteria to filter out false quads at the contour retrieval stage
                 | cv::CALIB_CB_FAST_CHECK);		// run fast check on the image which looks for chessboard corners

    // Locate the chessboard corners
    bool found = cv::findChessboardCorners(image, m_patternSize, imagePoints);

    if (found) {
        // Refine the location of the corners
        cv::cornerSubPix(image, imagePoints, cv::Size(5,5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }

    return found;
}


std::vector<cv::Point3f> getObjectPoints()
{
    std::vector<cv::Point3f> objectPoints;
    for (int y = 0; y < m_patternSize.height; ++y) {
        for (int x = 0; x < m_patternSize.width; ++x) {
            objectPoints.push_back(cv::Point3f(x*m_fieldSize, y*m_fieldSize, 0));
        }
    }
    return objectPoints;
}

void calibrateColorCamera(cv::Mat& colorCameraMatrix, cv::Mat& colorDistortion, const cv::Size imgSize)
{
    cv::Mat colorRotation, colorTranslation;
    const double rms = cv::calibrateCamera(
                           m_objectPoints,
                           m_colorPoints,
                           imgSize,
                           colorCameraMatrix,
                           colorDistortion,
                           colorRotation,
                           colorTranslation,
                           (cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3));

    std::cout << "Reprojection error color camera: " << rms << std::endl;
}

void getBlazeCalibration(cv::Mat& blazeCameraMatrix, cv::Mat& blazeDistortion)
{
    const auto cx = 312.182;
    const auto cy = 227.802;
    const auto f = 508.196;

    blazeCameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    blazeCameraMatrix.at<double>(0, 0) = f;
    blazeCameraMatrix.at<double>(0, 2) = cx;
    blazeCameraMatrix.at<double>(1, 1) = f;
    blazeCameraMatrix.at<double>(1, 2) = cy;
    blazeCameraMatrix.at<double>(2, 2) = 1;

    blazeDistortion = cv::Mat::zeros(1, 5, CV_64F);
}


void stereoCalibration(const cv::Mat& colorCameraMatrix, const cv::Mat& colorDistortion, const cv::Mat& blazeCameraMatrix, const cv::Mat& blazeDistortion, const cv::Size imgSize)
{
    cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(1, 3, CV_64F);

    // Calibrate the Combination of RGB and 3D camera
    cv::Mat essentialMatrix, fundamentalMatrix;
    const double rms = cv::stereoCalibrate(
                           m_objectPoints,
                           m_blazePoints,
                           m_colorPoints,
                           blazeCameraMatrix,
                           blazeDistortion,
                           colorCameraMatrix,
                           colorDistortion,
                           imgSize,
                           rotation,
                           translation,
                           essentialMatrix,
                           fundamentalMatrix);

    std::cout << "Reprojection error stereo setup: " << rms << std::endl;

    // Write calibration
    // const std::string filename = std::string("calibration_") + m_blazeCamera.GetCameraInfo().strSerialNumber + "_"
    //                              + m_colorCamera.GetCameraInfo().strSerialNumber + ".xml";
    
    const std::string filename = "Calibration/calib_data.xml";                             
    const std::string path =  filename;
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "colorCameraMatrix" << colorCameraMatrix << "colorDistortion" << colorDistortion;
    fs << "blazeCameraMatrix" << blazeCameraMatrix << "blazeDistortion" << blazeDistortion;
    fs << "rotation" << rotation << "translation" << translation;
    std::cout << "Wrote calibration to " << path << std::endl;
}

void callback(const sensor_msgs::ImageConstPtr& image_blaze, const sensor_msgs::ImageConstPtr& image_rgb)
{
  // Solve all of perception here...
  std::cout<<"inside sync func: \n";
  try
   {
       cv::Mat colorImg= cv_bridge::toCvShare(image_rgb, "bgr8")->image;
       cv::Mat blazeImg= cv_bridge::toCvShare(image_blaze, "mono8")->image;

     cv::imshow("Blaze", blazeImg);
     cv::imshow("HikVision", colorImg);
    //  cv::waitKey(1);
    // Start calibration here ====================================

    cv::Mat color_gray_Img;
    cv::cvtColor(colorImg, color_gray_Img, cv::COLOR_BGR2GRAY);

    char key = static_cast<char>(cv::waitKey(1));
    // if ('q' == key) {
    //     // Cancel and close program
    //     break;
    // } else 
    
    if ('s' == key)  {
        // Detect chessboard corner points
        std::vector<cv::Point2f> colorPoints;
        const bool colorFound = locateChessboardCorners(color_gray_Img, colorPoints);

        std::vector<cv::Point2f> blazePoints;
        const bool blazeFound = locateChessboardCorners(blazeImg, blazePoints);
        cv::Mat blazeImgResult;

        // Saving the detected chessboard corner points
        if (blazeFound && colorFound) {
            std::cout << "Added images from position " << cnt+1 << std::endl;
            m_blazePoints.push_back(blazePoints);
            m_colorPoints.push_back(colorPoints);
            m_objectPoints.push_back(getObjectPoints());
            const std::string path = std::string("calib_images/");
            cv::imwrite(path + "blaze_" + std::to_string(cnt) + ".png", blazeImg);
            cv::imwrite(path + "color_" + std::to_string(cnt) + ".png", colorImg);
            cnt++;
        }
        else {
            std::cout << "Chessboard was not found. Please make sure that the chessboard "
                << "is completely visible in both camera images." << std::endl;
        }

    } else if ('c' == key) {
        // Perform calibration using the previously captured images
        if (m_objectPoints.size() > 1) {
            std::cout << "Calibration" << std::endl;
            cv::Mat colorCameraMatrix, colorDistortion;
            calibrateColorCamera(colorCameraMatrix, colorDistortion, colorImg.size());
            cv::Mat blazeCameraMatrix, blazeDistortion;
            getBlazeCalibration(blazeCameraMatrix, blazeDistortion);
            stereoCalibration(colorCameraMatrix, colorDistortion, blazeCameraMatrix, blazeDistortion, colorImg.size());
            // break;
        } else {
            std::cout << "Not enough images for calibration available!" << std::endl;
        }
    }


    // ===========================================================

   }
   catch (cv_bridge::Exception& e)
   {
    //  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}


int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "Calibration_node");
    ros::NodeHandle nh;
    cv::namedWindow("Blaze");
    cv::namedWindow("HikVision");


    image_transport::ImageTransport it(nh);

    // image_transport::Subscriber sub_blaze = it.subscribe("Blaze/intensity_image", 1, imageCallback_blaze);
    // image_transport::Subscriber sub_rgb = it.subscribe("hik_vision/rgb_image", 1, imageCallback_hik);


    message_filters::Subscriber<sensor_msgs::Image> sub_blaze(nh,"Blaze/intensity_image",1);
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb(nh,"hik_vision/rgb_image",1);

    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_blaze, sub_rgb,10);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    // image_transport::CameraSubscriber 
    
    
    ros::spin();
    cv::destroyWindow("Blaze");
    cv::destroyWindow("HikVision");


    return 0;

}