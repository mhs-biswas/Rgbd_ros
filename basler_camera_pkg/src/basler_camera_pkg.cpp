/*
Pulisher node to publish Intensity images from Basler Blaze-101 camera.
Published topic: "Blaze/intensity_image"

*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include<iostream>
#include <ConsumerImplHelper/BlazeCamera.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>

using namespace GenTLConsumerImplHelper;
using namespace GenApi;

// CameraList cameraList = CBlazeCamera::EnumerateCameras();


void setupBlazeCamera(CBlazeCamera& m_blazeCamera, std::string dev_serial_num)
{
    // m_blazeCamera.Open(SerialNumber, "23882275");
    m_blazeCamera.Open(SerialNumber, dev_serial_num);
    // m_blazeCamera.Open(cameraList[0]);
    // std::cout << typeid(cameraList).name() << std::endl;

    // If there are multiple cameras connected and you want to open a specific one, use
    // the CToFCamera::Open(CameraInfoKey, string) method.
    std::cout << "Connected to ToF camera " << m_blazeCamera.GetCameraInfo().strDisplayName << std::endl;

    // Enable range data.
    m_blazeCamera.SetCameraParameterValue("ComponentSelector", "Range");
    m_blazeCamera.SetCameraParameterValue("ComponentEnable", true);
    m_blazeCamera.SetCameraParameterValue("PixelFormat", "Coord3D_ABC32f");
    m_blazeCamera.SetCameraParameterValue("Scan3dCoordinateSelector", "CoordinateA");
    m_blazeCamera.SetCameraParameterValue("Scan3dInvalidDataValue", 0.0);
    m_blazeCamera.SetCameraParameterValue("Scan3dCoordinateSelector", "CoordinateB");
    m_blazeCamera.SetCameraParameterValue("Scan3dInvalidDataValue", 0.0);
    m_blazeCamera.SetCameraParameterValue("Scan3dCoordinateSelector", "CoordinateC");
    m_blazeCamera.SetCameraParameterValue("Scan3dInvalidDataValue", 0.0);
    m_blazeCamera.SetCameraParameterValue("OperatingMode","ShortRange");
    m_blazeCamera.SetCameraParameterValue("FastMode",true);

    // m_blazeCamera.SetCameraParameterValue("AmbiguityFilter",true);

    // Enable the intensity image.
    m_blazeCamera.SetCameraParameterValue("ComponentSelector", "Intensity");
    m_blazeCamera.SetCameraParameterValue("ComponentEnable", true);
    m_blazeCamera.SetCameraParameterValue("PixelFormat", "Mono16");

    // Disable the confidence map.
    m_blazeCamera.SetCameraParameterValue("ComponentSelector", "Confidence");
    m_blazeCamera.SetCameraParameterValue("ComponentEnable", false);

    // Reduce exposure time to avoid overexposure at close-up range
    m_blazeCamera.SetCameraParameterValue("ExposureTime", 1000.0);

    // Configure the camera for software trigger.
    m_blazeCamera.SetCameraParameterValue("TriggerMode", "On");
    m_blazeCamera.SetCameraParameterValue("TriggerSource", "Software");
}



cv::Mat processBlazeData(const GrabResult& result, CBlazeCamera& m_blazeCamera)
{
    BufferParts parts;
    m_blazeCamera.GetBufferParts(result, parts);

    const int width = (int)parts[1].width;
    const int height = (int)parts[1].height;
    const int count = width * height;
    cv::Mat intensity = cv::Mat(height, width, CV_16UC1, parts[1].pData);
    intensity.convertTo(intensity, CV_8UC1, 1.0 / 256.0);

    return intensity;
}

cv::Mat processBlazeData_cloudpoint(const GrabResult& result, CBlazeCamera& m_blazeCamera)
{
    BufferParts parts;
    m_blazeCamera.GetBufferParts(result, parts);

    const int width = (int)parts[0].width;
    const int height = (int)parts[0].height;
    // const int count = width * height;
    cv::Mat pointcloud = cv::Mat(height, width, CV_32FC3, parts[0].pData);
    // pointcloud.convertTo(pointcloud, CV_FC3);
    pointcloud=pointcloud;

    return pointcloud;
}


image_transport::Publisher intensity_pub_, pointcloud_pub_;

bool publish(cv::Mat frame, cv::Mat frame_pcl, ros::Time acquisition_time)
{ 
  std::cout<<frame.size()<<'\n';
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"mono8", frame).toImageMsg();
  intensity_pub_.publish(msg);

  sensor_msgs::ImagePtr pcl = cv_bridge::CvImage(std_msgs::Header(),"rgb8", frame_pcl).toImageMsg();
  pointcloud_pub_.publish(pcl);

  return 1;

}

void getBlazeCalibration(CBlazeCamera& m_blazeCamera)
{
    const auto cx = m_blazeCamera.GetCameraParameterValue<double>("Scan3dPrincipalPointU");
    const auto cy = m_blazeCamera.GetCameraParameterValue<double>("Scan3dPrincipalPointV");
    const auto f = m_blazeCamera.GetCameraParameterValue<double>("Scan3dFocalLength");

    std::cout<<"Cx: "<<cx<<" Cy: "<<cy<<" f:"<<f<<'\n';

    // blazeCameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    // blazeCameraMatrix.at<double>(0, 0) = f;
    // blazeCameraMatrix.at<double>(0, 2) = cx;
    // blazeCameraMatrix.at<double>(1, 1) = f;
    // blazeCameraMatrix.at<double>(1, 2) = cy;
    // blazeCameraMatrix.at<double>(2, 2) = 1;

    // blazeDistortion = cv::Mat::zeros(1, 5, CV_64F);
}

int main(int argc, char* argv[])
{   

    ros::init(argc, argv, "Blaze_node");
    ros::NodeHandle n("~");
    image_transport::ImageTransport it_(n);
    intensity_pub_=it_.advertise("Blaze/intensity_image", 1);
    pointcloud_pub_ =it_.advertise("Blaze/pointcloud", 1);
    // ros::Rate loop_rate(100);


    std::string serial_num;

    n.getParam("serial_num",serial_num);
    std::cout<<"serial_num"<<serial_num<<'\n';
    CBlazeCamera m_blazeCamera;
    CBlazeCamera::InitProducer();
    int nBuffer =10;

    cv::Mat blazeCameraMatrix, blazeDistortion;
    // getBlazeCalibration(m_blazeCamera);

    setupBlazeCamera(m_blazeCamera, serial_num);
    getBlazeCalibration(m_blazeCamera);
    m_blazeCamera.PrepareAcquisition(nBuffer);

    for (size_t i = 0; i < nBuffer; ++i) {
            m_blazeCamera.QueueBuffer(i);
           
        }


    m_blazeCamera.StartAcquisition();
    m_blazeCamera.IssueAcquisitionStartCommand();
    m_blazeCamera.ExecuteCameraCommand("TriggerSoftware");
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        GrabResult blazeResult;
        m_blazeCamera.GetGrabResult(blazeResult, 1000);
        m_blazeCamera.ExecuteCameraCommand("TriggerSoftware");

        if (blazeResult.status != GrabResult::Ok) {
                std::cerr << "Failed to grab image." << std::endl;
            
            if (blazeResult.status != GrabResult::Timeout) {
                        // Put the buffer back into the acquisition engine's input queue.
                        m_blazeCamera.QueueBuffer(blazeResult.hBuffer);
                    }

            continue;
        }


        cv::Mat blazeImg = processBlazeData(blazeResult, m_blazeCamera);
        cv::Mat pointcloud = processBlazeData_cloudpoint(blazeResult, m_blazeCamera);
        // cv::imshow("blaze", blazeImg);
        // cv::waitKey(0);
        // cv::imshow("")

        publish(blazeImg,pointcloud,ros::Time::now());
        // publish(pointcloud,)
        m_blazeCamera.QueueBuffer(blazeResult.hBuffer);
        ros::spinOnce();
        loop_rate.sleep();
    }
    // processBlazeData()
     ros::spin();
    return 0;
}