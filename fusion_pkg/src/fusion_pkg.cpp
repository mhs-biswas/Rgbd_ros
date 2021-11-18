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
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/common/common_headers.h>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vtkRenderWindow.h>
#include <typeinfo>

// #include <

using namespace message_filters;

cv::Mat         m_rotation;
cv::Mat         m_translation;
cv::Mat         m_colorCameraMatrix;
cv::Mat         m_colorDistortion;

pcl::visualization::PCLVisualizer::Ptr m_visualizer;


void loadCalibrationFile()
{
    // It is assumed that the 'calibration.xml' contains information about the relative orientation
    // of the cameras to each other and the optical calibration of the color camera.
    // The calibration program can be used to create the file.
    // const std::string filename = std::string("calibration_") + m_blazeCamera.GetCameraInfo().strSerialNumber + "_"
    //                              + m_colorCamera.GetCameraInfo().strSerialNumber + ".xml";

    const std::string path = "Calibration/calib_data.xml";
    std::cout << "Loading the calibration file: " << path << std::endl;
    cv::FileStorage fs(path, cv::FileStorage::READ);

    // if (!fs.isOpened()) {
    //     std::string msg("For the fusion of the data, calibration must first be performed.\nMake sure that there is a valid calibration with file name 'calibration.xml' in the directory.");
    //     throw RUNTIME_EXCEPTION(msg);
    // }

    fs["rotation"] >> m_rotation;
    fs["translation"] >> m_translation;
    fs["colorCameraMatrix"] >> m_colorCameraMatrix;
    fs["colorDistortion"] >> m_colorDistortion;
}


void setupVisualizer()
{
    // Set up a PCLVisualizer, i.e. background color, coordinate system and initial camera position
    m_visualizer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Basler AG - RGBD-Fusion"));
    m_visualizer->setBackgroundColor(0, 0, 0);
    m_visualizer->setShowFPS(false);
    m_visualizer->addCoordinateSystem(0.1);
    m_visualizer->setCameraPosition(0.0, 0.1, -2.5, 0, -1.0, 0);
    m_visualizer->getRenderWindow()->GlobalWarningDisplayOff();
    vtkObject::GlobalWarningDisplayOff();
}


cv::Mat warpColorToDepth(const cv::Mat pointcloud, const cv::Mat& color)
{
    const int width = pointcloud.cols;
    const int height = pointcloud.rows;
    cv::Mat copy_pcl(pointcloud.size(), CV_32FC3);
    // pointcloud.convertTo(pointcloud, CV_32FC3, 1/255.0);
    // pointcloud.copyTo(copy_pcl);

    // const cv::Vec3f point3d = pointcloud.at<cv::Vec3f>(0, 0)

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            const cv::Vec3f point3d = pointcloud.at<cv::Vec3f>(i, j);
            copy_pcl.at<cv::Vec3f>(i,j)=point3d;

        }

    }

    cv::Mat coloredDepth(pointcloud.size(), CV_8UC3);
    
    std::cout<< "Size: "<<pointcloud.size()<<'\n';
    // std::cout << typeid(pointcloud).name() << endl;
    
    // cv::namedWindow("Blaze_depth");
    // cv::imshow("Blaze_depth", pointcloud);
    // // cv::destroyWindow("Blaze_depth");
    // cv::waitKey(1);


                    cv::Mat pointcloudVec = copy_pcl.reshape(3, 1);
                    // std::cout<< "Size pointcloudVec: "<<pointcloudVec.size()<<'\n';
                    cv::Mat projectedPoints;
                    cv::projectPoints(pointcloudVec, m_rotation, m_rotation* m_translation, m_colorCameraMatrix, m_colorDistortion, projectedPoints);

                    const cv::Vec3b invalidPoint(0, 0, 0);
                    cv::Rect colorRect(cv::Point(), color.size());
                    for (int row = 0; row < height; ++row) {
                        for (int col = 0; col < width; ++col) {
                            const int idx = row * width + col;
                            const cv::Vec3f& point3d = pointcloudVec.at<cv::Vec3f>(idx);
                            if (point3d[2] != 0.0f) {
                                const cv::Point2f& colorPoint = projectedPoints.at<cv::Point2f>(idx);
                                if (colorRect.contains(colorPoint)) {
                                    coloredDepth.at<cv::Vec3b>(row, col) = color.at<cv::Vec3b>(colorPoint);
                                } else {
                                    // No color information avalable
                                    coloredDepth.at<cv::Vec3b>(row, col) = invalidPoint;
                                }
                            } else {
                                // No depth information available for this pixel. Zero it.
                                coloredDepth.at<cv::Vec3b>(row, col) = invalidPoint;
                            }
                        }
                    }

    return coloredDepth;
}


std::string cloudID = "colored_cloud";
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void callback(const sensor_msgs::ImageConstPtr& image_blaze_pointcloud, const sensor_msgs::ImageConstPtr& image_rgb)
{
    std::cout<<"inside fusion func: \n";
  try
   {    

        cv::Mat colorImg= cv_bridge::toCvShare(image_rgb, "rgb8")->image;
        cv::Mat blazeImg= cv_bridge::toCvShare(image_blaze_pointcloud, "rgb8")->image;
        cv::Mat coloredDepth = warpColorToDepth(blazeImg, colorImg);

        // cv::namedWindow("coloured_depth");
        // cv::imshow("coloured_depth", coloredDepth);
        // // cv::destroyWindow("Blaze_depth");
        // cv::waitKey(1);
        // cv::destroyWindow("coloured_depth");

        const int width = blazeImg.cols;
        const int height = blazeImg.rows;
        pointcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
        for (int row = 0; row < height; row += 1) {
            for (int col = 0; col < width; col += 1) {
                auto& coloredPoint = pointcloud->at(col, row); // PCL uses column/row instead of row/column

                const cv::Vec3f point3d = blazeImg.at<cv::Vec3f>(row, col)/ 1000.0f; // Scaling from mm to m for PCL
                coloredPoint.x = point3d[0];
                coloredPoint.y = point3d[1];
                coloredPoint.z = point3d[2];
                // std::cout<<"X: "<<coloredPoint.x<<" Y: "<<coloredPoint.y<<" Z: "<<coloredPoint.z<<'\n'; 

                const cv::Vec3b color = coloredDepth.at<cv::Vec3b>(row, col);
                coloredPoint.r = color[0];
                coloredPoint.g = color[1];
                coloredPoint.b = color[2];
            }
        }

        // Update the pointcloud and refresh the visualizer
        m_visualizer->updatePointCloud(pointcloud, cloudID);
        m_visualizer->spinOnce();


   }
   catch (cv_bridge::Exception& e)
   {
    //  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }

}

int main(int argc, char* argv[])
{   
    loadCalibrationFile();

    setupVisualizer();
    m_visualizer->addPointCloud(pointcloud, cloudID);
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID);


    ros::init(argc, argv, "Fusion_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    message_filters::Subscriber<sensor_msgs::Image> sub_blaze_pointcloud(nh,"Blaze/pointcloud",1);
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb(nh,"hik_vision/rgb_image",1);

    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_blaze_pointcloud, sub_rgb,10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

}