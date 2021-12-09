#include<eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include<iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
// #include <icp.h>
#include <pcl-1.10/pcl/registration/icp.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace message_filters;

PointCloud::Ptr pointcloud(new PointCloud);
PointCloud::Ptr test_pcl(new PointCloud);
PointCloud::Ptr pcl_tr(new PointCloud);

ros::Publisher  pcl_output;
int done =0;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
Eigen::Matrix4f rel_tf = Eigen::Matrix4f::Identity();


void callback(const PointCloud::ConstPtr& msg_left, const PointCloud::ConstPtr& msg_right)
{
  printf ("Cloud_left: width = %d, height = %d\n", msg_left->width, msg_left->height);
  printf ("Cloud_right: width = %d, height = %d\n", msg_right->width, msg_right->height);
    // pointcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
  pointcloud = PointCloud::Ptr(new PointCloud(msg_left->width, msg_left->height));
  pointcloud->header.frame_id = "map";
//   msg_left->header.frame_id = "new_map";
//   BOOST_FOREACH (const pcl::PointXYZRGB& pt_left, msg_left->points)
//   {
//     //   printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt_left.x, pt_left.y, pt_left.z, pt_left.r, pt_left.g, pt_left.b);
//     //   if((pt_left.r>100 && pt_left.r<255) && (pt_left.g>100 && pt_left.g<255) && (pt_left.b>0 && pt_left.b<10))
//     //   {
//     //    printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt_left.x, pt_left.y, pt_left.z, pt_left.r, pt_left.g, pt_left.b);   
//     //   }
//     //   else 
//     //   {
//     //       pt_left.r = 255;
//     //       pt_left.g = 255;
//     //       pt_left.b = 255;
//     //   }
//   }
//   pcl_output.publish(*msg_left);

//   BOOST_FOREACH (const pcl::PointXYZRGB& pt_right, msg_right->points)
//     {

//     }
    // printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt.x, pt.y, pt.z, pt.r, pt.g, pt.b);

    transformation(0,3)=2.5;
    test_pcl = PointCloud::Ptr(new PointCloud(msg_left->width, msg_left->height));
    pcl::transformPointCloud (*msg_right, *test_pcl, transformation);
    *pcl_tr= *test_pcl;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    if (done == 0)
    {
        
        icp.setInputCloud(test_pcl);
        icp.setInputTarget(msg_right);
        // icp.setMaxCorrespondenceDistance (0.1);
        // icp.setMaximumIterations (100);
        // icp.setTransformationEpsilon (1e-8);
        // icp.setEuclideanFitnessEpsilon (1);
        
        // pointcloud= msg_right;
        // PointCloud Final;
        icp.align(*pointcloud);
        rel_tf = icp.getFinalTransformation ();
    }
    done =1;
    std::cout<<"TF obtained: \n"<<rel_tf<<'\n';
    // DEFINING MY OWN Tranformation Matrix:
    // transformation(0,3)=2.5;

    // icp.transformCloud(*msg_left,*pointcloud,transformation);
    pcl::transformPointCloud (*msg_right, *pointcloud, rel_tf);

    (*pointcloud)=(*pointcloud)+(*test_pcl);

    pcl_output.publish(pointcloud);
    printf("aligned \n");

}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Rel_TF_node");
    ros::NodeHandle nh("~");
    // ros::Subscriber sub = nh.subscribe<PointCloud>("/cloud_pcl_left", 1, callback);
    pcl_output = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/processed_pcl", 10);
    

    message_filters::Subscriber<PointCloud> sub_left_pcl(nh,"/cloud_pcl_left",10);
    message_filters::Subscriber<PointCloud> sub_right_pcl(nh,"/cloud_pcl_right",10);

    TimeSynchronizer<PointCloud, PointCloud> sync(sub_left_pcl, sub_right_pcl,10);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::spin();

// Eigen::umeyama()

}