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
#include <iostream>
#include <string>
#include<pcl-1.10/pcl/io/ply_io.h>

// #include <pcl/io/ply_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/registration/icp.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/console/time.h> 


#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

// typedef pcl::PointXYZ PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

// bool next_iteration = false;

// void
// print4x4Matrix (const Eigen::Matrix4d & matrix)
// {
//   printf ("Rotation matrix :\n");
//   printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//   printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
//   printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
//   printf ("Translation vector :\n");
//   printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
// }

// void
// keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
//                        void* nothing)
// {
//   if (event.getKeySym () == "space" && event.keyDown ())
//     next_iteration = true;
// }

// int
// main (int argc,
//       char* argv[])
// {
//   // The point clouds we will be using
//   PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
//   PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
//   PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
// PointCloudT::Ptr cloud_final (new PointCloudT);
//   // Checking program arguments
//   if (argc < 2)
//   {
//     printf ("Usage :\n");
//     printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
//     PCL_ERROR ("Provide one ply file.\n");
//     return (-1);
//   }

//   int iterations = 1;  // Default number of ICP iterations
//   if (argc > 2)
//   {
//     // If the user passed the number of iteration as an argument
//     iterations = atoi (argv[2]);
//     if (iterations < 1)
//     {
//       PCL_ERROR ("Number of initial iterations must be >= 1\n");
//       return (-1);
//     }
//   }

//   pcl::console::TicToc time;
//   time.tic ();
//   if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
//   {
//     PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
//     return (-1);
//   }
//   std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

//   // Defining a rotation matrix and translation vector
//   Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

//   // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//   double theta = M_PI/8 ;  // The angle of rotation in radians
//   transformation_matrix (0, 0) = std::cos (theta);
//   transformation_matrix (0, 1) = -sin (theta);
//   transformation_matrix (1, 0) = sin (theta);
//   transformation_matrix (1, 1) = std::cos (theta);

//   // A translation on Z axis (0.4 meters)
//   transformation_matrix (2, 3) = 300;
//   transformation_matrix (0, 3) = 200;
//   transformation_matrix (1, 3) = 100;

//   // Display in terminal the transformation matrix
//   std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
//   print4x4Matrix (transformation_matrix);

//   // Executing the transformation
//   pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
//   *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

//   // The Iterative Closest Point algorithm
//   time.tic ();
//   pcl::IterativeClosestPoint<PointT, PointT> icp;
//   icp.setMaximumIterations (iterations);
//   icp.setInputSource (cloud_icp);
//   icp.setInputTarget (cloud_in);
//   icp.align (*cloud_final);
//    pcl::transformPointCloud (*cloud_in, *cloud_final, transformation_matrix);
//   icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
//   std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

//   if (icp.hasConverged ())
//   {
//     std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
//     std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//     transformation_matrix = icp.getFinalTransformation ().cast<double>();
//     print4x4Matrix (transformation_matrix);
//   }
//   else
//   {
//     PCL_ERROR ("\nICP has not converged.\n");
//     return (-1);
//   }

//   // Visualization
//   pcl::visualization::PCLVisualizer viewer ("ICP demo");
//   // Create two vertically separated viewports
//   int v1 (0);
//   int v2 (1);
//   viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//   viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

//   // The color we will be using
//   float bckgr_gray_level = 0.0;  // Black
//   float txt_gray_lvl = 1.0 - bckgr_gray_level;

//   // Original point cloud is white
//   pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
//                                                                              (int) 255 * txt_gray_lvl);
//   viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
//   viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

//   // Transformed point cloud is green
//   pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
//   viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

//   // ICP aligned point cloud is red
//   pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
//   viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

//   // Adding text descriptions in each viewport
//   viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
//   viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

//   std::stringstream ss;
//   ss << iterations;
//   std::string iterations_cnt = "ICP iterations = " + ss.str ();
//   viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

//   // Set background color
//   viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
//   viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

//   // Set camera position and orientation
//   viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//   viewer.setSize (1280, 1024);  // Visualiser window size

//   // Register keyboard callback :
//   viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

//   // Display the visualiser
//   while (!viewer.wasStopped ())
//   {
//     viewer.spinOnce ();

//     // The user pressed "space" :
//     if (next_iteration)
//     {
//       // The Iterative Closest Point algorithm
//       time.tic ();
//       icp.align (*cloud_icp);
//       std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

//       if (icp.hasConverged ())
//       {
//         printf ("\033[11A");  // Go up 11 lines in terminal output.
//         printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
//         std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//         transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
//         print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

//         ss.str ("");
//         ss << iterations;
//         std::string iterations_cnt = "ICP iterations = " + ss.str ();
//         viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
//         viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
//       }
//       else
//       {
//         PCL_ERROR ("\nICP has not converged.\n");
//         return (-1);
//       }
//     }
//     next_iteration = false;
//   }
//   return (0);
// }



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

    // transformation(0,3)=2.5;
    // test_pcl = PointCloud::Ptr(new PointCloud(msg_left->width, msg_left->height));
    // pcl::transformPointCloud (*msg_right, *test_pcl, transformation);
    // *pcl_tr= *test_pcl;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    if (done == 0)
    {
        
        icp.setInputSource(msg_left);
        icp.setInputTarget(msg_right);
        icp.setMaxCorrespondenceDistance (0.1);
        icp.setMaximumIterations (100);
        icp.setTransformationEpsilon (1e-9);
        icp.setEuclideanFitnessEpsilon (1);
        
        // pointcloud= *msg_right;
        PointCloud Final;
        icp.align(Final);
        transformation = icp.getFinalTransformation ();
        transformation(1,3) = 0.08;
    }
    done =1;
    std::cout<<"TF obtained: \n"<<transformation<<'\n';
    // DEFINING MY OWN Tranformation Matrix:
    // transformation(0,3)=2.5;

    // icp.transformCloud(*msg_left,*pointcloud,transformation);
    
    pcl::transformPointCloud (*msg_right, *pointcloud, transformation);

    (*pointcloud)=(*pointcloud)+(*msg_left);

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