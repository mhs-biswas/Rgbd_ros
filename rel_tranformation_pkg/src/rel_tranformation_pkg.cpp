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
#include <pcl-1.10/pcl/registration/icp_nl.h>
#include <pcl-1.10/pcl/registration/gicp.h>
#include <iostream>
#include <string>
#include<pcl-1.10/pcl/io/ply_io.h>

// #include <pcl/io/ply_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/registration/icp.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/console/time.h> 
#include <pcl-1.10/pcl/keypoints/sift_keypoint.h>
#include <pcl-1.10/pcl/registration/correspondence_rejection_sample_consensus.h>
#include <iostream>
#include <string>
#include <pcl-1.10/pcl/features/pfh.h>
#include <pcl-1.10/pcl/features/pfhrgb.h>
#include <pcl-1.10/pcl/io/ply_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/registration/icp.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/console/time.h>   // TicToc
#include <pcl-1.10/pcl/correspondence.h>

#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/console/time.h>
#include <pcl-1.10/pcl/features/normal_3d.h>
#include <pcl-1.10/pcl/features/fpfh.h>
#include <pcl-1.10/pcl/registration/correspondence_estimation.h>
#include <pcl-1.10/pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl-1.10/pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl-1.10/pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl-1.10/pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl-1.10/pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl-1.10/pcl/registration/default_convergence_criteria.h>
#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.10/pcl/filters/voxel_grid.h>
#include <pcl-1.10/pcl/console/parse.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_representation.h>

#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/conversions.h>
#include <pcl-1.10/pcl/filters/uniform_sampling.h>
#include <pcl-1.10/pcl/features/normal_3d.h>
#include <pcl-1.10/pcl/features/fpfh.h>
#include <pcl-1.10/pcl/registration/correspondence_estimation.h>
#include <pcl-1.10/pcl/registration/correspondence_rejection_distance.h>
#include <pcl-1.10/pcl/registration/transformation_estimation_svd.h>

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

using namespace pcl::console;
using namespace pcl::registration;
// using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> PointCloud_norm;
using namespace message_filters;

PointCloud::Ptr pointcloud(new PointCloud);
PointCloud::Ptr test_pcl(new PointCloud);
PointCloud::Ptr pcl_tr(new PointCloud);

ros::Publisher  pcl_output;
ros::Publisher  left_keypts;
ros::Publisher  right_keypts;
int done =0;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
Eigen::Matrix4d rel_tf = Eigen::Matrix4d::Identity();
// ================================================================

void
estimateKeypoints (const PointCloud::ConstPtr &src, 
                   const PointCloud::ConstPtr &tgt,
                   PointCloud &keypoints_src,
                   PointCloud &keypoints_tgt)
{
  // Get an uniform grid of keypoints
  std::cout<<"EstimateKeyPoint: \n";
  pcl::UniformSampling<pcl::PointXYZRGB> uniform;
  uniform.setRadiusSearch (0.1);  // 1m

  uniform.setInputCloud (src);
  uniform.filter (keypoints_src);

  uniform.setInputCloud (tgt);
  uniform.filter (keypoints_tgt);

  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
  // pcl_viewer source_pcd keypoints_src.pcd -ps 1 -ps 10
//   savePCDFileBinary ("keypoints_src.pcd", keypoints_src);
//   savePCDFileBinary ("keypoints_tgt.pcd", keypoints_tgt);
}

// ================================================================

const float min_scale = 0.01f; // the standard deviation of the smallest scale in the scale space
const int n_octaves = 3;  // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave = 4; // the number of scales to compute within each octave
const float min_contrast = 0.0001f; // the minimum contrast required for detection

void
estimate_SIFT_Keypoints (const PointCloud::Ptr &src, 
                   const PointCloud::Ptr &tgt,
                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src,
                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_tgt)
{
  // Get an uniform grid of keypoints
  std::cout<<"Estimate_SIFT_KeyPoint: \n";
  
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
  sift_detect.setScales( min_scale , n_octaves , n_scales_per_octave);
  sift_detect.setMinimumContrast(min_contrast);
  
  sift_detect.setInputCloud(src);
  sift_detect.compute(*keypoints_src);

  sift_detect.setInputCloud(tgt);
  sift_detect.compute(*keypoints_tgt);

//   pcl::UniformSampling<pcl::PointXYZRGB> uniform;
//   uniform.setRadiusSearch (0.1);  // 1m

//   uniform.setInputCloud (src);
//   uniform.filter (keypoints_src);

//   uniform.setInputCloud (tgt);
//   uniform.filter (keypoints_tgt);

  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
  // pcl_viewer source_pcd keypoints_src.pcd -ps 1 -ps 10
//   savePCDFileBinary ("keypoints_src.pcd", keypoints_src);
//   savePCDFileBinary ("keypoints_tgt.pcd", keypoints_tgt);
}


// ================================================================
void
estimateNormals (const PointCloud::Ptr &src, 
                 const PointCloud::Ptr &tgt,
                 PointCloud_norm &normals_src,
                 PointCloud_norm &normals_tgt)
{
  std::cout<<"EstimateNormals: \n";
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_est;
  normal_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
  normal_est.setInputCloud (src);
  normal_est.setRadiusSearch (0.25);  // 50cm
  normal_est.compute (normals_src);

  normal_est.setInputCloud (tgt);
  normal_est.compute (normals_tgt);

//   // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
//   // pcl_viewer normals_src.pcd
//   PointCloud<PointNormal> s, t;
//   copyPointCloud (*src, s);
//   copyPointCloud (normals_src, s);
//   copyPointCloud (*tgt, t);
//   copyPointCloud (normals_tgt, t);
//   savePCDFileBinary ("normals_src.pcd", s);
//   savePCDFileBinary ("normals_tgt.pcd", t);
}

// ================================================================
// void
// estimateFPFH (const PointCloud::Ptr &src, 
//               const PointCloud::Ptr &tgt,
//               const PointCloud_norm::Ptr &normals_src,
//               const PointCloud_norm::Ptr &normals_tgt,
//               const PointCloud::Ptr &keypoints_src,
//               const PointCloud::Ptr &keypoints_tgt,
//               pcl::PointCloud<pcl::PFHRGBSignature250> &fpfhs_src,
//               pcl::PointCloud<pcl::PFHRGBSignature250> &fpfhs_tgt)
// {   
//   std::cout<<"EstimateFPFH: \n";
//   pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> fpfh_est;
//   fpfh_est.setInputCloud (keypoints_src);
//   fpfh_est.setInputNormals (normals_src);
//   fpfh_est.setRadiusSearch (0.1); // 1m
//   fpfh_est.setSearchSurface (src);
//   fpfh_est.compute (fpfhs_src);

//   fpfh_est.setInputCloud (keypoints_tgt);
//   fpfh_est.setInputNormals (normals_tgt);
//   fpfh_est.setSearchSurface (tgt);
//   fpfh_est.compute (fpfhs_tgt);

//   // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
//   // pcl_viewer fpfhs_src.pcd
// //   PCLPointCloud2 s, t, out;
// //   toPCLPointCloud2 (*keypoints_src, s); toPCLPointCloud2 (fpfhs_src, t); concatenateFields (s, t, out);
// //   savePCDFile ("fpfhs_src.pcd", out);
// //   toPCLPointCloud2 (*keypoints_tgt, s); toPCLPointCloud2 (fpfhs_tgt, t); concatenateFields (s, t, out);
// //   savePCDFile ("fpfhs_tgt.pcd", out);
// }

// ================================================================
void
estimate_SIFT_FPFH (const PointCloud::Ptr &src, 
              const PointCloud::Ptr &tgt,
              const PointCloud_norm::Ptr &normals_src,
              const PointCloud_norm::Ptr &normals_tgt,
              pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src,
              pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_tgt,
              pcl::PointCloud<pcl::PFHRGBSignature250> &fpfhs_src,
              pcl::PointCloud<pcl::PFHRGBSignature250> &fpfhs_tgt)
{   
  std::cout<<"Estimate_SIFT_FPFH: \n";
  pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> fpfh_est;
  fpfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb_src(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*keypoints_src , *keypoints_xyzrgb_src);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb_tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*keypoints_tgt , *keypoints_xyzrgb_tgt);


  fpfh_est.setInputCloud (keypoints_xyzrgb_src);
  fpfh_est.setInputNormals (normals_src);
  fpfh_est.setRadiusSearch (0.25); // 1m
  fpfh_est.setSearchSurface (src);
  fpfh_est.compute (fpfhs_src);

  fpfh_est.setInputCloud (keypoints_xyzrgb_tgt);
  fpfh_est.setInputNormals (normals_tgt);
  fpfh_est.setSearchSurface (tgt);
  fpfh_est.compute (fpfhs_tgt);

  // For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
  // pcl_viewer fpfhs_src.pcd
//   PCLPointCloud2 s, t, out;
//   toPCLPointCloud2 (*keypoints_src, s); toPCLPointCloud2 (fpfhs_src, t); concatenateFields (s, t, out);
//   savePCDFile ("fpfhs_src.pcd", out);
//   toPCLPointCloud2 (*keypoints_tgt, s); toPCLPointCloud2 (fpfhs_tgt, t); concatenateFields (s, t, out);
//   savePCDFile ("fpfhs_tgt.pcd", out);
}



// ================================================================
void
findCorrespondences (const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_src,
                     const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_tgt,
                     pcl::Correspondences &all_correspondences)
{   
  std::cout<<"Find Correspondence: \n";
  //CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
  //CorrespondenceEstimation<PointT, PointT> est;
  pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  
//   est.setSourceNormals (src);
//   est.setTargetNormals (tgt);
//   est.setKSearch (10);
  est.determineCorrespondences (all_correspondences);
  //est.determineReciprocalCorrespondences (all_correspondences);
}

// ================================================================

void
rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr &src,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr &tgt,
                          pcl::Correspondences &remaining_correspondences)
{

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_src_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_tgt_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::copyPointCloud(*src, *keypoints_src_xyzrgb);
  pcl::copyPointCloud(*tgt, *keypoints_tgt_xyzrgb);

  std::cout<<"Reject Bad correspondence: \n";
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rej;
  rej.setInputSource (keypoints_src_xyzrgb);
  rej.setInputTarget (keypoints_tgt_xyzrgb);
  // rej.setMaximumDistance (1);    // 1m

  rej.setInlierThreshold(0.1);
  rej.setMaximumIterations(5000);
  rej.setRefineModel(true);//false

  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);


//   rej.setMedianFactor (8.79241104);
//   rej.setInputCorrespondences (all_correspondences);

//   rej.getCorrespondences (remaining_correspondences);
//   return;
  
//   pcl::CorrespondencesPtr remaining_correspondences_temp (new pcl::Correspondences);
//   rej.getCorrespondences (*remaining_correspondences_temp);
//   PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());

//   // Reject if the angle between the normals is really off
//   pcl::registration::CorrespondenceRejectorSurfaceNormal rej_normals;
//   rej_normals.setThreshold (std::acos (pcl::deg2rad (45.0)));
//   rej_normals.initializeDataContainer<pcl::PointXYZRGB, pcl::PointXYZRGB> ();
//   rej_normals.setInputCloud<pcl::PointXYZRGB> (src);
//   rej_normals.setInputNormals<pcl::PointXYZRGB, pcl::PointXYZRGB> (src);
//   rej_normals.setInputTarget<pcl::PointXYZRGB> (tgt);
//   rej_normals.setTargetNormals<pcl::PointXYZRGB, pcl::PointXYZRGB> (tgt);
//   rej_normals.setInputCorrespondences (remaining_correspondences_temp);
//   rej_normals.getCorrespondences (remaining_correspondences);
}

// ==================================================================

// void
// findTransformation (const PointCloud::ConstPtr &src,
//                     const PointCloud::ConstPtr &tgt,
//                     const pcl::CorrespondencesPtr &correspondences,
//                     Eigen::Matrix4d &transform)
// {
//   pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGB, pcl::PointXYZRGB, double> trans_est;
//   trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
// }

// ==================================================================

pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb_src(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb_tgt(new pcl::PointCloud<pcl::PointXYZRGB>);

void callback(const PointCloud::ConstPtr& left, const PointCloud::ConstPtr& right)
{   

    /*
        msg_left is my source
        msg_right is my target
    */

   PointCloud::Ptr msg_left (new PointCloud(left->width, left->height));
   PointCloud::Ptr msg_right (new PointCloud(left->width, left->height));
  
   *msg_left = *left;
   *msg_right = *right;

  //  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = msg_left->begin(); it != msg_left->end(); it++) {
  //     if ((it->r== 0) && (it->g== 0) && (it->b== 0)) {
  //         // pcl::PointCloud<pcl::PointXYZRGB>::iterator dumy ;
  //         // *dumy=*it;
  //         msg_left->erase(it);
          
  //     }
  //   }

  //   for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = msg_right->begin(); it != msg_right->end(); it++) {
  //     if ((it->r== 0) && (it->g== 0) && (it->b== 0)) {
  //         msg_right->erase(it);
  //     }
  //   }


    printf ("Cloud_left: width = %d, height = %d\n", left->width, left->height);
    printf ("Cloud_right: width = %d, height = %d\n", right->width, right->height);
                                                        // pointcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(width, height));
                                                        pointcloud = PointCloud::Ptr(new PointCloud(msg_left->width, msg_left->height));
                                                        pointcloud->header.frame_id = "map";
                                                        //                 //   msg_left->header.frame_id = "new_map";
                                                        //                 //   BOOST_FOREACH (const pcl::PointXYZRGB& pt_left, msg_left->points)
                                                        //                 //   {
                                                        //                 //     //   printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt_left.x, pt_left.y, pt_left.z, pt_left.r, pt_left.g, pt_left.b);
                                                        //                 //     //   if((pt_left.r>100 && pt_left.r<255) && (pt_left.g>100 && pt_left.g<255) && (pt_left.b>0 && pt_left.b<10))
                                                        //                 //     //   {
                                                        //                 //     //    printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt_left.x, pt_left.y, pt_left.z, pt_left.r, pt_left.g, pt_left.b);   
                                                        //                 //     //   }
                                                        //                 //     //   else 
                                                        //                 //     //   {
                                                        //                 //     //       pt_left.r = 255;
                                                        //                 //     //       pt_left.g = 255;
                                                        //                 //     //       pt_left.b = 255;
                                                        //                 //     //   }
                                                        //                 //   }
                                                        //                 //   pcl_output.publish(*msg_left);

                                                        //                 //   BOOST_FOREACH (const pcl::PointXYZRGB& pt_right, msg_right->points)
                                                        //                 //     {

                                                        //                 //     }
                                                        //                     // printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt.x, pt.y, pt.z, pt.r, pt.g, pt.b);

                                                        //                     // transformation(0,3)=2.5;
                                                        //                     // test_pcl = PointCloud::Ptr(new PointCloud(msg_left->width, msg_left->height));
                                                        //                     // pcl::transformPointCloud (*msg_right, *test_pcl, transformation);
                                                        //                     // *pcl_tr= *test_pcl;

                                                        // pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                                                        // if (done == 0)
                                                        // {
                                                            
                                                        //     icp.setInputSource(msg_left);
                                                        //     icp.setInputTarget(msg_right);
                                                        //     icp.setMaxCorrespondenceDistance (0.1);
                                                        //     icp.setMaximumIterations (100);
                                                        //     icp.setTransformationEpsilon (1e-10);
                                                        //     icp.setEuclideanFitnessEpsilon (0.01);
                                                            
                                                        //     // pointcloud= *msg_right;
                                                        //     PointCloud Final;
                                                        //     icp.align(Final);
                                                        //     transformation = icp.getFinalTransformation ();
                                                        //     std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
                                                        //     // transformation(1,3) = 0.08;
                                                        // }
                                                        // done =1;

        

    // if (done<1)
    // { 
        // PointCloud::Ptr left (new PointCloud(msg_left->width, msg_left->height));
        // PointCloud::Ptr right (new PointCloud(msg_left->width, msg_left->height));
        
        // *left = *msg_left;
        // *right = *msg_right;

                      
        // pcl_output.publish(right);
        // *msg_left = *left;
        // *msg_right = *right;
    
    keypoints_xyzrgb_src->header.frame_id = "map";
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb_tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
    keypoints_xyzrgb_tgt->header.frame_id = "map";

    if (done <1)
    { 

        pcl::VoxelGrid<pcl::PointXYZRGB> sor ;
        sor.setInputCloud(msg_left);
        // sor.setMeanK(50);
        // sor.setStddevMulThresh(1);
        sor.setLeafSize(0.01,0.01,0.01);
        sor.filter(*msg_left) ;

        // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor ;
        sor.setInputCloud(msg_right);
        sor.setLeafSize(0.01,0.01,0.01);
        // sor.setMeanK(50);
        // sor.setStddevMulThresh(1);
        sor.filter(*msg_right) ;
        
        PointCloud_norm::Ptr normals_src (new PointCloud_norm), 
                          normals_tgt (new PointCloud_norm);
        estimateNormals (msg_left, msg_right, *normals_src, *normals_tgt);

        // PointCloud::Ptr keypoints_src (new PointCloud), keypoints_tgt (new PointCloud);
        
        // estimateKeypoints (msg_left, msg_right, *keypoints_src, *keypoints_tgt);

        pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_tgt(new pcl::PointCloud<pcl::PointWithScale>),
                                                  keypoints_src(new pcl::PointCloud<pcl::PointWithScale>);

        estimate_SIFT_Keypoints(msg_left, msg_right, keypoints_src, keypoints_tgt);

        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr fpfhs_src (new pcl::PointCloud<pcl::PFHRGBSignature250>), 
                                   fpfhs_tgt (new pcl::PointCloud<pcl::PFHRGBSignature250>);
        // estimateFPFH (msg_left, msg_right, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);
        
        estimate_SIFT_FPFH(msg_left,msg_right,normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

        pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), 
                     good_correspondences (new pcl::Correspondences);

        // rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
        findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);
        rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

        // for (const auto& corr : (*good_correspondences))
        // std::cerr << corr << std::endl;

        
        pcl::copyPointCloud(*keypoints_src , *keypoints_xyzrgb_src);

        
        pcl::copyPointCloud(*keypoints_tgt , *keypoints_xyzrgb_tgt);

        
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> trans_est;
        

        trans_est.estimateRigidTransformation (*keypoints_xyzrgb_src, *keypoints_xyzrgb_tgt, *good_correspondences, transformation);

        // transformation = rel_tf;
    }

    left_keypts.publish(*keypoints_xyzrgb_src);
    right_keypts.publish(*keypoints_xyzrgb_tgt);
    done+=1;



    // std::cout<<"TF obtained: \n"<<transformation<<'\n';
    // DEFINING MY OWN Tranformation Matrix:
    
    // transformation(0,0)=0.78707972;
    // transformation(0,1)=-0.30138103;
    // transformation(0,2)=0.53821463;
    // transformation(0,3)=-0.5321334;
    
    // transformation(1,0)=0.31636445;
    // transformation(1,1)=0.94625315;
    // transformation(1,2)=0.06721984;
    // transformation(1,3)=-0.12747919 ;
    
    // transformation(2,0)=-0.52954608;
    // transformation(2,1)=0.1173646 ;
    // transformation(2,2)=0.84012291;
    // transformation(2,3)=0.43884472;
    
    // transformation(3,0)=0;
    // transformation(3,1)=0;
    // transformation(3,2)=0;
    // transformation(3,3)=1;

    std::cout<<"TF obtained: \n"<<transformation<<'\n';
    

                                                // icp.transformCloud(*msg_left,*pointcloud,transformation);
    
    // Transform using the obatined rigid transformation matrix
    pcl::transformPointCloud(*left, *pointcloud, transformation);

    (*pointcloud)=(*pointcloud)+(*right);


    // Publish processed point cloud
    pcl_output.publish(pointcloud);
    printf("aligned \n");

}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Rel_TF_node");
    ros::NodeHandle nh("~");
    // ros::Subscriber sub = nh.subscribe<PointCloud>("/cloud_pcl_left", 1, callback);
    pcl_output = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/processed_pcl", 10);
    left_keypts = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/left_keypts", 10);
    right_keypts = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/right_keypts", 10);
    

    message_filters::Subscriber<PointCloud> sub_left_pcl(nh,"/cloud_pcl_left",10);
    message_filters::Subscriber<PointCloud> sub_right_pcl(nh,"/cloud_pcl_right",10);

    TimeSynchronizer<PointCloud, PointCloud> sync(sub_left_pcl, sub_right_pcl,10);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::spin();

// Eigen::umeyama()

}