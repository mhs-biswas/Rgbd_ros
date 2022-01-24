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

#include <iostream>
#include <string>

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
ros::Publisher  pcl_output_gray;
int done =0;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
Eigen::Matrix4d rel_tf = Eigen::Matrix4d::Identity();

Eigen::MatrixXd Left_points;
Eigen::MatrixXd right_points;

cv::Size        m_patternSize = cv::Size(6, 9);
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
        cv::cornerSubPix(image, imagePoints, cv::Size(6,9), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::drawChessboardCorners(image,m_patternSize,imagePoints,found);

        if(imagePoints.front().y > imagePoints.back().y){
        std::cout << "Reverse order\n";
        std::reverse(imagePoints.begin(),imagePoints.end());
        }

      //   for(size_t r=0;r<m_patternSize.height;r++){
      //   for(size_t c=0;c<m_patternSize.width;c++){
      //       std::ostringstream oss;
      //       cv::Point2f pt= imagePoints[r*m_patternSize.width+c];
      //       oss << "("<<pt.x<<","<<pt.y<<")";
      //       cv::Point2f test_pt;
      //       test_pt.x=pt.x;
      //       test_pt.y=pt.y;
      //       cv::putText(image, oss.str(), imagePoints[r*m_patternSize.width+c], cv::FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1);
      //       cv::putText(image, oss.str(), test_pt, cv::FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 1);
      //       cv::imshow("image_chessboard", image);
      //       cv::waitKey(0);
      //   }
      // }
    }

    return found;
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
                    // cv::Mat projectedPoints;
                    // cv::projectPoints(pointcloudVec, m_rotation, m_rotation* m_translation, m_colorCameraMatrix, m_colorDistortion, projectedPoints);

                    const cv::Vec3b invalidPoint(0, 0, 0);
                    cv::Rect colorRect(cv::Point(), color.size());
                    for (int row = 0; row < height; ++row) {
                        for (int col = 0; col < width; ++col) {
                            const int idx = row * width + col;
                            const cv::Vec3f& point3d = pointcloudVec.at<cv::Vec3f>(idx);
                            if (point3d[2] != 0.0f) {
                                const cv::Point2f& colorPoint = pointcloudVec.at<cv::Point2f>(idx);
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_gray_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void callback(const PointCloud::ConstPtr& msg_left, 
              const PointCloud::ConstPtr& msg_right,
              const sensor_msgs::ImageConstPtr& blaze_left_pointcloud_raw,
              const sensor_msgs::ImageConstPtr& blaze_right_pointcloud_raw,
              const sensor_msgs::ImageConstPtr& blaze_left_IR,
              const sensor_msgs::ImageConstPtr& blaze_right_IR)
{   

    /*
        msg_left is my source
        msg_right is my target
    */

    cv::Mat blaze_PCL_left= cv_bridge::toCvShare(blaze_left_pointcloud_raw, "rgb8")->image;
    cv::Mat blaze_PCL_right= cv_bridge::toCvShare(blaze_right_pointcloud_raw, "rgb8")->image;
    cv::Mat blazeImg_left= cv_bridge::toCvShare(blaze_left_IR, "mono8")->image;
    cv::Mat blazeImg_right= cv_bridge::toCvShare(blaze_right_IR, "mono8")->image;
        
     

    cv::imshow("Blaze_left_pcl", blaze_PCL_left);
    cv::imshow("Blaze_right_pcl", blaze_PCL_right);
    cv::imshow("Blaze_left_IR", blazeImg_left);
    cv::imshow("Blaze_right_IR", blazeImg_right);
    char key = static_cast<char>(cv::waitKey(1));
    

    if('s' == key)
    {
      std::cout<<"pcl_left_size: "<<blaze_PCL_left.size()<<'\n';
      std::cout<<"IR Image_size: "<<blazeImg_left.size()<<'\n';

      // point_gray_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(msg_left->width, msg_left->height));
      // point_gray_cloud->header.frame_id = "map";

      std::vector<cv::Point2f> LeftCam_corner_pts;
      const bool blazeFound_left = locateChessboardCorners(blazeImg_left, LeftCam_corner_pts);
      std::vector<cv::Point2f> RightCam_corner_pts;
      const bool blazeFound_right = locateChessboardCorners(blazeImg_right, RightCam_corner_pts);

      Eigen::Matrix<float, 3, 54> sub_left_points;
      Eigen::Matrix<float, 3, 54> sub_right_points;
      std::cout<<"LeftCam_corner_pts: "<<LeftCam_corner_pts<<'\n';

      std::cout<<"RightCam_corner_pts: "<<RightCam_corner_pts<<'\n';

      if(blazeFound_left && blazeFound_right)
      { int c=0;
        for(int i=0;i<LeftCam_corner_pts.size();i++)
        { 
          std::ostringstream oss;
          oss << "("<<int(LeftCam_corner_pts[i].x)<<","<<int(LeftCam_corner_pts[i].y)<<")";

          const cv::Vec3f point3d = blaze_PCL_left.at<cv::Vec3f>(int(LeftCam_corner_pts[i].y), int(LeftCam_corner_pts[i].x))/ 1000.0f;
          cv::putText(blazeImg_left, oss.str(), LeftCam_corner_pts[i], cv::FONT_HERSHEY_PLAIN, 1.5, (254, 0, 0), 1);
          cv::imshow("left", blazeImg_left);
          cv::waitKey(0);
          sub_left_points(0,c)= point3d[0];
          sub_left_points(1,c)= point3d[1];
          sub_left_points(2,c)= point3d[2];
          c++;

        }
        std::cout<< "Sub_left \n"<<sub_left_points <<'\n';
        c=0;
        for (int i=RightCam_corner_pts.size()-1;i>=0;i--)
        { 
          std::ostringstream oss_new;
          oss_new << "("<<int(RightCam_corner_pts[i].x)<<","<<int(RightCam_corner_pts[i].y)<<")";

          const cv::Vec3f point3d = blaze_PCL_right.at<cv::Vec3f>(int(RightCam_corner_pts[i].y), int(RightCam_corner_pts[i].x))/ 1000.0f;
          cv::putText(blazeImg_right, oss_new.str(), RightCam_corner_pts[i], cv::FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 1);
          cv::imshow("right", blazeImg_right);
          cv::waitKey(0);

          // const cv::Vec3f point3d = blazeImg_right.at<cv::Vec3f>(RightCam_corner_pts[i].x, RightCam_corner_pts[i].y)/ 1000.0f;
          sub_right_points(0,c)= point3d[0];
          sub_right_points(1,c)= point3d[1];
          sub_right_points(2,c)= point3d[2];
          c++;
        }
        
        std::cout<< "Sub_right \n"<<sub_right_points <<'\n';
        cv::destroyWindow("right");
        cv::destroyWindow("left");
      }

      transformation = Eigen::umeyama(sub_left_points, sub_right_points, false);

    }

    printf ("Cloud_left: width = %d, height = %d\n", msg_left->width, msg_left->height);
    printf ("Cloud_right: width = %d, height = %d\n", msg_right->width, msg_right->height);
                                                        
    pointcloud = PointCloud::Ptr(new PointCloud(msg_left->width, msg_left->height));
    pointcloud->header.frame_id = "map";    
    std::cout<<"TF obtained: \n"<<transformation<<'\n';
    // Transform using the obatined rigid transformation matrix
    pcl::transformPointCloud(*msg_left, *pointcloud, transformation);

    (*pointcloud)=(*pointcloud)+(*msg_right);


    // Publish processed point cloud
    pcl_output.publish(pointcloud);
    printf("aligned \n");

}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Rel_TF_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("Blaze_left_pcl");
    cv::namedWindow("Blaze_right_pcl");
    cv::namedWindow("Blaze_left_IR");
    cv::namedWindow("Blaze_right_IR");

    // ros::Subscriber sub = nh.subscribe<PointCloud>("/cloud_pcl_left", 1, callback);
    pcl_output = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/processed_pcl", 10);
    pcl_output_gray = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/left_gray_pcl", 10);

    message_filters::Subscriber<PointCloud> sub_left_pcl(nh,"/cloud_pcl_left",10);
    message_filters::Subscriber<PointCloud> sub_right_pcl(nh,"/cloud_pcl_right",10);

    message_filters::Subscriber<sensor_msgs::Image> sub_blaze_left_pointcloud(nh,"/Blaze_node_left/Blaze/pointcloud",10);
    message_filters::Subscriber<sensor_msgs::Image> sub_blaze_right_pointcloud(nh,"/Blaze_node_right/Blaze/pointcloud",10);

    message_filters::Subscriber<sensor_msgs::Image> sub_blaze_right_IR(nh,"/Blaze_node_right/Blaze/intensity_image",10);
    message_filters::Subscriber<sensor_msgs::Image> sub_blaze_left_IR(nh,"/Blaze_node_left/Blaze/intensity_image",10);

    TimeSynchronizer<PointCloud, PointCloud, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(sub_left_pcl, sub_right_pcl, sub_blaze_left_pointcloud, sub_blaze_right_pointcloud, sub_blaze_left_IR, sub_blaze_right_IR,10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));


    ros::spin();
    cv::destroyWindow("Blaze_left_pcl");
    cv::destroyWindow("Blaze_right_pcl");
    cv::destroyWindow("Blaze_left_IR");
    cv::destroyWindow("Blaze_right_IR");

// Eigen::umeyama()

}