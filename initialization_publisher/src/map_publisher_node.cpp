#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/core/utility.hpp"
#include <cv_bridge/cv_bridge.h>
#include "file_io.hpp"

using namespace std;
using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void pubTF(const Pose3D pose, const std_msgs::Header &header)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t = pose.translation();
    Quaterniond correct_q = pose.rotation();

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));
}

void error_compute(std::map<uint32_t, Pose3D> &GT, std::map<uint32_t, Pose3D> &Est)
{
    int N = 633;

    double travel_dist=0;
    double trans_err[N];
    Vec3 prevP;
    for (size_t i=0; i<N; i++)
    {
        Pose3D gtP=GT.at(i);
        Pose3D estP=Est.at(i);
        trans_err[i]=(gtP.translation()-estP.translation()).norm();
        if(i==0)
            prevP=gtP.translation();
        double delta_trans = (gtP.translation()-prevP).norm();
        travel_dist+=delta_trans;

        prevP = gtP.translation();
    }
    ROS_INFO("Travel Distance: %f", travel_dist);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_publisher");
    ros::NodeHandle n("~");

    string dataDir, GT_pose_file, Est_pose_file, cloud_name;
    n.param("data_dir", dataDir, std::string(""));
    char data_dir[1024];
    strcpy(data_dir, dataDir.c_str());
    n.param("cloud_name", cloud_name, std::string(""));
    n.param("gt_pose", GT_pose_file, std::string(""));
    n.param("est_pose", Est_pose_file, std::string(""));
    std::cout << "load point clouds " << cloud_name << std::endl;
    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPLYFile(cloud_name, *cloud);
    if (cloud->points.size() == 0)
    {
        ROS_WARN("can't load point clouds; wrong path");
        return 0;
    }
    else
    {
        ROS_INFO("Point cloud data: %d", (int)cloud->points.size());
    }

    std::map<uint32_t, Pose3D> gtPoses, estPoses;
    readData(GT_pose_file,',', 1, gtPoses);
    readData(Est_pose_file,',', 1, estPoses);
    error_compute(gtPoses, estPoses);
    ros::Publisher pub_3dmap, pub_gt_pose, pub_estpose, pub_gt_path, pub_est_path, pub_img;

    pub_gt_pose = n.advertise<nav_msgs::Odometry>("/benchmark_publisher/gt_pose", 1000);
    pub_gt_path = n.advertise<nav_msgs::Path>("/benchmark_publisher/gt_path", 1000);
    pub_estpose= n.advertise<nav_msgs::Odometry>("/benchmark_publisher/est_pose", 1000);
    pub_est_path = n.advertise<nav_msgs::Path>("/benchmark_publisher/est_path", 1000);
    pub_3dmap =  n.advertise<pcl::PointCloud<pcl::PointXYZ>>("/benchmark_publisher/map_clouds",1000);
    pub_img = n.advertise<sensor_msgs::Image>("/benchmark_publisher/img",1000);
    
    nav_msgs::Path gt_path,est_path;


    ros::Rate r(5); // 10 hz
    for (size_t i = 0; i < estPoses.size(); i++)
    {
        if(i<5)
        {
            pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
            cloud->header.frame_id = "world";
            pub_3dmap.publish(cloud);
        }
        Pose3D gtP=gtPoses.at(i);
        nav_msgs::Odometry gt_pose;
        gt_pose.header.stamp = ros::Time::now();
        gt_pose.header.frame_id = "world";
        gt_pose.child_frame_id = "world";
        gt_pose.pose.pose.position.x = gtP.translation().x();
        gt_pose.pose.pose.position.y = gtP.translation().y();
        gt_pose.pose.pose.position.z = gtP.translation().z();
        gt_pose.pose.pose.orientation.w = gtP.rotation().w();
        gt_pose.pose.pose.orientation.x = gtP.rotation().x();
        gt_pose.pose.pose.orientation.y = gtP.rotation().y();
        gt_pose.pose.pose.orientation.z = gtP.rotation().z();
        pub_gt_pose.publish(gt_pose);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = gt_pose.header;
        pose_stamped.pose = gt_pose.pose.pose;
        gt_path.header = gt_pose.header;
        gt_path.poses.push_back(pose_stamped);
        pub_gt_path.publish(gt_path);

        pubTF(gtP, gt_pose.header);

        Pose3D estP=estPoses.at(i);
        nav_msgs::Odometry est_pose;
        est_pose.header.stamp = gt_pose.header.stamp;
        est_pose.header.frame_id = "world";
        est_pose.child_frame_id = "world";
        est_pose.pose.pose.position.x = estP.translation().x();
        est_pose.pose.pose.position.y = estP.translation().y();
        est_pose.pose.pose.position.z = estP.translation().z();
        est_pose.pose.pose.orientation.w = estP.rotation().w();
        est_pose.pose.pose.orientation.x = estP.rotation().x();
        est_pose.pose.pose.orientation.y = estP.rotation().y();
        est_pose.pose.pose.orientation.z = estP.rotation().z();
        pub_gt_pose.publish(est_pose);

        geometry_msgs::PoseStamped est_pose_stamped;
        est_pose_stamped.header = est_pose.header;
        est_pose_stamped.pose = est_pose.pose.pose;
        est_path.header = est_pose.header;
        est_path.poses.push_back(est_pose_stamped);
        pub_est_path.publish(est_path);

        char img_file[100];
        sprintf(img_file,"%s/image/%06d_est.png",data_dir, i);
        string img_name(img_file);
        cv::Mat img=cv::imread(img_name);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(gt_pose.header, "bgr8", img).toImageMsg();
        pub_img.publish(msg);


        ros::spinOnce();
        r.sleep();
    }
}
