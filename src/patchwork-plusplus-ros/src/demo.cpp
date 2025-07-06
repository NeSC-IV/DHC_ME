#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include "patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZI;
using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;
ros::Publisher pub_reg_ground;
ros::Publisher pub_reg_non_ground;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> local_cloud_sub_;
std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> local_odom_sub_;
typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
SynchronizerLocalCloudOdom sync_local_cloud_odom_;

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;

    pcl::fromROSMsg(*cloud_msg, pc_curr);

    PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    ROS_INFO_STREAM("\033[1;32m" << "Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
         << " (running_time: " << time_taken << " sec)" << "\033[0m");

    pub_cloud.publish(cloud2msg(pc_curr, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_ground.publish(cloud2msg(pc_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
}

void pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &scanIn,
                                        const nav_msgs::OdometryConstPtr &input) {
    double time_taken;

    tf::Quaternion quaternion(input->pose.pose.orientation.x, input->pose.pose.orientation.y,
                              input->pose.pose.orientation.z, input->pose.pose.orientation.w);
    tf::Vector3 vector3(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);

    // 从sensor坐标系变换到map坐标系的齐次变换矩阵
    tf::Transform T_B_Bi(quaternion, vector3);

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    pcl::PointCloud<PointType> reg_pc_ground;
    pcl::PointCloud<PointType> reg_pc_non_ground;

    pcl::fromROSMsg(*scanIn, pc_curr);

    PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    ROS_INFO_STREAM("\033[1;32m" << "Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
         << " (running_time: " << time_taken << " sec)" << "\033[0m");

    Eigen::Matrix4f pose;
    pose << T_B_Bi.getBasis()[0][0], T_B_Bi.getBasis()[0][1], T_B_Bi.getBasis()[0][2], T_B_Bi.getOrigin()[0],
            T_B_Bi.getBasis()[1][0], T_B_Bi.getBasis()[1][1], T_B_Bi.getBasis()[1][2], T_B_Bi.getOrigin()[1],
            T_B_Bi.getBasis()[2][0], T_B_Bi.getBasis()[2][1], T_B_Bi.getBasis()[2][2], T_B_Bi.getOrigin()[2],
            0, 0, 0, 1;

    for (auto &point: pc_ground.points) {
        pcl::PointXYZI reg_point;
        reg_point.x = point.x * pose(0, 0) + point.y * pose(0, 1) + point.z * pose(0, 2) + pose(0, 3);
        reg_point.y = point.x * pose(1, 0) + point.y * pose(1, 1) + point.z * pose(1, 2) + pose(1, 3);
        reg_point.z = point.x * pose(2, 0) + point.y * pose(2, 1) + point.z * pose(2, 2) + pose(2, 3);
        reg_point.intensity = point.intensity;
        reg_pc_ground.points.push_back(reg_point);
    }

    for (auto &point: pc_non_ground.points) {
        pcl::PointXYZI reg_point;
        reg_point.x = point.x * pose(0, 0) + point.y * pose(0, 1) + point.z * pose(0, 2) + pose(0, 3);
        reg_point.y = point.x * pose(1, 0) + point.y * pose(1, 1) + point.z * pose(1, 2) + pose(1, 3);
        reg_point.z = point.x * pose(2, 0) + point.y * pose(2, 1) + point.z * pose(2, 2) + pose(2, 3);
        reg_point.intensity = point.intensity;
        reg_pc_non_ground.points.push_back(reg_point);
    }

    pub_cloud.publish(cloud2msg(pc_curr, scanIn->header.stamp, scanIn->header.frame_id));
    pub_ground.publish(cloud2msg(pc_ground, scanIn->header.stamp, scanIn->header.frame_id));
    pub_non_ground.publish(cloud2msg(pc_non_ground, scanIn->header.stamp, scanIn->header.frame_id));
    
    pub_reg_ground.publish(cloud2msg(reg_pc_ground, input->header.stamp, input->header.frame_id));
    pub_reg_non_ground.publish(cloud2msg(reg_pc_non_ground, input->header.stamp, input->header.frame_id));
}

int main(int argc, char**argv) {

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cloud_topic, odom_topic;
    pnh.param<string>("cloud_topic", cloud_topic, "pointcloud");
    pnh.param<string>("odom_topic", odom_topic, "odometry");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));

    pub_cloud       = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 100, true);
    pub_ground      = pnh.advertise<sensor_msgs::PointCloud2>("ground", 100, true);
    pub_non_ground  = pnh.advertise<sensor_msgs::PointCloud2>("nonground", 100, true);
    pub_reg_ground      = pnh.advertise<sensor_msgs::PointCloud2>("reg_ground", 100, true);
    pub_reg_non_ground  = pnh.advertise<sensor_msgs::PointCloud2>("reg_nonground", 100, true);

    // ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

    local_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, cloud_topic, 1));
    local_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic, 100));
    sync_local_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
            SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_));
    sync_local_cloud_odom_->registerCallback(boost::bind(&pointCloudOdomCallback, _1, _2));
    
    ros::spin();

    return 0;
}
