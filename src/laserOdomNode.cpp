// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "laserOdomClass.h"

LaserOdomClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;

ros::Publisher pubLaserOdometry;
ros::Publisher pub_selected_degenerate, pub_selected_surf, pub_selected_corner;
ros::Publisher pub_surf_map, pub_edge_map;

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void visual_factors(const std::vector<FactorParam> &params, const ros::Time &stamp,
                    const Eigen::Quaterniond &q, const Eigen::Vector3d &t, double color_feature,
                    double color_point, const ros::Publisher &pub)
{
    if (params.empty())
        return;
    static pcl::PointCloud<pcl::PointXYZI> cloud;
    static sensor_msgs::PointCloud2 msg;
    // static Pose3 pose_map;
    cloud.clear();
    if (pub.getNumSubscribers() > 0)
    {
        // pose_map.setIdentity();
        // _convert_factor_params_to_cloud(params, cloud, pose_map, pose_feature, color_feature, color_point);
        pcl::PointXYZI p;
        p.intensity = color_feature;
        Eigen::Vector3d p1, p2;
        for (const auto &param : params)
        {
            p.x = param.col(0).x();
            p.y = param.col(0).y();
            p.z = param.col(0).z();
            cloud.push_back(p);
            for (int i = 1; i <= 5; ++i)
            {
                p1 = param.col(0) + param.col(1) * 0.1 * i;
                p2 = param.col(0) - param.col(1) * 0.1 * i;
                p.x = p1.x();
                p.y = p1.y();
                p.z = p1.z();
                cloud.push_back(p);
                p.x = p2.x();
                p.y = p2.y();
                p.z = p2.z();
                cloud.push_back(p);
            }
        }

        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "/map";
        msg.header.stamp = stamp;
        pub.publish(msg);
    }
}

void publish_map(const ros::Publisher &pubLaserCloudMap, const ros::Time &stamp,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr &featsFromMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    if (pubLaserCloudMap.getNumSubscribers() > 0)
    {
        pcl::toROSMsg(*featsFromMap, laserCloudMap);
        laserCloudMap.header.stamp = stamp;
        laserCloudMap.header.frame_id = "map";
        pubLaserCloudMap.publish(laserCloudMap);
    }
}

bool is_odom_inited = false;
double total_time = 0;
int total_frame = 0;
void odom_estimation()
{
    TicToc tc;
    while (1)
    {
        if (!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty())
        {

            // read data
            mutex_lock.lock();
            if (!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.05))
            {
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;
            }

            if (!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.05))
            {
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;
            }
            // if time aligned

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if (is_odom_inited == false)
            {
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }
            else
            {
                tc.tic();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                total_frame++;
                float time_temp = tc.toc();
                total_time += time_temp;
                ROS_INFO("average odom estimation time %f ms, current estimate takes %f ms \n", total_time / total_frame, time_temp);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            // q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            visual_factors(odomEstimation.getEdgeFactor(), pointcloud_time, q_current, t_current, 0, 100, pub_selected_corner);
            visual_factors(odomEstimation.getSurfFactor(), pointcloud_time, q_current, t_current, 150, 250, pub_selected_surf);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);
            if (pub_surf_map.getNumSubscribers() > 0)
                publish_map(pub_surf_map, pointcloud_time, odomEstimation.getSurfMap());
            if (pub_edge_map.getNumSubscribers() > 0)
                publish_map(pub_edge_map, pointcloud_time, odomEstimation.getEdgeMap());
        }
        // sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m---->\033[0m LaserOdom Started.");

    double edge_res = 0.2;
    double surf_res = 0.4;
    double sampling_rate = 1.;
    double edge_noise_threshold = 0.3;
    double plane_noise_threshold = 0.05;
    double min_noise_prior = 0.02;
    double cube_len = 200;

    nh.getParam("/edge_resolution", edge_res);
    nh.getParam("/surf_resolution", surf_res);
    nh.getParam("/sampling_rate", sampling_rate);
    nh.getParam("/edge_noise_threshold", edge_noise_threshold);
    nh.getParam("/plane_noise_threshold", plane_noise_threshold);
    nh.getParam("/min_noise_prior", min_noise_prior);
    nh.getParam("/cube_len", cube_len);

    laserOdomParams params;
    params.edge_res = edge_res;
    params.surf_res = surf_res;
    params.sampling_rate = sampling_rate;
    params.edge_noise_threshold = edge_noise_threshold;
    params.plane_noise_threshold = plane_noise_threshold;
    params.min_noise_prior = min_noise_prior;
    params.cube_len = cube_len;

    odomEstimation.init(params);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pub_selected_surf = nh.advertise<sensor_msgs::PointCloud2>("selected_surf", 3, true);
    pub_selected_corner = nh.advertise<sensor_msgs::PointCloud2>("selected_corner", 3, true);
    pub_selected_degenerate = nh.advertise<sensor_msgs::PointCloud2>("selected_degenerate", 3, true);
    pub_edge_map = nh.advertise<sensor_msgs::PointCloud2>("/local_edge_map", 100);
    pub_surf_map = nh.advertise<sensor_msgs::PointCloud2>("/local_surf_map", 100);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
