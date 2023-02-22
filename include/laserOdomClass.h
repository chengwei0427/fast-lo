// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_ODOM_CLASS_H_
#define _LASER_ODOM_CLASS_H_

// std lib
#include <string>
#include <math.h>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

// ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// LOCAL LIB
#include "TicToc.hpp"
#include "lidarOptimization.h"
#include "ikd-Tree/ikd_Tree.h"

struct laserOdomParams
{
	double edge_res, surf_res, sampling_rate;
	double edge_noise_threshold, plane_noise_threshold;
	double min_noise_prior;
	double cube_len;
	laserOdomParams() {}
};
using FactorParam = Eigen::Matrix<double, 3, 4>;
typedef vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> PointVector;

class LaserOdomClass
{

public:
	LaserOdomClass();

	void init(laserOdomParams &lidar_param);
	void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in);
	void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in);
	pcl::PointCloud<pcl::PointXYZI>::Ptr getEdgeMap();
	pcl::PointCloud<pcl::PointXYZI>::Ptr getSurfMap();
	std::vector<FactorParam> getEdgeFactor() { return edge_factors; }
	std::vector<FactorParam> getSurfFactor() { return plane_factors; }
	std::vector<FactorParam> getDegenerateFactor() { return degenerate_factors; }

	Eigen::Isometry3d odom;

private:
	// optimization variable
	double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
	Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
	Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

	Eigen::Isometry3d last_odom;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;

	pcl::PointCloud<pcl::PointXYZI>::Ptr edge_ds, surf_ds;
	pcl::PointCloud<pcl::PointXYZI>::Ptr edge_world, surf_world;

	pcl::VoxelGrid<pcl::PointXYZI> downFilter;

	// local map

	KD_TREE<pcl::PointXYZI> ikdtree_surf, ikdtree_edge;
	vector<vector<int>> pointSearchInd_surf, pointSearchInd_edge;
	vector<BoxPointType> cub_needrm;
	vector<PointVector> Nearest_Points_surf, Nearest_Points_edge;
	BoxPointType LocalMap_Points;

	float DET_RANGE = 300.0f;
	const float MOV_THRESHOLD = 1.5f;

	// optimization count
	int optimization_count;
	laserOdomParams _params;
	TicToc ttc;
	std::vector<FactorParam> edge_factors, plane_factors, degenerate_factors;

	// function
	void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, ceres::Problem &problem, ceres::LossFunction *loss_function);
	void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, ceres::Problem &problem, ceres::LossFunction *loss_function);
	void pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
	void downSamplingScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out,
					  const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out);
	void downSampling(pcl::VoxelGrid<pcl::PointXYZI> &filter, const pcl::PointCloud<pcl::PointXYZI>::Ptr &c_in, double &leaf_size, pcl::PointCloud<pcl::PointXYZI>::Ptr &out);
	void lasermap_fov_segment();
	void map_incremental();
	float calc_dist(pcl::PointXYZI p1, pcl::PointXYZI p2)
	{
		float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
		return d;
	}
};

#endif // _LASER_ODOM_CLASS_H_
