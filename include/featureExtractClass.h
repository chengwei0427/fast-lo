// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include "TicToc.hpp"

struct VelodynePointXYZIRT
{
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	uint16_t ring;
	float time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
						    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

using CloudType = pcl::PointCloud<VelodynePointXYZIRT>;

class PointConvert
{
public:
	enum Format
	{
		UNKOWN,
		VELODYNE,
		// OUSTER,
		// ROBOSENSE
	};
	using Format = PointConvert::Format;
	PointConvert();
	void setFormat(Format format) { _format = format; }
	void convert(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud,
			   const bool re_sort) const;

private:
	void _downsample(CloudType &cloud, const int skip) const;
	void _convert_default(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const;
	void _convert_velodyne(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const;
	void _sort_cloud(CloudType &cloud) const;
	void _re_calc_point_stamp(CloudType &cloud) const;

	template <typename PointT>
	void remove_bad_points(pcl::PointCloud<PointT> &cloud) const
	{
		size_t i = 0;
		for (const auto &p : cloud)
		{
			if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)))
				continue;
			if (p.x * p.x + p.y * p.y + p.z * p.z == 0.0)
				continue;
			cloud[i++] = p;
		}

		if (i != cloud.size())
		{
			cloud.resize(i);
		}

		cloud.is_dense = true;
		cloud.height = 1;
		cloud.width = static_cast<uint32_t>(i);
	}

	template <typename T>
	void _copy_xyzi(const T &p_in, CloudType::PointType &p_out) const
	{
		p_out.x = p_in.x;
		p_out.y = p_in.y;
		p_out.z = p_in.z;
		p_out.intensity = p_in.intensity;
	}

	template <typename T>
	void _copy_meta_and_resize(const T &in, CloudType &out) const
	{
		out.header = in.header;
		out.width = in.width;
		out.height = in.height;
		out.is_dense = in.is_dense;
		out.resize(in.size());
	}

private:
	Format _format = VELODYNE;
};

class ScanExtraction
{
	struct PointInfo
	{
		float range;
		float angle_diff;
	};

	struct CloudExtractInfo
	{
		using Ptr = std::shared_ptr<CloudExtractInfo>;
		std_msgs::Header header;
		CloudType cloud;
		Eigen::Isometry3d pose;
		std::vector<char> label;
		std::vector<std::pair<int, int>> segments;

		void clear()
		{
			cloud.clear();
			label.clear();
			segments.clear();
		}
	}; // struct CloudExractInfo
public:
	ScanExtraction(){};
	void process(CloudType &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf);

private:
	void _fill_info(CloudType &cloud, std::vector<PointInfo> &point_info,
				 std::vector<std::pair<int, int>> &segment) const;
	void _extract(CloudExtractInfo &info, CloudType &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf);
	void _extract_features(const std::vector<PointInfo> &point_info,
					   const std::vector<std::pair<int, int>> &segment,
					   std::vector<char> &label) const;
	void _label_segment(const std::vector<PointInfo> &point_info,
					const int begin_ind, const int end_ind,
					std::vector<char> &label) const;
	float _calc_diff(const float range1, const float range2, const float angle) const
	{
		return atan((range2 - range1) / (range1 * angle));
	}

	float _calc_angel(const CloudType::PointType &p1, const CloudType::PointType &p2) const
	{
		return std::acos(Eigen::Vector3f(p1.x, p1.y, p1.z).normalized().dot(Eigen::Vector3f(p2.x, p2.y, p2.z).normalized()));
	}

	float _calc_dist(const CloudType::PointType &p) const
	{
		return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
	}

private:
	static constexpr char LABEL_SHARP = 255;
	static constexpr char LABEL_FLAT = 128;
	int MIN_SEG_LEN = 5;
	double MIN_POINT_DIST = 1.0;
	float SMOOTH_THRESHOLD = 50.0 * M_PI / 180.0;
	float SHARP_THRESHOLD = 60.0 * M_PI / 180.0;
	int NEIGHBOR_RADIUS = 5;
	float ANGLE_DIFF_THREHOLD = 6.0 * M_PI / 180.0;
	float DISTANCE_DIFF_THRESHOLD = 1;
};
#endif // _LASER_PROCESSING_CLASS_H_
