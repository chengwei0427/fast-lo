// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "featureExtractClass.h"

PointConvert::PointConvert() {}

void PointConvert::convert(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud,
                           const bool re_sort) const
{
    cloud.clear();
    switch (_format)
    {
    case VELODYNE:
        _convert_velodyne(msg, cloud);
        break;
    // case OUSTER:
    //      _convert_ouster(msg, cloud);
    //      break;
    // case ROBOSENSE:
    //     _convert_robosense(msg, cloud);
    //     break;
    default:
        _convert_default(msg, cloud);
        break;
    }
    if (cloud.empty())
        return;
    if (re_sort)
    {
        _sort_cloud(cloud);
        if (_format == VELODYNE)
        {
            _re_calc_point_stamp(cloud);
            _sort_cloud(cloud);
        }
    }
}

void PointConvert::_downsample(CloudType &cloud, const int skip) const
{
    int j = 0;
    const auto size = cloud.size();
    for (int i = 0; i < size; i += skip + 1)
    {
        cloud[j++] = cloud[i];
    }
    cloud.resize(j);
}

void PointConvert::_convert_default(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const
{
    pcl::fromROSMsg(*msg, cloud);
    remove_bad_points(cloud);
}

void PointConvert::_convert_velodyne(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const
{
    _convert_default(msg, cloud);
}

void PointConvert::_sort_cloud(CloudType &cloud) const
{
    std::sort(cloud.begin(), cloud.end(), [](const CloudType::PointType &u, const CloudType::PointType &v)
              {
            if(u.ring != v.ring) return u.ring < v.ring;
            return u.time < v.time; });
}

void PointConvert::_re_calc_point_stamp(CloudType &cloud) const
{
    float min_t, max_t;
    size_t first_ind, last_ind;

    first_ind = last_ind = 0;
    min_t = max_t = cloud[0].time;

    auto last_i = 0;
    const auto size = cloud.size();

    for (size_t i = 1; i < size; ++i)
    {
        if (cloud[i].ring != cloud[last_i].ring || i == size - 1)
        {
            if (i - last_i > 1)
            {
                // clock-wise
                const auto start_angle = -std::atan2(cloud[first_ind].y, cloud[first_ind].x);
                auto end_angle = -std::atan2(cloud[last_ind].y, cloud[last_ind].x) + 2 * M_PI;
                if (end_angle - start_angle > M_PI * 3)
                    end_angle -= 2 * M_PI;
                else if (end_angle - start_angle < M_PI)
                    end_angle += 2 * M_PI;

                auto sec_per_angle = (max_t - min_t) / (end_angle - start_angle);

                bool half_passed = false;
                for (size_t j = last_i; j < i; ++j)
                {
                    auto angle = -std::atan2(cloud[j].y, cloud[j].x);
                    if (!half_passed)
                    {
                        if (angle < start_angle - 0.5 * M_PI)
                            angle += 2 * M_PI;
                        else if (angle > start_angle + 1.5 * M_PI)
                            angle -= 2 * M_PI;

                        if (angle - start_angle > M_PI)
                            half_passed = true;
                    }
                    else
                    {
                        angle += 2 * M_PI;
                        if (angle < end_angle - 1.5 * M_PI)
                            angle += 2 * M_PI;
                        else if (angle > end_angle + 0.5 * M_PI)
                            angle -= 2 * M_PI;
                    }
                    cloud[j].time = min_t + sec_per_angle * (angle - start_angle);
                }
            }

            first_ind = last_ind = i;
            min_t = max_t = cloud[i].time;
            last_i = i;
            continue;
        }

        const auto &p = cloud[i];
        if (p.time < min_t)
        {
            min_t = p.time;
            first_ind = i;
        }
        if (p.time > max_t)
        {
            max_t = p.time;
            last_ind = i;
        }
    }
}

void ScanExtraction::process(CloudType &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf)
{
    std::vector<PointInfo> point_info;
    // std::cout << __FUNCTION__ << ", cloud size: " << cloud.size() << std::endl;
    CloudExtractInfo::Ptr extract_info = std::make_shared<CloudExtractInfo>();
    extract_info->cloud = cloud;
    _fill_info(cloud, point_info, extract_info->segments);
    _extract_features(point_info, extract_info->segments, extract_info->label);
    _extract(*extract_info, cloud, pc_out_edge, pc_out_surf);
}

void ScanExtraction::_fill_info(CloudType &cloud, std::vector<PointInfo> &point_info,
                                std::vector<std::pair<int, int>> &segment) const
{
    const auto size = cloud.size();
    point_info.resize(size);
    segment.clear();
    int last_seg_ind = 0;
    point_info[0].angle_diff = 0;
    point_info[0].range = _calc_dist(cloud[0]);
    for (int i = 1; i < size; ++i)
    {
        point_info[i].range = _calc_dist(cloud[i]);
        if (cloud[i - 1].ring != cloud[i].ring)
        {
            segment.emplace_back(last_seg_ind, i);
            last_seg_ind = i;
            point_info[i].angle_diff = 0;
            continue;
        }
        point_info[i].angle_diff = _calc_angel(cloud[i - 1], cloud[i]);
    }
    segment.emplace_back(last_seg_ind, size);
}

void ScanExtraction::_extract_features(const std::vector<PointInfo> &point_info,
                                       const std::vector<std::pair<int, int>> &segment, std::vector<char> &label) const
{
    std::vector<std::pair<int, int>> cut_point;
    label.resize(point_info.size(), 0);
    for (const auto &seg : segment) //  每一线数据进行处理
    {
        cut_point.resize(0);
        int last_cut_point = seg.first;
        for (int j = seg.first + 1; j < seg.second; ++j) //  找出一段一段的连续数据
        {
            const float range_diff = point_info[j].range - point_info[j - 1].range;
            const bool over_angle_thres = point_info[j].angle_diff > ANGLE_DIFF_THREHOLD;

            if (!over_angle_thres && std::abs(range_diff) < DISTANCE_DIFF_THRESHOLD) //  距离差异和角度差都小于阈值，跳过；否则，则为cut point
                continue;

            cut_point.emplace_back(last_cut_point, j);

            last_cut_point = j;
        }
        cut_point.emplace_back(last_cut_point, seg.second);

        for (int i = 1; i < cut_point.size(); ++i)
        {
            const auto &left = cut_point[i].first;                                 //  起点
            const auto &right = cut_point[i].second;                               //  终点
            const auto diff = point_info[left].range - point_info[left - 1].range; //    距离上一个分段的距离差
            // if(right - left < MIN_SEG_LEN || cut_point[i-1].second - cut_point[i-1].first < MIN_SEG_LEN)
            //     continue;
            if (point_info[left].angle_diff > ANGLE_DIFF_THREHOLD)
                label[left] = label[left - 1] = LABEL_SHARP;
            else
                label[diff < 0 ? left : left - 1] = LABEL_SHARP;
        }

        for (const auto piece : cut_point)
        {
            _label_segment(point_info, piece.first, piece.second, label);
        }
    }
}

void ScanExtraction::_label_segment(const std::vector<PointInfo> &point_info, const int begin_ind, const int end_ind, std::vector<char> &label) const
{
    int seg_len = end_ind - begin_ind;
    if (seg_len < MIN_SEG_LEN)
        return;

    const int radius = std::min(NEIGHBOR_RADIUS, seg_len / 2);
    using IndexedSmoothness = std::pair<int, float>;
    static std::vector<IndexedSmoothness> smoothness;
    static std::vector<int> picked;

    smoothness.resize(0);
    picked.resize(seg_len, 0);

    const int start = begin_ind + radius;
    const int end = end_ind - radius;
    for (int ind = start; ind < end; ++ind)
    {
        const float cur_depth = point_info[ind].range;

        float diff = 0;
        float angle1 = 0;
        float angle2 = 0;

        for (int i = 1; i <= radius; ++i)
        {
            angle1 += point_info[ind + i].angle_diff;
            angle2 += point_info[ind - i + 1].angle_diff;

            float diff1 = _calc_diff(cur_depth, point_info[ind + i].range, angle1);
            float diff2 = _calc_diff(cur_depth, point_info[ind - i + 1].range, angle2);
            diff += (diff1 + diff2);
        }

        diff = std::abs(diff) / radius;
        smoothness.emplace_back(ind, diff);
    }

    std::sort(smoothness.begin(), smoothness.end(), [](const IndexedSmoothness &u, const IndexedSmoothness &v)
              { return u.second < v.second; });

    auto rp = smoothness.rbegin();
    while (rp != smoothness.rend())
    {
        if (rp->second < SHARP_THRESHOLD)
            break;
        const int ind = rp->first;
        const int picked_ind = rp->first - begin_ind;
        if (picked[picked_ind] == 0) //  未访问过
        {
            label[ind] = LABEL_SHARP;
            for (int i = 1; i <= radius; ++i) //  左右都标记
            {
                picked[picked_ind + i] = 1;
                picked[picked_ind - i] = 1;
            }
        }
        ++rp;
    }

    auto p = smoothness.begin();
    while (p != smoothness.end())
    {
        if (p->second > SMOOTH_THRESHOLD)
            break;
        const int ind = p->first;
        const int picked_ind = p->first - begin_ind;
        if (picked[picked_ind] == 0)
        {
            label[ind] = LABEL_FLAT;
            // for(int i = 1; i <= radius; ++i)
            // {
            //     picked[picked_ind + i] = 1;
            //     picked[picked_ind - i] = 1;
            // }
        }
        ++p;
    }
}

void ScanExtraction::_extract(CloudExtractInfo &info, CloudType &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf)
{
    pc_out_edge->clear(), pc_out_surf->clear();
    const auto size = info.label.size();
    pcl::PointXYZI pt;
    for (int i = 0; i < size; ++i)
    {
        pt.x = cloud[i].x, pt.y = cloud[i].y, pt.z = cloud[i].z;
        pt.intensity = cloud[i].intensity;
        if (info.label[i] == LABEL_SHARP)
            pc_out_edge->push_back(pt);
        else if (info.label[i] == LABEL_FLAT)
            pc_out_surf->push_back(pt);
    }
}