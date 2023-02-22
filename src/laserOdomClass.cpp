// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserOdomClass.h"

void LaserOdomClass::init(laserOdomParams &params)
{
    _params = params;
    // init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count = 2;
}

void LaserOdomClass::downSampling(pcl::VoxelGrid<pcl::PointXYZI> &filter, const pcl::PointCloud<pcl::PointXYZI>::Ptr &in, double &leaf_size, pcl::PointCloud<pcl::PointXYZI>::Ptr &out)
{
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.setInputCloud(in);
    filter.filter(*out);
}

void LaserOdomClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in)
{
    if (ikdtree_edge.Root_Node == nullptr)
    {
        ikdtree_edge.set_downsample_param(_params.edge_res);
        ikdtree_edge.Build(edge_in->points);
    }
    if (ikdtree_surf.Root_Node == nullptr)
    {
        ikdtree_surf.set_downsample_param(_params.surf_res);
        ikdtree_surf.Build(surf_in->points);
    }
    std::cout << __FUNCTION__ << ", map num: " << ikdtree_edge.validnum() << ", " << ikdtree_surf.validnum() << std::endl;
    optimization_count = 12;
}

void LaserOdomClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in)
{

    if (optimization_count > 2)
        optimization_count--;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    lasermap_fov_segment();

    edge_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());
    surf_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());
    edge_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
    surf_world.reset(new pcl::PointCloud<pcl::PointXYZI>());

    downSamplingScan(edge_in, edge_ds, surf_in, surf_ds);

    edge_world->resize(edge_ds->size()), surf_world->resize(surf_ds->size());
    Nearest_Points_edge.resize(edge_ds->size());
    Nearest_Points_surf.resize(surf_ds->size());
    // ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if (ikdtree_edge.validnum() > 10 && ikdtree_surf.validnum() > 50)
    {

        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

            addEdgeCostFactor(edge_ds, problem, loss_function);

            addSurfCostFactor(surf_ds, problem, loss_function);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
        }
    }
    else
    {
        printf("not enough points in map to associate, map error");
    }
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;
    ttc.tic();
    map_incremental();
    // addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
    double t = ttc.toc();
    std::cout << "add point takes: " << t << "ms" << std::endl;
}

void LaserOdomClass::lasermap_fov_segment()
{
    cub_needrm.clear();
    Eigen::Vector3d pos_LiD = t_w_curr;
    static bool Localmap_Initialized = false;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - _params.cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + _params.cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((_params.cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    PointVector points_history, points_history_surf;
    ikdtree_edge.acquire_removed_points(points_history);
    ikdtree_surf.acquire_removed_points(points_history_surf);

    if (cub_needrm.size() > 0)
    {
        ikdtree_edge.Delete_Point_Boxes(cub_needrm);
        ikdtree_surf.Delete_Point_Boxes(cub_needrm);
    }
}

void LaserOdomClass::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    //  edge
    PointToAdd.reserve(edge_ds->size());
    PointNoNeedDownsample.reserve(edge_ds->size());
    for (int i = 0; i < edge_ds->size(); i++)
    {
        pointAssociateToMap(&(edge_ds->points[i]), &(edge_world->points[i]));
        if (!Nearest_Points_edge[i].empty())
        {
            const PointVector &points_near = Nearest_Points_edge[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            pcl::PointXYZI downsample_result, mid_point;
            mid_point.x = floor(edge_world->points[i].x / _params.edge_res) * _params.edge_res + 0.5 * _params.edge_res;
            mid_point.y = floor(edge_world->points[i].y / _params.edge_res) * _params.edge_res + 0.5 * _params.edge_res;
            mid_point.z = floor(edge_world->points[i].z / _params.edge_res) * _params.edge_res + 0.5 * _params.edge_res;
            float dist = calc_dist(edge_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * _params.edge_res && fabs(points_near[0].y - mid_point.y) > 0.5 * _params.edge_res && fabs(points_near[0].z - mid_point.z) > 0.5 * _params.edge_res)
            {
                PointNoNeedDownsample.push_back(edge_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < 10; readd_i++)
            {
                if (points_near.size() < 10)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(edge_world->points[i]);
        }
        else
            PointToAdd.push_back(edge_world->points[i]);
    }
    ikdtree_edge.Add_Points(PointToAdd, true);
    ikdtree_edge.Add_Points(PointNoNeedDownsample, false);
    // surf
    PointVector PointToAdd_surf;
    PointVector PointNoNeedDownsample_surf;
    PointToAdd_surf.reserve(surf_ds->size());
    PointNoNeedDownsample_surf.reserve(surf_ds->size());
    for (int i = 0; i < surf_ds->size(); i++)
    {
        pointAssociateToMap(&(surf_ds->points[i]), &(surf_world->points[i]));
        if (!Nearest_Points_surf[i].empty())
        {
            const PointVector &points_near = Nearest_Points_surf[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            pcl::PointXYZI downsample_result, mid_point;
            mid_point.x = floor(surf_world->points[i].x / _params.surf_res) * _params.surf_res + 0.5 * _params.surf_res;
            mid_point.y = floor(surf_world->points[i].y / _params.surf_res) * _params.surf_res + 0.5 * _params.surf_res;
            mid_point.z = floor(surf_world->points[i].z / _params.surf_res) * _params.surf_res + 0.5 * _params.surf_res;
            float dist = calc_dist(surf_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * _params.surf_res && fabs(points_near[0].y - mid_point.y) > 0.5 * _params.surf_res && fabs(points_near[0].z - mid_point.z) > 0.5 * _params.surf_res)
            {
                PointNoNeedDownsample_surf.push_back(surf_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < 5; readd_i++)
            {
                if (points_near.size() < 5)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd_surf.push_back(surf_world->points[i]);
        }
        else
            PointToAdd_surf.push_back(surf_world->points[i]);
    }
    ikdtree_surf.Add_Points(PointToAdd_surf, true);
    ikdtree_surf.Add_Points(PointNoNeedDownsample_surf, false);
}

void LaserOdomClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    // po->intensity = 1.0;
}

void LaserOdomClass::downSamplingScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out,
                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out)
{
    double res_edge = _params.edge_res * _params.sampling_rate;
    double res_surf = _params.surf_res * _params.sampling_rate;
    downSampling(downFilter, edge_pc_in, res_edge, edge_pc_out);
    downSampling(downFilter, surf_pc_in, res_surf, surf_pc_out);
    // downSizeFilterEdge.setInputCloud(edge_pc_in);
    // downSizeFilterEdge.filter(*edge_pc_out);
    // downSizeFilterSurf.setInputCloud(surf_pc_in);
    // downSizeFilterSurf.filter(*surf_pc_out);
}

void LaserOdomClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    edge_factors.clear();
    degenerate_factors.clear();
    constexpr int K = 10;
    int corner_num = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        vector<float> pointSearchSqDis(K);
        auto &points_near = Nearest_Points_edge[i];

        ikdtree_edge.Nearest_Search(point_temp, K, points_near, pointSearchSqDis);

        if (points_near.size() >= K && pointSearchSqDis[K - 1] < 1.0)
        {
            Eigen::Matrix<double, K, 3> near_points;
            for (int j = 0; j < K; ++j)
                near_points.row(j) << points_near[j].x, points_near[j].y, points_near[j].z;

            Eigen::Vector3d center = near_points.colwise().sum() / K;
            near_points.rowwise() -= center.transpose();
            Eigen::Matrix3d covMat = near_points.transpose() * near_points;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
            Eigen::Vector3d eigen_val = saes.eigenvalues().cwiseSqrt() / std::sqrt(K - 1); //  特征值，从小到大

            const auto sigma = std::sqrt(eigen_val[0] * eigen_val[0] + eigen_val[1] * eigen_val[1]);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);

            if (eigen_val[0] < _params.plane_noise_threshold && eigen_val[1] * 3 > _params.edge_res &&
                eigen_val[2] * 3 > _params.edge_res && std::abs(center.normalized().dot(saes.eigenvectors().col(0).normalized())) > std::cos(M_PI * 17.0 / 36.0))
            {
                FactorParam param;
                param.col(0) = center;
                param.col(1) = saes.eigenvectors().col(0).normalized(); //  最小特征值，对应平面法向量
                param.col(2) << pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z;
                param.col(3)[0] = std::max(std::sqrt(eigen_val[0] * eigen_val[0] + _params.plane_noise_threshold * _params.plane_noise_threshold), _params.min_noise_prior);
                // param.col(3)[0] = std::max(eigen_val[0] + _plane_noise_threshold, _min_noise_prior);
                degenerate_factors.push_back(param);
            }
            else if (sigma < _params.edge_noise_threshold && 1.5 * sigma < eigen_val[2])
            {
                FactorParam param;
                param.col(0) = center;
                param.col(1) = saes.eigenvectors().col(2).normalized(); //   最大特征值，对应直线方向
                param.col(2) << pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z;
                param.col(3)[0] = std::max(sigma, _params.min_noise_prior);
                edge_factors.push_back(param);

                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;
            }
        }
    }
    if (corner_num < 20)
    {
        printf("not enough edge points");
    }
}

void LaserOdomClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    plane_factors.clear();
    constexpr int K = 5;
    int surf_num = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        vector<float> pointSearchSqDis(K);
        auto &points_near = Nearest_Points_surf[i];

        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        ikdtree_surf.Nearest_Search(point_temp, K, points_near, pointSearchSqDis);

        Eigen::Matrix<double, K, 3> matA0;
        Eigen::Matrix<double, K, 1> matB0 = -1 * Eigen::Matrix<double, K, 1>::Ones();
        if (points_near.size() >= K /*&& pointSearchSqDis[K - 1] < 1.0*/)
        {
            Eigen::Matrix<double, K, 3> near_points;
            for (int j = 0; j < K; ++j)
                near_points.row(j) << points_near[j].x, points_near[j].y, points_near[j].z;

            Eigen::Vector3d center = near_points.colwise().sum() / K;
            near_points.rowwise() -= center.transpose();
            Eigen::Matrix3d covMat = near_points.transpose() * near_points;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
            Eigen::Vector3d eigen_val = saes.eigenvalues().cwiseSqrt() / std::sqrt(K - 1);

            for (int j = 0; j < K; j++)
            {
                matA0(j, 0) = points_near[j].x;
                matA0(j, 1) = points_near[j].y;
                matA0(j, 2) = points_near[j].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < K; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * points_near[j].x +
                         norm(1) * points_near[j].y +
                         norm(2) * points_near[j].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid && eigen_val[0] < _params.plane_noise_threshold && eigen_val[1] * 3 > _params.surf_res && eigen_val[2] * 3 > _params.surf_res)
            {
                FactorParam param;
                param.col(0) = center;
                param.col(1) = saes.eigenvectors().col(0).normalized(); //  最小特征值，对应平面法向量
                param.col(2) << pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z;
                param.col(3)[0] = std::max(eigen_val[0], _params.min_noise_prior);
                plane_factors.push_back(param);

                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }
    }
    if (surf_num < 20)
    {
        printf("not enough surf points");
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserOdomClass::getEdgeMap()
{

    PointVector().swap(ikdtree_edge.PCL_Storage);
    ikdtree_edge.flatten(ikdtree_edge.Root_Node, ikdtree_edge.PCL_Storage, NOT_RECORD);
    laserCloudCornerMap->clear();
    laserCloudCornerMap->points = ikdtree_edge.PCL_Storage;
    return laserCloudCornerMap;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LaserOdomClass::getSurfMap()
{

    PointVector().swap(ikdtree_surf.PCL_Storage);
    ikdtree_surf.flatten(ikdtree_surf.Root_Node, ikdtree_surf.PCL_Storage, NOT_RECORD);
    laserCloudSurfMap->clear();
    laserCloudSurfMap->points = ikdtree_surf.PCL_Storage;
    return laserCloudSurfMap;
}

LaserOdomClass::LaserOdomClass()
{
}
