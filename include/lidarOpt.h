#ifndef _LIDAR_OPTIMIZATION_SE3_H_
#define _LIDAR_OPTIMIZATION_SE3_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

namespace TLS
{
     class PointToLineErr : public ceres::SizedCostFunction<3, 6>
     {
     public:
          PointToLineErr(
              Eigen::Vector3d &curr_point_,
              Eigen::Vector3d &line_point_a_,
              Eigen::Vector3d &line_point_b_,
              double &weight_, double *cost_);

          ~PointToLineErr() override {}

          virtual bool Evaluate(
              double const *const *parameters,
              double *residuals,
              double **jacobians) const;

     public:
          Eigen::Vector3d curr_point;
          Eigen::Vector3d line_point_a;
          Eigen::Vector3d line_point_b;
          double weight;
          mutable double *cost;
     };

     class PointToPlaneErr : public ceres::SizedCostFunction<1, 6>
     {
     public:
          PointToPlaneErr(Eigen::Vector3d curr_point_, Eigen::Vector3d &unit_norm_, double &devia_, double &weight_, double *cost_);

          ~PointToPlaneErr() override {}

          virtual bool Evaluate(
              double const *const *parameters,
              double *residuals,
              double **jacobians) const;

     public:
          Eigen::Vector3d curr_point;
          Eigen::Vector3d unit_norm;
          double devia;
          double weight;
          mutable double *cost;
     };

     class PoseSE3Parameterization : public ceres::LocalParameterization
     {
     public:
          PoseSE3Parameterization() = default;
          ~PoseSE3Parameterization() override = default;

          bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
          bool ComputeJacobian(const double *x, double *jacobian) const override;
          int GlobalSize() const override
          {
               return 6;
          }
          int LocalSize() const override
          {
               return 6;
          }
     };
}
#endif //_LIDAR_OPTIMIZATION_SE3_H_