#include "lidarOpt.h"

namespace TLS
{
     PointToLineErr::PointToLineErr(Eigen::Vector3d &curr_point_, Eigen::Vector3d &line_point_a_,
                                    Eigen::Vector3d &line_point_b_, double &weight_, double *cost_)
         : curr_point(curr_point_), line_point_a(line_point_a_), line_point_b(line_point_b_), weight(weight_), cost(cost_)
     {
     }

     bool PointToLineErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
     {
          Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(parameters[0]);

          Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);

          Eigen::Vector3d curr_world = T * curr_point;

          Eigen::Vector3d nu = (curr_world - line_point_a).cross(curr_world - line_point_b);
          Eigen::Vector3d de = line_point_a - line_point_b;

          residuals[0] = nu.x() / de.norm() * weight;
          residuals[1] = nu.y() / de.norm() * weight;
          residuals[2] = nu.z() / de.norm() * weight;

          *cost = std::pow((residuals[0] + residuals[1] + residuals[2]), 2);

          if (jacobians != nullptr)
          {
               if (jacobians[0] != nullptr)
               {
                    Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_se3_(jacobians[0]);

                    Eigen::Matrix<double, 3, 6> dt_by_se3 = Eigen::Matrix<double, 3, 6>::Zero();

                    dt_by_se3.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * weight;
                    dt_by_se3.block(0, 3, 3, 3) = -Sophus::SO3d::hat(curr_world) * weight;

                    Eigen::Vector3d re = line_point_b - line_point_a;
                    auto skew_re = Sophus::SO3d::hat(re);

                    J_se3_ = skew_re * dt_by_se3 / de.norm();
               }
          }

          return true;
     }

     PointToPlaneErr::PointToPlaneErr(Eigen::Vector3d curr_point_, Eigen::Vector3d &unit_norm_, double &devia_,
                                      double &weight_, double *cost_)
         : curr_point(curr_point_), unit_norm(unit_norm_), devia(devia_), weight(weight_), cost(cost_)
     {
     }

     bool PointToPlaneErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
     {
          Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(parameters[0]);
          Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);
          Eigen::Vector3d curr_world = T * curr_point;
          residuals[0] = unit_norm.dot(curr_world) + devia;
          *cost = std::pow(residuals[0], 2);

          if (jacobians != nullptr)
          {
               if (jacobians[0] != nullptr)
               {
                    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J_se3_(jacobians[0]);

                    Eigen::Matrix<double, 3, 6> dt_by_se3 = Eigen::Matrix<double, 3, 6>::Zero();

                    dt_by_se3.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * weight;
                    dt_by_se3.block(0, 3, 3, 3) = -Sophus::SO3d::hat(curr_world) * weight;

                    J_se3_.block<1, 6>(0, 0) = unit_norm.transpose() * dt_by_se3;
               }
          }

          return true;
     }

     bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
     {
          Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_pose(x);
          Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3_delta(delta);
          Eigen::Map<Eigen::Matrix<double, 6, 1>> se3_x_plus_delta(x_plus_delta);

          Sophus::SE3d T = Sophus::SE3d::exp(se3_pose);
          Sophus::SE3d delta_T = Sophus::SE3d::exp(se3_delta);
          // Use left perturbation in the global coordinate system, use right perturbation in the local coordinate system
          se3_x_plus_delta = (delta_T * T).log();

          return true;
     }

     bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
     {
          ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);

          return true;
     }
}