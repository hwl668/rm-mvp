#include "math_tools.hpp"

#include <cmath>
#include <opencv2/core.hpp>  // CV_PI

namespace tools
{
double limit_rad(double angle)
{
  while (angle > CV_PI) angle -= 2 * CV_PI;
  while (angle <= -CV_PI) angle += 2 * CV_PI;
  return angle;
}

Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
  if (!extrinsic) std::swap(axis0, axis2);

  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);
  if (is_proper) k = 3 - i - j;
  auto sign = (i - j) * (j - k) * (k - i) / 2;

  double a, b, c, d;
  Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  Eigen::Vector3d eulers;
  auto n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  auto half_sum = std::atan2(b, a);
  auto half_diff = std::atan2(-d, c);

  auto eps = 1e-7;
  auto safe1 = std::abs(eulers[1]) >= eps;
  auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
  auto safe = safe1 && safe2;
  if (safe) {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;
  } else {
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
  }

  for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

  if (!is_proper) {
    eulers[2] *= sign;
    eulers[1] -= CV_PI / 2;
  }

  if (!extrinsic) std::swap(eulers[0], eulers[2]);

  return eulers;
}

Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
  Eigen::Quaterniond q(R);
  return eulers(q, axis0, axis1, axis2, extrinsic);
}

Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)
{
  double roll  = ypr[2];
  double pitch = ypr[1];
  double yaw   = ypr[0];

  double cy = std::cos(yaw),   sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll),  sr = std::sin(roll);

  // z-y-x 旋转顺序
  Eigen::Matrix3d R;
  R << cy * cp,                  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
       sy * cp,                  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
       -sp,                      cp * sr,                 cp * cr;
  return R;
}

Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];

  const double denom_xy = (x * x + y * y);
  const double denom_xyz = std::sqrt(x * x + y * y + z * z);
  const double denom_pitch = (z * z / denom_xy + 1.0);
  const double sqrt_xy = std::sqrt(denom_xy);

  double dyaw_dx = -y / denom_xy;
  double dyaw_dy =  x / denom_xy;
  double dyaw_dz =  0.0;

  double dpitch_dx = -(x * z) / (denom_pitch * std::pow(denom_xy, 1.5));
  double dpitch_dy = -(y * z) / (denom_pitch * std::pow(denom_xy, 1.5));
  double dpitch_dz =  1.0 / (denom_pitch * sqrt_xy);

  double ddistance_dx = x / denom_xyz;
  double ddistance_dy = y / denom_xyz;
  double ddistance_dz = z / denom_xyz;

  Eigen::MatrixXd J(3,3);
  J << dyaw_dx,     dyaw_dy,     dyaw_dz,
       dpitch_dx,   dpitch_dy,   dpitch_dz,
       ddistance_dx,ddistance_dy,ddistance_dz;
  return J;
}


Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  auto x = distance * std::cos(pitch) * std::cos(yaw);
  auto y = distance * std::cos(pitch) * std::sin(yaw);
  auto z = distance * std::sin(pitch);
  return {x, y, z};
}

Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  double cy = std::cos(yaw),   sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);

  double dx_dyaw = distance * cp * (-sy);
  double dy_dyaw = distance * cp * ( cy);
  double dz_dyaw = 0.0;

  double dx_dpitch = distance * (-sp) * cy;
  double dy_dpitch = distance * (-sp) * sy;
  double dz_dpitch = distance *  cp;

  double dx_ddistance = cp * cy;
  double dy_ddistance = cp * sy;
  double dz_ddistance = sp;

  Eigen::MatrixXd J(3,3);
  J << dx_dyaw,      dx_dpitch,      dx_ddistance,
       dy_dyaw,      dy_dpitch,      dy_ddistance,
       dz_dyaw,      dz_dpitch,      dz_ddistance;
  return J;
}

double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)
{
  std::chrono::duration<double> c = a - b;
  return c.count();
}

double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)
{
  if (vec1.norm() == 0. || vec2.norm() == 0.) {
    return 0.;
  }
  return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

double limit_min_max(double input, double min, double max)
{
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
}  // namespace tools

