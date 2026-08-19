// SPDX-License-Identifier: MIT
#include "odc/pose_ekf.hpp"

#include <cmath>

namespace odc {

double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

namespace {
constexpr std::size_t kN = 4;
using Mat = std::array<std::array<double, kN>, kN>;

/// C = A * B
Mat matmul(const Mat& a, const Mat& b) {
  Mat c{};
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t k = 0; k < kN; ++k) {
      const double aik = a[i][k];
      if (aik == 0.0) continue;
      for (std::size_t j = 0; j < kN; ++j) c[i][j] += aik * b[k][j];
    }
  return c;
}

/// C = A * B^T
Mat matmulT(const Mat& a, const Mat& b) {
  Mat c{};
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = 0; j < kN; ++j) {
      double s = 0.0;
      for (std::size_t k = 0; k < kN; ++k) s += a[i][k] * b[j][k];
      c[i][j] = s;
    }
  return c;
}

/// In-place 3x3 inverse. Returns false if the matrix is effectively singular.
bool invert3(const double m[3][3], double out[3][3]) {
  const double a = m[0][0], b = m[0][1], c = m[0][2];
  const double d = m[1][0], e = m[1][1], f = m[1][2];
  const double g = m[2][0], h = m[2][1], i = m[2][2];
  const double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const double det = a * A + b * B + c * C;
  if (std::abs(det) < 1e-18) return false;
  const double inv = 1.0 / det;
  out[0][0] = A * inv;
  out[0][1] = -(b * i - c * h) * inv;
  out[0][2] = (b * f - c * e) * inv;
  out[1][0] = B * inv;
  out[1][1] = (a * i - c * g) * inv;
  out[1][2] = -(a * f - c * d) * inv;
  out[2][0] = C * inv;
  out[2][1] = -(a * h - b * g) * inv;
  out[2][2] = (a * e - b * d) * inv;
  return true;
}
}  // namespace

void PoseEkf::reset(const Pose2D& initial) {
  x_ = {initial.x, initial.y, initial.theta, 0.0};
  last_nis_ = 0.0;
  nis_fresh_ = false;
  P_ = Mat{};
  P_[0][0] = P_[1][1] = 0.25;  // 0.5 m
  P_[2][2] = 0.09;             // ~17 deg
  P_[3][3] = 1e-4;             // gyro bias
  time_since_fix_ = 0.0;
}

void PoseEkf::predict(const Twist2D& t, double gyro_z, double dt,
                      bool slip_detected) {
  if (dt <= 0.0) return;

  const double th = x_[2];
  const double c = std::cos(th), s = std::sin(th);
  const double wz = gyro_z - x_[3];

  // --- State propagation (exact Euler on the unicycle-free holonomic model).
  x_[0] += (t.vx * c - t.vy * s) * dt;
  x_[1] += (t.vx * s + t.vy * c) * dt;
  x_[2] = wrapAngle(th + wz * dt);
  // gyro bias is a random walk: mean unchanged.

  // --- Jacobian F = d f / d x.
  Mat F{};
  for (std::size_t i = 0; i < kN; ++i) F[i][i] = 1.0;
  F[0][2] = (-t.vx * s - t.vy * c) * dt;
  F[1][2] = (t.vx * c - t.vy * s) * dt;
  F[2][3] = -dt;

  // --- Process noise. Translation noise is isotropic in the body frame then
  //     rotated into the world frame, which matters a lot for a holonomic base
  //     that can be strafing sideways at full speed.
  const double infl = slip_detected ? p_.slip_inflation : 1.0;
  const double qv = p_.sigma_v * p_.sigma_v * dt * dt * infl;
  Mat Q{};
  Q[0][0] = qv * (c * c + s * s);
  Q[1][1] = qv * (s * s + c * c);
  Q[2][2] = p_.sigma_gyro * p_.sigma_gyro * dt * dt * infl;
  Q[3][3] = p_.sigma_bias_walk * p_.sigma_bias_walk * dt;

  const Mat FP = matmul(F, P_);
  P_ = matmulT(FP, F);
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = 0; j < kN; ++j) P_[i][j] += Q[i][j];

  time_since_fix_ += dt;
}

bool PoseEkf::update(const PoseFix& fix) {
  if (!fix.valid) return false;

  // H = [I3 | 0], so most of the algebra collapses to the top-left 3x3 block.
  double innov[3] = {fix.pose.x - x_[0], fix.pose.y - x_[1],
                     wrapAngle(fix.pose.theta - x_[2])};

  double S[3][3];
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) S[i][j] = P_[i][j];
  const double rxy = fix.std_xy * fix.std_xy;
  S[0][0] += rxy;
  S[1][1] += rxy;
  S[2][2] += fix.std_theta * fix.std_theta;

  double Sinv[3][3];
  if (!invert3(S, Sinv)) return false;

  // --- Mahalanobis gate. One bad scan match should not teleport the robot.
  double d2 = 0.0;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) d2 += innov[i] * Sinv[i][j] * innov[j];
  // Record the innovation statistic before gating: a fix so bad it gets
  // thrown away is the strongest possible evidence that odometry is lying.
  last_nis_ = d2;
  nis_fresh_ = true;
  if (d2 > p_.chi2_gate) return false;

  // --- Gain K = P H^T S^-1  (4x3).
  double K[kN][3]{};
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = 0; j < 3; ++j) {
      double s = 0.0;
      for (std::size_t k = 0; k < 3; ++k) s += P_[i][k] * Sinv[k][j];
      K[i][j] = s;
    }

  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = 0; j < 3; ++j) x_[i] += K[i][j] * innov[j];
  x_[2] = wrapAngle(x_[2]);

  // --- Joseph-free but symmetric-safe covariance update: P = (I - K H) P.
  Mat KH{};
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = 0; j < 3; ++j) KH[i][j] = K[i][j];

  Mat I_KH{};
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = 0; j < kN; ++j)
      I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];

  P_ = matmul(I_KH, P_);
  for (std::size_t i = 0; i < kN; ++i)
    for (std::size_t j = i + 1; j < kN; ++j) {
      const double sym = 0.5 * (P_[i][j] + P_[j][i]);
      P_[i][j] = P_[j][i] = sym;
    }

  time_since_fix_ = 0.0;
  return true;
}

double PoseEkf::positionSigma() const {
  return std::sqrt(std::max(0.0, P_[0][0] + P_[1][1]));
}

}  // namespace odc
