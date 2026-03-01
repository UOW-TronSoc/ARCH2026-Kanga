#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

namespace kanga_arm_controller
{

/**
 * @brief Lightweight, non-ROS arm kinematics helper for the 5-DOF arm stack.
 *
 * Uses a DH-style transformation-matrix chain for the first 4 joints, with a
 * mode-selectable final fixed tool transform (roll tool vs scoop). Joint 5
 * (tool roll) is intentionally excluded from the endpoint Jacobian.
 */
class ArmKinematics
{
public:
  enum class EndEffectorMode
  {
    kRollTool,
    kScoop
  };

  struct ToolTransform
  {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
  };

  explicit ArmKinematics(size_t dof = 5)
  : dof_(dof)
  {
    // Defaults from provided symbolic/DH model (SI units).
    d_ = {0.084, 0.111, -0.0905, 0.06844};
    a_ = {0.0, -0.449997, -0.390, 0.0};
    alpha_ = {-M_PI_2, 0.0, 0.0, M_PI_2};
    theta_offsets_ = {0.0, M_PI_2, M_PI_2, 0.0};

    // Final tool transform defaults from provided model.
    roll_tool_tf_ = dhTransform(0.0, 0.241725, 0.0, 0.0);
    scoop_tool_tf_ = dhTransform(0.0, 0.230, 0.0, 0.0);
  }

  void setLinkLengths(const std::vector<double> & link_lengths)
  {
    // Keep config compatibility by mapping the legacy link-length vector into
    // the active DH model where meaningful.
    if (link_lengths.size() > 0) {
      d_[0] = link_lengths[0];
    }
    if (link_lengths.size() > 1) {
      a_[1] = -std::abs(link_lengths[1]);
    }
    if (link_lengths.size() > 2) {
      a_[2] = -std::abs(link_lengths[2]);
    }
    if (link_lengths.size() > 3) {
      d_[3] = link_lengths[3];
    }
    if (link_lengths.size() > 4) {
      // Legacy configs often specified terminal reach as X offset. Keep this as
      // a compatibility fallback unless explicit tool transforms are configured.
      if (!roll_tool_override_) {
        roll_tool_tf_ = Eigen::Matrix4d::Identity();
        roll_tool_tf_(0, 3) = link_lengths[4];
      }
      if (!scoop_tool_override_) {
        scoop_tool_tf_ = Eigen::Matrix4d::Identity();
        scoop_tool_tf_(0, 3) = link_lengths[4];
      }
    }
  }

  void setToolTransforms(const ToolTransform & roll_tool, const ToolTransform & scoop_tool)
  {
    roll_tool_tf_ = toMatrix(roll_tool.translation, roll_tool.rpy);
    scoop_tool_tf_ = toMatrix(scoop_tool.translation, scoop_tool.rpy);
    roll_tool_override_ = true;
    scoop_tool_override_ = true;
  }

  void setToolTransforms(const Eigen::Matrix4d & roll_tool_tf, const Eigen::Matrix4d & scoop_tool_tf)
  {
    roll_tool_tf_ = roll_tool_tf;
    scoop_tool_tf_ = scoop_tool_tf;
    roll_tool_override_ = true;
    scoop_tool_override_ = true;
  }

  bool setEndEffectorMode(const std::string & mode)
  {
    const std::string mode_lower = toLower(mode);
    if (mode_lower == "roll_tool" || mode_lower == "roll") {
      mode_ = EndEffectorMode::kRollTool;
      return true;
    }
    if (mode_lower == "scoop") {
      mode_ = EndEffectorMode::kScoop;
      return true;
    }
    return false;
  }

  EndEffectorMode mode() const
  {
    return mode_;
  }

  std::string modeString() const
  {
    return mode_ == EndEffectorMode::kRollTool ? "roll_tool" : "scoop";
  }

  /**
   * @brief Full end-effector transform in the world frame for joint state q.
   */
  Eigen::Isometry3d forwardTransform(const Eigen::VectorXd & q) const
  {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();

    const size_t active_chain_dof = std::min<size_t>(4, dof_);
    for (size_t i = 0; i < active_chain_dof; ++i) {
      const double theta = theta_offsets_[i] + qAt(q, i);
      t = t * dhTransform(theta, d_[i], a_[i], alpha_[i]);
    }

    t = t * activeToolTransform();

    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.matrix() = t;
    return out;
  }

  Eigen::Vector3d forwardPosition(const Eigen::VectorXd & q) const
  {
    return forwardTransform(q).translation();
  }

  /**
   * @brief Computes a 6xDOF geometric Jacobian from the transform chain.
   *
   * Jv_i = z_i x (p_ee - p_i), Jw_i = z_i for revolute joints.
   * Only joints 1..4 are active in this task-space Jacobian.
   */
  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd & q) const
  {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(dof_));
    const size_t active_chain_dof = std::min<size_t>(4, dof_);
    if (active_chain_dof == 0) {
      return jacobian;
    }

    std::vector<Eigen::Matrix4d> prefix(active_chain_dof + 1, Eigen::Matrix4d::Identity());
    for (size_t i = 0; i < active_chain_dof; ++i) {
      const double theta = theta_offsets_[i] + qAt(q, i);
      prefix[i + 1] = prefix[i] * dhTransform(theta, d_[i], a_[i], alpha_[i]);
    }

    const Eigen::Matrix4d t_ee = prefix[active_chain_dof] * activeToolTransform();
    const Eigen::Vector3d p_ee = t_ee.block<3, 1>(0, 3);

    for (size_t i = 0; i < active_chain_dof; ++i) {
      const Eigen::Vector3d z_i = prefix[i].block<3, 1>(0, 2);
      const Eigen::Vector3d p_i = prefix[i].block<3, 1>(0, 3);
      jacobian.block<3, 1>(0, static_cast<Eigen::Index>(i)) = z_i.cross(p_ee - p_i);
      jacobian.block<3, 1>(3, static_cast<Eigen::Index>(i)) = z_i;
    }

    return jacobian;
  }

private:
  static Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha)
  {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t(0, 0) = ct;
    t(0, 1) = -st * ca;
    t(0, 2) = st * sa;  
    t(0, 3) = a * ct;
    t(1, 0) = st;
    t(1, 1) = ct * ca;
    t(1, 2) = -ct * sa;
    t(1, 3) = a * st;
    t(2, 0) = 0.0;
    t(2, 1) = sa;
    t(2, 2) = ca;
    t(2, 3) = d;
    return t;
  }

  static Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d & rpy)
  {
    const Eigen::AngleAxisd rx(rpy.x(), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(rpy.y(), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(rpy.z(), Eigen::Vector3d::UnitZ());
    return (rz * ry * rx).toRotationMatrix();
  }

  static Eigen::Matrix4d toMatrix(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy)
  {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = rpyToRotation(rpy);
    t.block<3, 1>(0, 3) = xyz;
    return t;
  }

  static std::string toLower(std::string s)
  {
    std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
  }

  double qAt(const Eigen::VectorXd & q, size_t index) const
  {
    if (index >= static_cast<size_t>(q.size())) {
      return 0.0;
    }
    return q(static_cast<Eigen::Index>(index));
  }

  const Eigen::Matrix4d & activeToolTransform() const
  {
    return mode_ == EndEffectorMode::kRollTool ? roll_tool_tf_ : scoop_tool_tf_;
  }

  size_t dof_{5};
  std::array<double, 4> d_{};
  std::array<double, 4> a_{};
  std::array<double, 4> alpha_{};
  std::array<double, 4> theta_offsets_{};
  EndEffectorMode mode_{EndEffectorMode::kRollTool};
  Eigen::Matrix4d roll_tool_tf_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d scoop_tool_tf_{Eigen::Matrix4d::Identity()};
  bool roll_tool_override_{false};
  bool scoop_tool_override_{false};
};

}  // namespace kanga_arm_controller
