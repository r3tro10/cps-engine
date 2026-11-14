#include "kinematics/kinematics.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

/**
 * @brief Computes the 4x4 homogeneous transformation matrix from joint angles.
 *
 * Maps the 3-DOF joint angle space directly to a complete SE(3) pose matrix.
 * Uses Z-yaw followed by upward pitch in the vertical plane so det(R) == +1.
 *
 * @param q Vector containing [q1 (Base Yaw), q2 (Shoulder Pitch), q3 (Elbow Pitch)] in radians.
 * @return Eigen::Matrix4d Complete 4x4 SE(3) pose matrix of the end-effector.
 */
Eigen::Matrix4d KinematicsSolver::forwardKinematics(const Eigen::Vector3d &q) const
{
    const double base_yaw = q(0);
    const double shoulder_pitch = q(1);
    const double elbow_pitch = q(2);

    const double base_yaw_sin = std::sin(base_yaw);
    const double base_yaw_cos = std::cos(base_yaw);

    const double shoulder_pitch_sin = std::sin(shoulder_pitch);
    const double shoulder_pitch_cos = std::cos(shoulder_pitch);

    // Cumulative pitch angle in the vertical arm plane: theta_23 = q2 + q3
    const double combined_pitch = shoulder_pitch + elbow_pitch;
    const double combined_pitch_sin = std::sin(combined_pitch);
    const double combined_pitch_cos = std::cos(combined_pitch);

    // Radial reach in the horizontal XY-plane
    const double upperarm_horizontal_reach = geometry_.upper_arm_length * shoulder_pitch_cos;
    const double forearm_horizontal_reach = geometry_.forearm_length * combined_pitch_cos;
    const double total_horizontal_radius = upperarm_horizontal_reach + forearm_horizontal_reach;

    // 3D Cartesian position p = [x, y, z]^T
    const double position_x = total_horizontal_radius * base_yaw_cos;
    const double position_y = total_horizontal_radius * base_yaw_sin;
    const double position_z = geometry_.base_height + geometry_.upper_arm_length * shoulder_pitch_sin + geometry_.forearm_length * combined_pitch_sin;

    // Proper SO(3) rotation block (det(R) = +1): Rz(q1) * Ry(-combined_pitch)
    // Yaw around +Z, then pitch upward (-combined_pitch around +Y)

        // 1. Let Eigen compute the 3x3 rotation from your two angles
    Eigen::Matrix3d rotation = (Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-combined_pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();

    // 2. Create the 4x4 matrix (starts with 1s on the diagonal, 0s elsewhere)
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // 3. Copy the 3x3 rotation into top-left, and 3x1 position into top-right
    transformation_matrix.block<3, 3>(0, 0) = rotation;
    transformation_matrix.block<3, 1>(0, 3) = Eigen::Vector3d(position_x, position_y, position_z);

    return transformation_matrix;


}

/**
 * @brief Computes analytical inverse kinematics for a target 3D Cartesian position.
 *
 * @param target_xyz Vector containing target [x, y, z] spatial coordinates in meters.
 * @return std::optional<Eigen::Vector3d> Target joint angles [q1, q2, q3] if reachable,
 * or std::nullopt if the point falls out of bounds.
 */
std::optional<Eigen::Vector3d> KinematicsSolver::inverseKinematics(const Eigen::Vector3d &target_xyz) const
{
    const double target_x = target_xyz(0);
    const double target_y = target_xyz(1);
    const double target_z = target_xyz(2);

    const double upper = geometry_.upper_arm_length;
    const double forearm = geometry_.forearm_length;

    // 1. Cylindrical projection
    const double horizontal_radius = std::hypot(target_x, target_y);
    const double vertical_offset = target_z - geometry_.base_height;
    const double reach_distance = std::hypot(horizontal_radius, vertical_offset);

    // 2. Workspace boundary guard
    const double max_reach = upper + forearm;
    const double min_reach = std::abs(upper - forearm);

    if (reach_distance > max_reach + 1e-9 || reach_distance < min_reach - 1e-9)
    {
        return std::nullopt;
    }

    // 3. Solve joint angles (elbow-up configuration)
    const double base_yaw = std::atan2(target_y, target_x);

    const double cos_elbow = (reach_distance * reach_distance - upper * upper - forearm * forearm) / (2.0 * upper * forearm);
    const double clamped_cos = std::clamp(cos_elbow, -1.0, 1.0);
    const double elbow_pitch = -std::acos(clamped_cos);

    const double shoulder_pitch = std::atan2(vertical_offset, horizontal_radius) - std::atan2(forearm * std::sin(elbow_pitch), upper + forearm * std::cos(elbow_pitch));

    return Eigen::Vector3d(base_yaw, shoulder_pitch, elbow_pitch);
}
