#include "kinematics/kinematics.hpp"
#include <cmath>
#include <algorithm>

/**
 * @brief Computes the 4x4 homogeneous transformation matrix from joint angles.
 *
 * Maps the 3-DOF joint angle space directly to a complete SE(3) pose matrix.
 * This combines the internal links using trigonometric projections to yield the
 * exact location and orientation of the tool center point.
 *
 * @param q Vector containing [q1 (Base Yaw), q2 (Shoulder Pitch), q3 (Elbow Pitch)] in radians.
 * @return Eigen::Matrix4d Complete 4x4 SE(3) pose matrix of the end-effector.
 */
Eigen::Matrix4d KinematicsSolver::forwardKinematics(const Eigen::Vector3d &q) const
{
    double base_yaw = q(0);
    double shoulder_pitch = q(1);
    double elbow_pitch = q(2);

    // Base Yaw Trigonometry
    double base_yaw_sin = std::sin(base_yaw);
    double base_yaw_cos = std::cos(base_yaw);

    // Shoulder Pitch Trigonometry
    double shoulder_pitch_sin = std::sin(shoulder_pitch);
    double shoulder_pitch_cos = std::cos(shoulder_pitch);

    // Combined Angle (Shoulder + Elbow) Trigonometry
    // In robotics, the elbow angle acts relative to the shoulder,
    // so their angles add together in world space.
    double combined_pitch = shoulder_pitch + elbow_pitch;
    double combined_pitch_sin = std::sin(combined_pitch);
    double combined_pitch_cos = std::cos(combined_pitch);

    // Calculate the total horzontal radius (X & Y) by adding up horizontal steps
    double upperarm_horizontal_reach = geometry_.upper_arm_length * shoulder_pitch_cos;
    double forearm_horizontal_reach = geometry_.forearm_length * combined_pitch_cos;
    double total_horizontal_radius = upperarm_horizontal_reach + forearm_horizontal_reach;

    // Calculate the total vertical height (Z) by adding up all vertical steps
    double base_lift = geometry_.base_height;
    double upperarm_vertical_lift = geometry_.upper_arm_length * shoulder_pitch_sin;
    double forearm_vertical_lift = geometry_.forearm_length * combined_pitch_sin;

    // Project that radius onto the X and Y axes using the base yaw angle
    double position_x = total_horizontal_radius * base_yaw_cos;
    double position_y = total_horizontal_radius * base_yaw_sin;
    double position_z = base_lift + upperarm_vertical_lift + forearm_vertical_lift;

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // 
    transformation_matrix(0, 3) = position_x;
    transformation_matrix(1, 3) = position_y;
    transformation_matrix(2, 3) = position_z;

    transformation_matrix(0, 0) = base_yaw_cos * combined_pitch_cos;
    transformation_matrix(0, 1) = -base_yaw_cos * combined_pitch_sin;
    transformation_matrix(0, 2) = -base_yaw_sin;

    transformation_matrix(1, 0) = base_yaw_sin * combined_pitch_cos;
    transformation_matrix(1, 1) = -base_yaw_sin * combined_pitch_sin;
    transformation_matrix(1, 2) = base_yaw_cos;

    transformation_matrix(2, 0) = combined_pitch_sin;
    transformation_matrix(2, 1) = combined_pitch_cos;
    transformation_matrix(2, 2) = 0.0;

    return transformation_matrix;
}

/**
 * @brief Computes analytical inverse kinematics for a target 3D Cartesian position.
 *
 * Calculates necessary joint configurations to place the end-effector at a specific
 * coordinate. If the target location violates mechanical reach boundaries, it returns
 * an empty container state instead of a dangerous or invalid zero vector.
 *
 * @param target_xyz Vector containing target [x, y, z] spatial coordinates in meters.
 * @return std::optional<Eigen::Vector3d> Target joint angles [q1, q2, q3] if reachable,
 * or std::nullopt if the point falls out of bounds.
 */
std::optional<Eigen::Vector3d> KinematicsSolver::inverseKinematics(const Eigen::Vector3d &target_xyz) const
{
    // TODO: Where do you get the target cartesian values?
    double target_x = target_xyz(0);
    double target_y = target_xyz(1);
    double target_z = target_xyz(2);


}