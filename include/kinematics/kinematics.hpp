#pragma once

#include <Eigen/Dense>
#include <optional>
#include <stdexcept>

struct ArmGeometry
{
    double base_height = 0.10;      // Base height in meters
    double upper_arm_length = 0.30; // Upper arm length in meters
    double forearm_length = 0.25;   // Forearm length in meters
};

class KinematicsSolver
{
public:
    // Explicit constructor ensuring intentional instantiation with geometric parameters
    explicit KinematicsSolver(const ArmGeometry &geometry)
        : geometry_(geometry)
    {
        if (geometry_.base_height < 0.0 || geometry_.upper_arm_length <= 0.0 || geometry_.forearm_length <= 0.0)
        {
            throw std::invalid_argument("ArmGeometry link lengths must be strictly positive.");
        }
    }

    Eigen::Matrix4d forwardKinematics(const Eigen::Vector3d &q) const;

    std::optional<Eigen::Vector3d> inverseKinematics(const Eigen::Vector3d &target_xyz) const;

private:
    ArmGeometry geometry_;
};
