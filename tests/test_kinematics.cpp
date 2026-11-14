#undef NDEBUG
#include "kinematics/kinematics.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
constexpr double kTol = 1e-6;

bool isApprox(double a, double b, double tol = kTol) {
    return std::abs(a - b) <= tol;
}
} // namespace

// 1. Constructor invariant check (throws on zero/negative link lengths)
void test_invalid_geometry_throws() {
    bool threw = false;
    try {
        KinematicsSolver bad_solver(ArmGeometry{0.10, 0.0, -0.25});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw && "Constructor must reject non-positive link lengths");
}

// 2. Zero-configuration pose & SE(3) matrix validity
void test_forward_kinematics_zero_pose() {
    ArmGeometry geom{0.10, 0.30, 0.25}; // L1=0.10, L2=0.30, L3=0.25 -> max reach = 0.55
    KinematicsSolver solver(geom);

    Eigen::Matrix4d T = solver.forwardKinematics(Eigen::Vector3d(0.0, 0.0, 0.0));

    // Check translation at q = (0, 0, 0) -> (0.55, 0.0, 0.10)
    assert(isApprox(T(0, 3), 0.55));
    assert(isApprox(T(1, 3), 0.00));
    assert(isApprox(T(2, 3), 0.10));

    // Check bottom homogeneous row
    assert(isApprox(T(3, 0), 0.0));
    assert(isApprox(T(3, 1), 0.0));
    assert(isApprox(T(3, 2), 0.0));
    assert(isApprox(T(3, 3), 1.0));

    // Check valid SO(3) rotation block (R^T * R == I and det(R) == 1)
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    assert((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), kTol));
    assert(isApprox(R.determinant(), 1.0));
}

// 3. Out-of-workspace boundary guard
void test_out_of_workspace_rejection() {
    ArmGeometry geom{0.10, 0.30, 0.25};
    KinematicsSolver solver(geom);

    auto unreachable = solver.inverseKinematics(Eigen::Vector3d(2.0, 0.0, 0.10));
    assert(!unreachable.has_value() && "Unreachable target must return std::nullopt");
}

// 4. IK -> FK round-trip accuracy
void test_round_trip_invariant() {
    ArmGeometry geom{0.10, 0.30, 0.25};
    KinematicsSolver solver(geom);

    Eigen::Vector3d target(0.25, 0.20, 0.30);
    auto q_opt = solver.inverseKinematics(target);
    assert(q_opt.has_value() && "Reachable target should return valid joint angles");

    Eigen::Matrix4d T = solver.forwardKinematics(q_opt.value());
    Eigen::Vector3d fk_pos = T.block<3, 1>(0, 3);

    assert((fk_pos - target).norm() < kTol && "FK(IK(target)) must match target within 1e-6m");
}

int main() {
    test_invalid_geometry_throws();
    test_forward_kinematics_zero_pose();
    test_out_of_workspace_rejection();
    test_round_trip_invariant();

    std::cout << "[PASS] All KinematicsSolver SE(3) & IK tests passed!\n";
    return 0;
}
