#include "legged_controllers/RobotKinematics.h"
#include <iostream>  // For std::cout, std::cerr
#include <algorithm> // For std::sort, std::unique

// --- Private Helper Functions for RobotKinematics (using Eigen) ---

// Rx, Ry, Rz (unchanged)
Eigen::Matrix3d RobotKinematics::Rx(double x) const {
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,
         0.0, std::cos(x), -std::sin(x),
         0.0, std::sin(x), std::cos(x);
    return R;
}

Eigen::Matrix3d RobotKinematics::Ry(double x) const {
    Eigen::Matrix3d R;
    R << std::cos(x), 0.0, std::sin(x),
         0.0, 1.0, 0.0,
         -std::sin(x), 0.0, std::cos(x);
    return R;
}

Eigen::Matrix3d RobotKinematics::Rz(double x) const {
    Eigen::Matrix3d R;
    R << std::cos(x), -std::sin(x), 0.0,
         std::sin(x), std::cos(x), 0.0,
         0.0, 0.0, 1.0;
    return R;
}

// Trans_equ (unchanged)
std::vector<double> RobotKinematics::Trans_equ(double a, double b, double c, double lb, double ub) const {
    std::vector<double> solutions;

    double R = std::sqrt(a * a + b * b);

    if (R < EPSILON) {
        if (std::abs(c) < EPSILON) {
            throw std::runtime_error("Error: Infinite solutions (0*cos(x) + 0*sin(x) = 0). Cannot enumerate all solutions in the interval.");
        } else {
            throw std::runtime_error("Error: No solution exists for 0*cos(x) + 0*sin(x) = " + std::to_string(c) + " (c != 0).");
        }
    }

    if (std::abs(c) > R + EPSILON) {
        throw std::runtime_error("Error: No solution exists: |c| > sqrt(a^2 + b^2)");
    }

    double phi = std::atan2(b, a);
    double theta = std::acos(c / R);

    double base_sol1 = theta + phi;
    double base_sol2 = -theta + phi;

    int n_min1 = static_cast<int>(std::ceil((lb - base_sol1) / (2 * M_PI) - EPSILON));
    int n_max1 = static_cast<int>(std::floor((ub - base_sol1) / (2 * M_PI) + EPSILON));

    int n_min2 = static_cast<int>(std::ceil((lb - base_sol2) / (2 * M_PI) - EPSILON));
    int n_max2 = static_cast<int>(std::floor((ub - base_sol2) / (2 * M_PI) + EPSILON));

    for (int n = n_min1; n <= n_max1; ++n) {
        double current_sol = base_sol1 + 2 * M_PI * n;
        if (current_sol >= lb - EPSILON && current_sol <= ub + EPSILON) {
            solutions.push_back(current_sol);
        }
    }

    for (int n = n_min2; n <= n_max2; ++n) {
        double current_sol = base_sol2 + 2 * M_PI * n;
        if (current_sol >= lb - EPSILON && current_sol <= ub + EPSILON) {
            solutions.push_back(current_sol);
        }
    }

    std::sort(solutions.begin(), solutions.end());

    solutions.erase(std::unique(solutions.begin(), solutions.end(),
                                [](double val1, double val2) {
                                    return std::abs(val1 - val2) < EPSILON;
                                }),
                    solutions.end());

    if (solutions.empty()) {
        std::cout << "Warning: No solution found within the specified bounds." << std::endl;
    }

    return solutions;
}

// --- RobotKinematics Class Methods ---

RobotKinematics::RobotKinematics() {
    // Member variables are initialized in-class directly.
    // This constructor can be empty or used for additional setup.
}

Eigen::Matrix4d RobotKinematics::kine(const std::string& leg, const Eigen::Vector3d& q) const {
    if (q.size() != 3) {
        throw std::invalid_argument("Input 'q' must be an Eigen::Vector3d.");
    }

    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];

    // Local copies of link lengths to be modified based on leg type
    double current_L1 = L1_base;
    double current_L2 = L2_base;
    double current_L3 = L3_base;
    double current_L4 = L4_base; // L4 and L5 are not modified by leg type in your MATLAB example
    double current_L5 = L5_base;

    // Apply leg-specific transformations to link lengths
    if (leg == "LF_FOOT") {
        // Default, no changes needed for LF
    } else if (leg == "LH_FOOT") {
        current_L1 = -L1_base;
    } else if (leg == "RF_FOOT") {
        current_L2 = -L2_base;
        current_L3 = -L3_base;
    } else if (leg == "RH_FOOT") {
        current_L1 = -L1_base;
        current_L2 = -L2_base;
        current_L3 = -L3_base;
    } else {
        throw std::invalid_argument("Unrecognized leg type: " + leg + ". Expected 'LF', 'LH', 'RF', or 'RH'.");
    }

    Eigen::Vector3d pb_1(current_L1, current_L2, 0.0);
    Eigen::Vector3d p1_2(0.0, current_L3, 0.0);
    Eigen::Vector3d p2_3(0.0, 0.0, -current_L4); // Using current_L4
    Eigen::Vector3d p3_e(0.0, 0.0, -current_L5); // Using current_L5

    auto homo_trans_eigen = [](const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = p;
        return T;
    };

    Eigen::Matrix4d Tb_1 = homo_trans_eigen(Rx(q1), pb_1);
    Eigen::Matrix4d T1_2 = homo_trans_eigen(Ry(q2), p1_2);
    Eigen::Matrix4d T2_3 = homo_trans_eigen(Ry(q3), p2_3);
    Eigen::Matrix4d T3_e = homo_trans_eigen(Eigen::Matrix3d::Identity(), p3_e);

    Eigen::Matrix4d T = Tb_1 * T1_2 * T2_3 * T3_e;

    return T;
}

std::vector<Eigen::Vector3d> RobotKinematics::ikine(const std::string& leg, const Eigen::Vector3d& p_target) const {
    if (p_target.size() != 3) {
        throw std::invalid_argument("Input vector 'p_target' must be an Eigen::Vector3d.");
    }

    // Local copies of link lengths to be modified based on leg type
    double current_L1 = L1_base;
    double current_L2 = L2_base;
    double current_L3 = L3_base;
    double current_L4 = L4_base;
    double current_L5 = L5_base;

    // Apply leg-specific transformations to link lengths
    if (leg == "LF_FOOT") {
        // Default, no changes needed for LF
    } else if (leg == "LH_FOOT") {
        current_L1 = -L1_base;
    } else if (leg == "RF_FOOT") {
        current_L2 = -L2_base;
        current_L3 = -L3_base;
    } else if (leg == "RH_FOOT") {
        current_L1 = -L1_base;
        current_L2 = -L2_base;
        current_L3 = -L3_base;
    } else {
        throw std::invalid_argument("Unrecognized leg type: " + leg + ". Expected 'LF', 'LH', 'RF', or 'RH'.");
    }

    double x = p_target[0];
    double y = p_target[1];
    double z = p_target[2];

    // Analytical solution for q3 using current_L values
    double num = std::pow(x - current_L1, 2) + std::pow(y - current_L2, 2) + std::pow(z, 2) - std::pow(current_L3, 2) - std::pow(current_L4, 2) - std::pow(current_L5, 2);
    double den = 2 * current_L4 * current_L5;

    double cos_q3 = num / den;

    if (std::abs(cos_q3) > 1.0 + EPSILON) {
        throw std::runtime_error("q3 has no valid solution: |cos_q3| > 1.");
    }

    cos_q3 = std::max(-1.0, std::min(1.0, cos_q3));

    // Only take the negative solution for q3 as per MATLAB code
    double q3 = -std::acos(cos_q3);

    // Solve for q1 and q2 using the Trans_equ solver
    // For q1: (y-L2) * cos(q1) + z * sin(q1) = L3
    std::vector<double> q1_candidates = Trans_equ(y - current_L2, z, current_L3, lb_[0], ub_[0]);

    // For q2: (L5*sin(q3)) * cos(q2) + (L4+L5*cos(q3)) * sin(q2) = L1 - x
    std::vector<double> q2_candidates = Trans_equ(current_L5 * std::sin(q3), current_L4 + current_L5 * std::cos(q3), current_L1 - x, lb_[1], ub_[1]);

    std::vector<Eigen::Vector3d> solutions;

    // Iterate through all combinations of q1 and q2 candidates
    for (double current_q1 : q1_candidates) {
        for (double current_q2 : q2_candidates) {
            Eigen::Vector3d q_test(current_q1, current_q2, q3);
            // Check if q3 is within its allowed range (lb_[2], ub_[2])
            if (q_test[2] >= lb_[2] - EPSILON && q_test[2] <= ub_[2] + EPSILON) {
                // IMPORTANT: Pass the 'leg' argument to kine here!
                Eigen::Matrix4d p_test_matrix = kine(leg, q_test); // <--- Changed here
                Eigen::Vector3d p_test_end_effector = p_test_matrix.block<3, 1>(0, 3);

                double dist = (p_test_end_effector - p_target).norm();

                if (dist < TOL) {
                    solutions.push_back(q_test);
                }
            }
        }
    }

    std::sort(solutions.begin(), solutions.end(), [](const Eigen::Vector3d& q_a, const Eigen::Vector3d& q_b) {
        if (std::abs(q_a[0] - q_b[0]) > TOL) return q_a[0] < q_b[0];
        if (std::abs(q_a[1] - q_b[1]) > TOL) return q_a[1] < q_b[1];
        return q_a[2] < q_b[2];
    });

    solutions.erase(std::unique(solutions.begin(), solutions.end(),
                                [](const Eigen::Vector3d& q_a, const Eigen::Vector3d& q_b) {
                                    return (q_a - q_b).norm() < TOL;
                                }),
                    solutions.end());

    if (solutions.empty()) {
        throw std::runtime_error("No valid inverse kinematics solution found within bounds for the given position.");
    } else if (solutions.size() > 1) {
        std::cerr << "Warning: Multiple inverse kinematics solutions found. Returning all." << std::endl;
    }

    return solutions;
}