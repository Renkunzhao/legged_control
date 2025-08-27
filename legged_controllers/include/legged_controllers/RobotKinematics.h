#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <Eigen/Dense> // Core Eigen functionalities for matrices and vectors
#include <vector>      // For std::vector (for joint angles, bounds, and solutions)
#include <cmath>       // For M_PI, std::sin, std::cos
#include <string>      // For std::string
#include <stdexcept>   // For std::runtime_error
#include <algorithm>   // For std::max, std::min (used in Trans_equ)

// Define M_PI if your compiler doesn't provide it (GNU extension)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// RobotKinematics class for forward and inverse kinematics.
class RobotKinematics {
public:
    /**
     * @brief Constructor for RobotKinematics.
     * Initializes link lengths and default bounds in-class.
     */
    RobotKinematics();

    /**
     * @brief Calculates the forward kinematics (end-effector pose).
     * @param q A vector containing the joint angles [q1, q2, q3] in radians.
     * @param leg A string indicating the leg type ("LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT").
     * @return A 4x4 homogeneous transformation matrix represented by Eigen::Matrix4d.
     * @throws std::invalid_argument if the input joint angle vector 'q' is not of size 3
     * or if the 'leg' string is unrecognized.
     */
    Eigen::Matrix4d kine(const std::string& leg, const Eigen::Vector3d& q) const;

    /**
     * @brief Calculates the inverse kinematics (joint angles for a given end-effector position).
     * This implementation uses an analytical approach for q3 and then solves
     * transcendental equations for q1 and q2.
     * @param p_target A 3x1 position vector [x, y, z] of the desired end-effector position.
     * @param leg A string indicating the leg type ("LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT").
     * @return A vector of possible joint angle solutions. Each solution is an Eigen::Vector3d
     * ([q1, q2, q3]). If multiple solutions are found, all are returned.
     * @throws std::runtime_error if q3 has no valid solution or no inverse solution is found
     * within the given bounds.
     * @throws std::invalid_argument if the 'leg' string is unrecognized.
     */
    std::vector<Eigen::Vector3d> ikine(const std::string& leg, const Eigen::Vector3d& p_target) const;

    // Optionally, you can add setters if you want to change these values after construction.
    void setJointBounds(const Eigen::Vector3d& lb, const Eigen::Vector3d& ub) {
        lb_ = lb;
        ub_ = ub;
    }

    // Optionally, add getters to access these values
    Eigen::Vector3d getLowerBounds() const { return lb_; }
    Eigen::Vector3d getUpperBounds() const { return ub_; }

private:
    // Define tolerance for floating point comparisons
    static constexpr double TOL = 1e-4;
    static constexpr double EPSILON = 1e-9; // For float comparison in Trans_equ

    // Link lengths (meters) - These remain constant base values
    const double L1_base = 0.1881;
    const double L2_base = 0.04675;
    const double L3_base = 0.08;
    const double L4_base = 0.213;
    const double L5_base = 0.213;

    // Lower and upper bounds for joint angles - Directly initialized in-class
    Eigen::Vector3d lb_ = Eigen::Vector3d(-0.8630, -0.6860, -2.8180);
    Eigen::Vector3d ub_ = Eigen::Vector3d(0.8630, 4.5010, -0.8880);

    /**
     * @brief Solves a*cos(x) + b*sin(x) = c for x within [lb, ub].
     * @param a, b, c Coefficients of the equation.
     * @param lb, ub Lower and upper bounds for x.
     * @return A vector of solutions.
     * @throws std::runtime_error on unsolvable conditions (e.g., |c| > R, or 0=0 case).
     */
    std::vector<double> Trans_equ(double a, double b, double c, double lb, double ub) const;

    // Helper functions for rotation matrices (private as they are internal to kine/ikine)
    Eigen::Matrix3d Rx(double x) const;
    Eigen::Matrix3d Ry(double x) const;
    Eigen::Matrix3d Rz(double x) const;
};

#endif // ROBOT_KINEMATICS_H