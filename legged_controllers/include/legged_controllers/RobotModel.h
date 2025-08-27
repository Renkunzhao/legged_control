#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>  // 需要包含这个头文件
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/math/rpy.hpp>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <string>
#include <vector>

#include "legged_controllers/RobotKinematics.h"

/**
    * @brief RobotModel类，封装了Pinocchio模型的基本操作
    * @details 该类支持两种基座方向表示方式：四元数和欧拉角ZYX, 分别对应base_orientation_type_="quaternion" or "eulerZYX"。
    * @note Pinocchio中关节顺序是固定的[LF, LH, RF, RH], 与OCS2一致，这主要是因为pinocchio默认按照字典顺序排列关节。
    * @note rbdstate = [eulerZyx, base_position, joint_positions, base_angularVel(world), base_linearVel(world) joint_velocities]
    * @note base_orientation_type_ = "quaternion"时
                q_pinocchio = [base_position, base_orientation_quat, joint_positions]
                v_pinocchio = [base_linearVel(base), base_angularVel(base), joint_velocities]
    * @note base_orientation_type_ = "eulerZYX"时
                q_pinocchio = [base_position, base_orientation_eulerZYX, joint_positions
                v_pinocchio = [base_linearVel(world), eulerZyx_dot, joint_velocities]
 */

// RobotModel类，封装了Pinocchio模型的基本操作
// 需要注意，Pinocchio中关节顺序是固定的[LF, LH, RF, RH]，而OCS2中默认顺序是[LF, RF, LH, RH]。
class RobotModel {
public:
    explicit RobotModel(const std::string& urdf_path, const std::string& base_orientation_type = "quaternion") :
        base_orientation_type_(base_orientation_type) {
        if (base_orientation_type == "quaternion") {
            // 使用 pinocchio::JointModelFreeFlyer 的浮动基机器人模型（基于四元数）
            pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
        } else if (base_orientation_type == "eulerZYX") {
            // 使用 pinocchio::JointModelComposite(Translation + EulerZYX) 的浮动基机器人模型   
            pinocchio::JointModelComposite jointComposite(2);
            jointComposite.addJoint(pinocchio::JointModelTranslation());      // 3 DoF 平移
            jointComposite.addJoint(pinocchio::JointModelSphericalZYX());     // 3 DoF 旋转
            pinocchio::urdf::buildModel(urdf_path, jointComposite, model_);
        } else {
            throw std::runtime_error("Invalid orientation type specified: " + base_orientation_type);
        }

        init();  // 初始化 q 和 v    
        setLimits();  // 设置关节位置和速度的上下限
        
        data_ = pinocchio::Data(model_);    
    }

    int nq() const { return model_.nq; }
    int nv() const { return model_.nv; }
    int nqBase() const { return nqBase_; }

    const pinocchio::Model& getModel() const { return model_; }
    pinocchio::Data& getData() { return data_; }

    std::vector<Eigen::Vector3d> getInitfeetPositions() const {
        return initFeetPositions_;
    }

    /**
    * @brief 将关节配置向量 `q` 中基座的方向部分设置为零旋转。
    *
    * 根据 `base_orientation_type_` (四元数或欧拉角)，将 `q` 中基座的方向分量
    * 设置为对应表示的零旋转（例如四元数 `[0,0,0,1]` 或欧拉角 `[0,0,0]`）。
    * 基座的位置 (x, y, z) 不受影响。
    *
    * @param q 待修改的关节配置向量，其维度必须 >= `nqBase_`。
    * @throws std::runtime_error 如果 `q` 的维度不足。
    */
    void setBaseOrientationZero(Eigen::VectorXd& q) const {
        if (q.size() < nqBase_) {
            throw std::runtime_error("Input vector q must have at least " + std::to_string(nqBase_) + " dimensions.");
        }

        if (base_orientation_type_ == "quaternion") {
            q.segment<4>(3) << 0, 0, 0, 1;
        } else if (base_orientation_type_ == "eulerZYX") {
            q.segment<3>(3).setZero();
        }
    }

    /**
    * @brief 根据 base_orientation_type_ 从 q 中获取基座方向。
    *
    * 此函数从 Pinocchio 的广义关节位置向量 q 中提取基座的姿态信息，
    * 并根据 `base_orientation_type_` (四元数或欧拉角ZYX) 将其转换为旋转矩阵。
    *
    * @param q Pinocchio的广义关节位置向量，其头部包含基座位置和方向。
    * @return 一个 `Eigen::Matrix3d` 旋转矩阵，表示基座相对于世界坐标系的姿态。
    * @throws std::runtime_error 如果 `base_orientation_type_` 无效或 `q` 向量的维度不正确。
    */
    Eigen::Matrix3d getBaseOrientation(const Eigen::VectorXd& q) const {
        if (q.size() < nqBase_) {
            throw std::runtime_error("Input vector q must have at least " + std::to_string(nqBase_) + " dimensions.");
        }

        if (base_orientation_type_ == "quaternion") {
            // 从 q 中提取四元数并转换为旋转矩阵
            Eigen::Quaterniond quat(q.segment<4>(3));
            return quat.normalized().toRotationMatrix();
        } else if (base_orientation_type_ == "eulerZYX") {
            // 从 q 中提取欧拉角并转换为旋转矩阵
            Eigen::Vector3d rpy = q.segment<3>(3);
            return pinocchio::rpy::rpyToMatrix(rpy);
        } else {
            throw std::runtime_error("Invalid base orientation type: " + base_orientation_type_);
        }
    }

    int getLegIndex(const std::string& footName) const {
        auto it = std::find(feetNames_.begin(), feetNames_.end(), footName);
        if (it != feetNames_.end()) {
            return std::distance(feetNames_.begin(), it);
        } else {
            throw std::runtime_error("Foot name not found: " + footName);
        }
    }

    /**
     * @brief 将 RbdState 格式的数据转换为 Pinocchio 的 q 和 v 向量。
     * * 此函数利用类成员变量 base_orientation_type_ 来决定正确的转换逻辑。
     * RbdState 向量的格式假定为：
     * @note rbdstate = [eulerZyx, base_position, joint_positions, base_angularVel(world), base_linearVel(world) joint_velocities]
     *
     * @param[in] rbdStateMeasured 测量到的 RbdState 向量。
     * @param[out] q_pinocchio 转换后的 Pinocchio q 向量。
     * @param[out] v_pinocchio 转换后的 Pinocchio v 向量。
     * @throws std::runtime_error 如果 `rbdStateMeasured` 向量的维度不正确或 `base_orientation_type_` 无效。
     */
    void convertRbdStateToPinocchio(const Eigen::VectorXd& rbdStateMeasured,
                                    Eigen::VectorXd& q_pinocchio,
                                    Eigen::VectorXd& v_pinocchio) const {
        using Scalar = double;
        // The RbdState vector should have a size of 2 * nqGeneralized_.
        if (rbdStateMeasured.size() != 2 * nqGeneralized_) {
            throw std::runtime_error("Invalid size of rbdStateMeasured vector.");
        }
        
        // 1. 获取 RbdState 的位置数据，并转换为 Pinocchio q
        // RbdState q part: [base_euler_zyx(3), base_pos(3), joint_pos(N)]
        const Eigen::Vector3d baseEulerZyx_rbd = rbdStateMeasured.head<3>();
        const Eigen::Vector3d basePosition_rbd = rbdStateMeasured.segment<3>(3);
        const Eigen::VectorXd jointPositions_rbd = rbdStateMeasured.segment(6, njoints_);
        
        q_pinocchio.resize(model_.nq);
        q_pinocchio.head<3>() = basePosition_rbd;
        
        if (base_orientation_type_ == "quaternion") {
            // Pinocchio q: [base_pos(3), base_quat(4), joint_pos(N)]
            Eigen::AngleAxis<Scalar> roll(baseEulerZyx_rbd(2), Eigen::Vector3d::UnitX());
            Eigen::AngleAxis<Scalar> pitch(baseEulerZyx_rbd(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxis<Scalar> yaw(baseEulerZyx_rbd(0), Eigen::Vector3d::UnitZ());
            Eigen::Quaternion<Scalar> q_quat = yaw * pitch * roll;

            q_pinocchio.segment<4>(3) << q_quat.x(), q_quat.y(), q_quat.z(), q_quat.w();
        } else if (base_orientation_type_ == "eulerZYX") {
            // Pinocchio q: [base_pos(3), base_euler(3), joint_pos(N)]
            q_pinocchio.segment<3>(3) = baseEulerZyx_rbd;
        } else {
            throw std::runtime_error("Invalid base orientation type provided to converter.");
        }
        q_pinocchio.tail(njoints_) = jointPositions_rbd;

        // 2. 获取 RbdState 的速度数据，并转换为 Pinocchio v
        // RbdState v part: [base_angular_vel_world(3), base_linear_vel_world(3), joint_velocities(N)]
        const Eigen::Vector3d baseAngularVelocity_world_rbd = rbdStateMeasured.segment<3>(nqGeneralized_);
        const Eigen::Vector3d baseLinearVelocity_world_rbd = rbdStateMeasured.segment<3>(nqGeneralized_ + 3);
        const Eigen::VectorXd jointVelocities_rbd = rbdStateMeasured.segment(nqGeneralized_ + 6, njoints_);

        v_pinocchio.resize(model_.nv);
        
        if (base_orientation_type_ == "quaternion") {
            // @note v_pinocchio = [base_linearVel(base), base_angularVel(base), joint_velocities]
            Eigen::Matrix3d R_wb = getBaseOrientation(q_pinocchio);
            Eigen::Vector3d baseLinearVelocity_base = R_wb.transpose() * baseLinearVelocity_world_rbd;
            Eigen::Vector3d baseAngularVelocity_base = R_wb.transpose() * baseAngularVelocity_world_rbd;

            v_pinocchio.head<3>() = baseLinearVelocity_base;
            v_pinocchio.segment<3>(3) = baseAngularVelocity_base;
        } else if (base_orientation_type_ == "eulerZYX") {
            // @note v_pinocchio = [base_linearVel(world), eulerZyx_dot, joint_velocities]
            v_pinocchio.head<3>() = baseLinearVelocity_world_rbd;

            Eigen::Vector3d rpy = q_pinocchio.segment<3>(3);
            Eigen::Vector3d eulerZyx_dot = ocs2::getEulerAnglesZyxDerivativesFromGlobalAngularVelocity(rpy, baseAngularVelocity_world_rbd);

            v_pinocchio.segment<3>(3) = eulerZyx_dot;
        } else {
            throw std::runtime_error("Invalid base orientation type provided to converter.");
        }
        
        // 关节速度可以直接赋值
        v_pinocchio.tail(njoints_) = jointVelocities_rbd;
    }
    
    /**
    * @brief 计算四个髋关节在世界坐标系下的位置。
    *
    * 此函数接收机器人的**基座（base）配置向量 `q_base`**，以及内部存储的关节数量 `njoints_`。
    * 它首先构建完整的关节配置向量 `q`（基座位姿 + 零值关节角度），
    * 然后执行正向运动学计算，并提取指定髋关节在世界坐标系下的位置。
    *
    * @param q_base 机器人的基座配置向量。其维度根据 `base_orientation_type_` 决定：
    * - 如果 `base_orientation_type_` 为 "quaternion" (四元数)，`q_base` 预期为 7 维
    * (3D 位置 + 4D 四元数)。
    * - 如果 `base_orientation_type_` 为 "eulerZYX" (ZYX 欧拉角)，`q_base` 预期为 6 维
    * (3D 位置 + 3D 欧拉角)。
    * @return 一个包含四个髋关节在世界坐标系下位置的 `Eigen::Vector3d` 向量列表。
    * 这些位置的顺序由 `hipNames_` 成员变量决定，通常预期顺序为：
    * [前左髋关节 (LF_HFE), 后左髋关节 (LH_HFE), 前右髋关节 (RF_HFE), 后右髋关节 (RH_HFE)]。
    * @throws std::runtime_error 如果 `q_base` 的维度与 `base_orientation_type_` 不匹配。
    */
    std::vector<Eigen::Vector3d> computeHipPositions(const Eigen::VectorXd& q_base) {
        // 确保输入的 q_base 维度正确
        // 如果 base_orientation_type_ 是四元数，则 q_base 应为 7 维；如果是欧拉ZYX，则应为 6 维。
        if ((base_orientation_type_ == "quaternion" && q_base.size() != 7) ||
            (base_orientation_type_ == "eulerZYX" && q_base.size() != 6)) {
            throw std::runtime_error("Invalid base orientation type or size of q_base.");
        } 

        // 构建完整的关节配置向量 q。
        // q 的前几维是 q_base（基座的位置和方向），
        // 后面是所有其他关节的关节角度（这里假定为零，即所有其他关节处于其默认零位）。
        Eigen::VectorXd q(q_base.size() + njoints_); // 初始化 q 为正确的大小
        q.head(q_base.size()) = q_base; // 将 q_base 赋值给 q 的头部
        q.tail(njoints_).setZero(); // 将 q 的尾部（其他关节角度）设置为零

        // Step 1: Forward Kinematics (执行正向运动学，计算所有关节和体连杆的位姿)
        pinocchio::forwardKinematics(model_, data_, q);

        // Step 2: Update frame placements (更新所有定义的 Frame 的空间位姿，这些位姿会存储在 data_.oMf 中)
        pinocchio::updateFramePlacements(model_, data_);

        // Step 3: Collect hip positions (收集髋关节的位置)
        std::vector<Eigen::Vector3d> hipPositions;
        for (const auto& hipName : hipNames_) { // 遍历预定义的髋关节名称列表 hipNames_
            if (model_.existFrame(hipName)) { // 检查模型中是否存在这个 Frame
                const pinocchio::FrameIndex frameId = model_.getFrameId(hipName); // 获取 Frame 的唯一 ID
                const pinocchio::SE3& hipPose = data_.oMf[frameId]; // 从 data_.oMf 中获取该 Frame 在世界坐标系下的位姿 (SE3 对象)
                hipPositions.push_back(hipPose.translation()); // 提取位姿的平移部分（即位置），并添加到结果列表中
            } else {
                std::cerr << "Frame '" << hipName << "' not found in model." << std::endl;
                hipPositions.push_back(Eigen::Vector3d::Zero());  // 如果 Frame 不存在，添加一个零向量以避免维度不一致
            }
        }
        return hipPositions;
    }

    /**
    * @brief 计算四個足端在世界坐标系下的位置。
    *
    * 该函数接收机器人的完整状态向量 `rbdState`，使用convertRbdStateToPinocchio函数将其转换为pinocchio的广义关节位置q_pinocchio，执行正向运动学计算，
    * 并提取指定足端（通常是末端执行器）的在世界坐标系下的空间位置（平移部分）。
    *
    * @param rbdState = [eulerZyx, base_position, joint_positions, base_angularVel(world), base_linearVel(world) joint_velocities] 
    * 机器人的完整关节状态向量。其维度应为 `nqGeneralized_*2`，包含基座和所有关节的位置、速度，一般由状态估计器提供。
    * @return 一个包含四个足端在世界坐标系下位置的 `Eigen::Vector3d` 向量列表。
    * 这些位置的顺序由 `feetNames_` 成员变量决定。
    */
    std::vector<Eigen::Vector3d> computeFootPositionsRbd(const Eigen::VectorXd& rbdState) {
        Eigen::VectorXd q_pinocchio, v_pinocchio;
        convertRbdStateToPinocchio(rbdState, q_pinocchio, v_pinocchio);
        return computeFootPositions(q_pinocchio);
    }

    /**
    * @brief 计算四個足端在世界坐标系下的位置。
    *
    * 该函数接收机器人的完整关节配置向量 `q`，执行正向运动学计算，
    * 并提取指定足端（通常是末端执行器）的在世界坐标系下的空间位置（平移部分）。
    *
    * @param q 机器人的完整关节配置向量。其维度应为 `model.nq`，包含基座和所有关节的角度。
    * 对于浮动基座机器人，`q` 通常以基座位置和方向（如四元数或欧拉角）开始，
    * 随后是所有关节的关节角度。
    * @return 一个包含四个足端在世界坐标系下位置的 `Eigen::Vector3d` 向量列表。
    * 这些位置的顺序由 `feetNames_` 成员变量决定。
    */
    std::vector<Eigen::Vector3d> computeFootPositions(const Eigen::VectorXd& q) {
        // Step 1: Forward Kinematics (执行正向运动学，计算所有关节和体连杆的位姿)
        pinocchio::forwardKinematics(model_, data_, q);

        // Step 2: Update frame placements (更新所有定义的 Frame 的空间位姿，这些位姿会存储在 data_.oMf 中)
        pinocchio::updateFramePlacements(model_, data_);

        // Step 3: Collect foot positions (收集足端的位置)
        std::vector<Eigen::Vector3d> footPositions;
        for (const auto& footName : feetNames_) { // 遍历预定义的足端名称列表 feetNames_
            if (model_.existFrame(footName)) { // 检查模型中是否存在这个 Frame
                const pinocchio::FrameIndex frameId = model_.getFrameId(footName); // 获取 Frame 的唯一 ID
                const pinocchio::SE3& footPose = data_.oMf[frameId]; // 从 data_.oMf 中获取该 Frame 在世界坐标系下的位姿 (SE3 对象)
                footPositions.push_back(footPose.translation()); // 提取位姿的平移部分（即位置），并添加到结果列表中
            } else {
                std::cerr << "Frame '" << footName << "' not found in model." << std::endl;
                // 如果 Frame 不存在，添加一个零向量，以确保返回列表的维度一致，防止后续操作出错。
                footPositions.push_back(Eigen::Vector3d::Zero());
            }
        }
        return footPositions;
    }

    Eigen::Matrix3d computeFootJacobian(const Eigen::VectorXd& q, std::string footName) {
        if (!model_.existFrame(footName)) {
            throw std::runtime_error("Foot frame not found: " + footName);
        }
        const pinocchio::FrameIndex frameId = model_.getFrameId(footName);
        Eigen::Matrix<double, 6, Eigen::Dynamic> J;
        J.setZero(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, q, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J.block(0, nvBase_ + 3 * getLegIndex(footName), 3, 3);
    }

    std::vector<Eigen::Matrix3d> computeFootJacobians(const Eigen::VectorXd& q) {
        std::vector<Eigen::Matrix3d> footJacobian;
        for (const auto& footName : feetNames_) {
            if (model_.existFrame(footName)) {
                auto J = computeFootJacobian(q, footName);
                footJacobian.push_back(J);
            } else {
                std::cerr << "Frame '" << footName << "' not found in model." << std::endl;
                footJacobian.push_back(Eigen::Matrix3d::Zero());  // 防止维度不一致
            }
        }
        return footJacobian;
    }

    /*
        DLS (Damped Least Squares):
        find q make: kine(q) = p_target

        kine(q0 + dq) = kine(q0) + J(q0) * dq = p_target
        J(q0) * dq = p_target - kine(q0) = error
        dq = J(q0)^{-1} * error

        but J(q0) is not square, so we use DLS:
        dq = J^T * (J * J^T + λI)^-1 * error
    */ 
    Eigen::Vector3d inverseKinematicsLeg(const std::string& footName, const Eigen::Vector3d& posInBaseFrame,
                                        int max_iters = 1000, double tol = 1e-4, double dt = 0.1, double damping = 1e-6) {
        if (!model_.existFrame(footName)) {
            throw std::runtime_error("Foot frame not found: " + footName);
        }
        const pinocchio::FrameIndex frameId = model_.getFrameId(footName);
        const pinocchio::JointIndex jointId = model_.frames[frameId].parent;

        auto q = q_init_;
        q.head<3>().setZero();  // 基座位置初始化为零
        setBaseOrientationZero(q);
        Eigen::Vector3d err;

        int leg_offset = nqBase_ + 3 * getLegIndex(footName);

        for (int i = 0; i < max_iters; ++i) {
            // 正向运动学和更新
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacement(model_, data_, frameId);

            const Eigen::Vector3d foot_pos = data_.oMf[frameId].translation();
            err = posInBaseFrame - foot_pos;

            // std::cout << "[IK Iter " << i << "] Error norm: " << err.norm()
            //         << ", Foot pos: " << foot_pos.transpose()
            //         << ", Target: " << posInBaseFrame.transpose() << std::endl;

            if (err.norm() < tol) {
                std::cout << "[IK] Converged in " << i << " iterations. Final error: " << err.norm() << std::endl;
                return q.segment<3>(leg_offset);
            }

            // 计算雅可比矩阵
            auto J = computeFootJacobian(q, footName);

            // dq = J^T * (J * J^T + λI)^-1 * err
            Eigen::Matrix3d JT = J.transpose();
            Eigen::Matrix3d JJt = J * JT;
            Eigen::Matrix3d JJt_damped = JJt + damping * Eigen::Matrix3d::Identity();
            Eigen::Vector3d dq = JT * (JJt_damped.ldlt().solve(err));

            q.segment<3>(leg_offset) += dq * dt;

            // 将角度包裹到 [-pi, pi]
            for (int j = 0; j < 3; ++j) {
                double& angle = q[leg_offset + j];
                angle = std::atan2(std::sin(angle), std::cos(angle));
            }
        }

        // 最终没收敛，报错并打印详细信息
        std::cerr << "[IK ERROR] Did not converge after " << max_iters << " iterations." << std::endl;
        std::cerr << "Final error norm: " << err.norm() << std::endl;
        std::cerr << "Final foot position: " << data_.oMf[frameId].translation().transpose() << std::endl;
        std::cerr << "Target position: " << posInBaseFrame.transpose() << std::endl;
        std::cerr << "Final joint values: " << q.segment<3>(leg_offset).transpose() << std::endl;

        // throw std::runtime_error("Inverse kinematics did not converge for " + footName);
    }

    // posInWordFrame: 脚在世界坐标系下的目标位置 q_base: 基座的位姿
    Eigen::VectorXd inverseKinematicsAllFeet(const std::vector<Eigen::Vector3d>& posInWorldFrame,
                                            const Eigen::VectorXd& q_base, const std::string& method = "DLS") {
        if (posInWorldFrame.size() != feetNames_.size()) {
            throw std::runtime_error("Mismatch in number of target positions and foot names");
        }

        if (q_base.size() != nqBase_) {
            throw std::runtime_error("Base pose vector size does not match nqBase_");
        }

        Eigen::VectorXd q(model_.nq);
        q.head(nqBase_) = q_base; // 设置基座位姿

        // 运行正向运动学以获得 base 的 oMf（base frame）
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_); // 更新所有 frame 的位置

        Eigen::Vector3d base_pos = q_base.head<3>();
        Eigen::Matrix3d R = data_.oMf[model_.getFrameId(baseName_)].rotation();

        for (size_t i = 0; i < feetNames_.size(); ++i) {
            Eigen::Vector3d foot_pos_in_base = R.transpose() * (posInWorldFrame[i] - base_pos);
            if (method == "DLS") {
                // 使用 DLS 方法进行逆运动学
                q.segment(nqBase_ + 3 * i, 3) = inverseKinematicsLeg(feetNames_[i], foot_pos_in_base);
            } else if (method == "analytical") {
                // 使用解析方法进行逆运动学
                auto q_ik = robotKinematics_.ikine(feetNames_[i], foot_pos_in_base);
                if (q_ik.empty()) {
                    throw std::runtime_error("No valid IK solution found for " + feetNames_[i]);
                }
                // 如果有多个解，报错
                if (q_ik.size() > 1) {
                    // throw std::runtime_error("Multiple IK solutions found for " + feetNames_[i]);
                    ROS_ERROR_STREAM("Multiple IK solutions found for " + feetNames_[i]);
                }
                q.segment(nqBase_ + 3 * i, 3) = q_ik[0];
            }
        }

        return q;
    }

    void printJointNamesInQOrder(const pinocchio::Model &model) {
        std::size_t q_idx = 0;
        for (pinocchio::JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id) {
            const std::string& joint_name = model.names[joint_id];
            std::size_t nq = model.joints[joint_id].nq();
            std::size_t nv = model.joints[joint_id].nv(); // 可选，用于说明速度部分

            std::cout << "Joint " << joint_id << " (" << joint_name << "): ";
            std::cout << "q[" << q_idx << " to " << q_idx + nq - 1 << "]" 
                    << " (nq=" << nq << ", nv=" << nv << ")\n";

            q_idx += nq;
        }
    }

    void printAllFrames() {
        std::cout << "type: 2(joint) 4(fixed) 8(body)" << std::endl;
        for (size_t i = 0; i < model_.frames.size(); ++i) {
            std::cout << "Frame " << i << ": " << model_.frames[i].name
                    << ", type = " << model_.frames[i].type << std::endl;
        }
    }

    void printFootPositions(const std::vector<Eigen::Vector3d>& footPositions) {
        // 输出脚位置
        std::cout << "Foot positions:" << std::endl;
        for (size_t i = 0; i < footPositions.size(); ++i) {
            std::cout << "Foot " << feetNames_[i] << ": " << footPositions[i].transpose() << std::endl;
        }
    }

    /**
     * @brief 验证 Pinocchio 浮动基模型中 v 向量与物理速度的关系。
     *
     * 此函数通过执行正向运动学，并使用 pinocchio::getFrameVelocity
     * 计算基座在世界坐标系下的物理速度，与输入的 v 向量进行比较。
     * 这有助于理解不同基座朝向表示下 v 向量的含义。
     *
     * @param q 机器人的关节配置向量。
     * @param v 机器人的速度向量。
     */
    void verifyBaseVelocityRelationship(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
        // Step 1: 更新运动学状态，包括所有关节和帧的位姿
        pinocchio::forwardKinematics(model_, data_, q, v);
        
        // Step 2: 获取基座 (base) 帧的 ID
        if (!model_.existFrame(baseName_)) {
            std::cerr << "Frame '" << baseName_ << "' not found in model." << std::endl;
            return;
        }
        const pinocchio::FrameIndex base_frame_id = model_.getFrameId(baseName_);
        
        // Step 3: 使用 getFrameVelocity 计算基座在身体坐标系下的物理速度
        auto v_base_local = pinocchio::getFrameVelocity(model_, data_, base_frame_id, pinocchio::ReferenceFrame::LOCAL).toVector();
        auto v_base_world = pinocchio::getFrameVelocity(model_, data_, base_frame_id, pinocchio::ReferenceFrame::WORLD).toVector();
        auto v_base_align = pinocchio::getFrameVelocity(model_, data_, base_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).toVector();

        // Step 5: 打印结果并比较
        std::cout << "--- 验证: " << (base_orientation_type_ == "quaternion" ? "四元数" : "欧拉角ZYX") << "浮动基 ---" << std::endl;
        std::cout << "v     : " << v.head<6>().transpose() << std::endl;
        std::cout << "data.v: " << data_.v[model_.getJointId("root_joint")].toVector().transpose() << std::endl;
        std::cout << "local : " << v_base_local.transpose() << std::endl;
        std::cout << "world : " << v_base_world.transpose() << std::endl;
        std::cout << "align : " << v_base_align.transpose() << std::endl;
        
        if (base_orientation_type_ == "quaternion") {
            Eigen::VectorXd v_base_quat(6);
            v_base_quat << getBaseOrientation(q) * v.head<3>(), getBaseOrientation(q) * v.segment<3>(3); // 将线速度和角速度组合成一个6维向量
            std::cout << "quat  : " << v_base_quat.transpose() << std::endl;
        } else if (base_orientation_type_ == "eulerZYX") {
            // 如果是欧拉角ZYX，计算对应的雅可比矩阵
            Eigen::Vector3d rpy = q.segment<3>(3);  // 提取欧拉角ZYX部分
            Eigen::Vector3d rpy_dot = v.segment<3>(3);  // 提取欧拉角ZYX的导数部分
            Eigen::VectorXd v_base_eulerZyx(6);
            // v_base_eulerZyx << v.head<3>(), pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED) * rpy_dot;
            v_base_eulerZyx << v.head<3>(), ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives(rpy, rpy_dot);
            std::cout << "euler : " << v_base_eulerZyx.transpose() << std::endl;
        }
    }

    /**
     * @brief 测试函数，演示如何使用 RobotModel 类的功能。
     * 
     * 该函数随机生成关节配置和速度向量，计算髋关节和足端位置，
     * 执行逆运动学，并输出相关信息。
     */
    void test() {
        // 随机初始化位姿 q（维度 nq）和速度 v（维度 nv）
        Eigen::VectorXd q = pinocchio::randomConfiguration(model_);
        Eigen::VectorXd v = Eigen::VectorXd::Random(model_.nv);

        verifyBaseVelocityRelationship(q, v);

        // setBaseOrientationZero(q);

        // q = q_init_;  // 使用初始化的 q
        // v = v_init_;  // 使用初始化的 v

        auto footPositions = computeFootPositions(q);
        auto footJacobians = computeFootJacobians(q);

        Eigen::VectorXd q_ik = inverseKinematicsAllFeet(footPositions, q.head(nqBase_), "DLS");
        Eigen::VectorXd q_ik_analy = inverseKinematicsAllFeet(footPositions, q.head(nqBase_), "analytical");

        auto footPositions_ik = computeFootPositions(q_ik);

        // 输出格式化
        std::cout << std::fixed << std::setprecision(4); // 设置全局精度为4位

        // 输出所有关节名称和对应的q索引
        // printJointNamesInQOrder(model_);

        // 输出所有Frame信息
        // printAllFrames();

        // 输出基座朝向类型
        if (base_orientation_type_ == "quaternion") {
            std::cout << "Using quaternion representation for base orientation." << std::endl;
        } else if (base_orientation_type_ == "eulerZYX") {
            std::cout << "Using Euler ZYX representation for base orientation." << std::endl;
        }

        // 输出上下限
        std::cout << "Position limits: \n" 
                  << model_.lowerPositionLimit.transpose().tail(njoints_) << "\n" 
                  << model_.upperPositionLimit.transpose().tail(njoints_) << std::endl;

        // 输出随机生成的q和v
        std::cout << "DOF: nq = " << model_.nq << ", nv = " << model_.nv << std::endl;
        std::cout << "Random q (position):\n" << q.transpose() << std::endl;
        std::cout << "Random v (velocity):\n" << v.transpose() << std::endl;

        // 输出逆运动学结果
        std::cout << "Recovered q from IK (DLS):\n" << q_ik.transpose() << std::endl;
        std::cout << "Recovered q from IK (ANA):\n" << q_ik_analy.transpose() << std::endl;

        // 输出脚位置
        std::cout << "Foot positions:" << std::endl;
        for (size_t i = 0; i < footPositions.size(); ++i) {
            std::cout << "Foot " << feetNames_[i] << ": " << footPositions[i].transpose() << " ; " << footPositions_ik[i].transpose() << std::endl;
        }

        // // 输出脚雅可比矩阵
        // std::cout << "Foot Jacobians:" << std::endl;
        // for (size_t i = 0; i < footJacobians.size(); ++i) {
        //     std::cout << "Foot " << feetNames_[i] << " Jacobian:\n"
        //               << footJacobians[i] << std::endl;
        // }

        q.head(3) << 0, 0, 0.15;
        setBaseOrientationZero(q);
        auto hipPositions = computeHipPositions(q.head(nqBase_)); // 计算髋关节位置
        footPositions.clear(); // 清空脚位置列表
        for (const auto& hipPos : hipPositions) {
            auto footPos = hipPos;
            footPos[0] -= 0.0048;
            footPos[2] = 0;
            footPositions.push_back(footPos);
        }
        q = inverseKinematicsAllFeet(footPositions, q.head(nqBase_), "DLS"); // 逆运动学求解

        std::cout << "q(0 0 1.5):" << q.transpose() << std::endl;

        pinocchio::centerOfMass(model_, data_, q); // 计算质心位置
        std::cout << "Center of Mass: " << data_.com[0].transpose() << std::endl;
        
        // 计算zmp位置
        Eigen::Vector3d zmp;
        for (const auto& fooPos : footPositions) {
            zmp += fooPos;
        }
        zmp /= footPositions.size();
        std::cout << "ZMP position: " << zmp.transpose() << std::endl;

        // // 输出髋位置
        std::cout << "Hip positions:" << std::endl;
        for (size_t i = 0; i < hipPositions.size(); ++i) {
            std::cout << "Hip " << hipNames_[i] << ": " << hipPositions[i].transpose() << std::endl;
        }

        // // 输出脚位置
        // std::cout << "Foot positions:" << std::endl;
        // for (size_t i = 0; i < footPositions.size(); ++i) {
        //     std::cout << "Foot " << feetNames_[i] << ": " << footPositions[i].transpose() << std::endl;
        // }
    }

private:
    void init() {
        if (base_orientation_type_ == "quaternion") {
            nqBase_ = 7;  // 四元数表示的基座有7个自由度
            q_init_.resize(model_.nq);
            q_init_ << 0, 0, 0.3, 0, 0, 0, 1,
                    -0.20, 0.72, -1.44,
                    -0.20, 0.72, -1.44,
                    0.20, 0.72, -1.44,
                    0.20, 0.72, -1.44;
            v_init_ = Eigen::VectorXd::Zero(model_.nv);
        } else if (base_orientation_type_ == "eulerZYX") {
            nqBase_ = 6;  // 欧拉ZYX表示的基座有6个自由度
            q_init_.resize(model_.nq);
            q_init_ << 0, 0, 0.3, 0, 0, 0,
                    -0.20, 0.72, -1.44,
                    -0.20, 0.72, -1.44,
                    0.20, 0.72, -1.44,
                    0.20, 0.72, -1.44;
            v_init_ = Eigen::VectorXd::Zero(model_.nv);
        } else {
            throw std::runtime_error("Invalid base orientation type: " + base_orientation_type_);
        }
    }

    // 设置上下限
    void setLimits() {
        // Translation bounds
        model_.lowerPositionLimit.head<3>().setConstant(-10.0);  // x, y, z
        model_.upperPositionLimit.head<3>().setConstant(10.0);

        // Orientation（四元数不设置限制，EulerZYX可以设置为 -pi 到 pi）
        if (base_orientation_type_ == "eulerZYX") {
            model_.lowerPositionLimit.segment<3>(3).setConstant(-M_PI);
            model_.upperPositionLimit.segment<3>(3).setConstant(M_PI);
        }

        // 设置关节限制 避开奇异点
        for (const auto& footName : feetNames_) {
            model_.lowerPositionLimit[nqBase_ + 3*getLegIndex(footName) + 1] = -0.1618;
            model_.upperPositionLimit[nqBase_ + 3*getLegIndex(footName) + 1] = M_PI/2;
        }
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::string base_orientation_type_ = "quaternion";  // 可选 "quaternion" 或 "eulerZYX"
    int nqBase_ = 7;   // 浮动基的自由度 7 or 6
    int nvBase_ = 6;   // 浮动基的速度自由度
    int njoints_ = 12; // 关节数目
    int nqGeneralized_ = 18; // 总的广义坐标数目（基座 + 关节）
    std::string baseName_ = "base"; // 基座名称
    std::vector<std::string> hipNames_ = {"LF_hip", "LH_hip", "RF_hip", "RH_hip"}; // 机器人脚的名称
    std::vector<std::string> feetNames_ = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"}; // 机器人脚的名称
    std::vector<Eigen::Vector3d> initFeetPositions_ = {
        Eigen::Vector3d( 0.1881,  0.12675, 0),
        Eigen::Vector3d(-0.1881,  0.12675, 0),
        Eigen::Vector3d( 0.1881, -0.12675, 0),
        Eigen::Vector3d(-0.1881, -0.12675, 0)
    };
    Eigen::VectorXd q_init_, v_init_;  // 初始位姿和速度

    RobotKinematics robotKinematics_;  //
};
