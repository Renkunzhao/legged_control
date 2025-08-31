//
// Created by qiayuan on 2022/6/24.
//

#include <string>
#include <vector>
#include <memory>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <fstream>
#include <sstream>

#include "legged_controllers/LeggedController.h"
#include "legged_controllers/RobotModel.h"
#include "legged_interface/LeggedInterface.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"

#include <pinocchio/algorithm/centroidal.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

// 回调函数
void LeggedController::jumpCallback(const std_msgs::Bool::ConstPtr& msg)
{
    should_jump_ = msg->data;
    ROS_INFO("Received jump: %s", should_jump_ ? "true" : "false");
}

bool legged::LeggedController::loadTrajectory(const std::string& filename) {
  trajectory_.clear();
  std::ifstream file(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Failed to open trajectory file: " << filename);
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    TrajectoryPoint point;
    std::string value;

    std::getline(ss, value, ',');
    point.time = std::stod(value);

    std::getline(ss, value, ',');
    point.x = std::stod(value);

    std::getline(ss, value, ',');
    point.dx = std::stod(value);

    std::getline(ss, value, ',');
    point.F = std::stod(value);

    trajectory_.emplace_back(point);
  }

  file.close();
  ROS_INFO_STREAM("Loaded trajectory with " << trajectory_.size() << " points from: " << filename);
  return true;
}

TrajectoryPoint LeggedController::interpolateTrajectory(double t) {
  const double dt = 0.001;  // 1kHz
  size_t index = static_cast<size_t>(t / dt);

  if (index >= trajectory_.size()) {
    index = trajectory_.size() - 1;  // 保持最后一个点
    should_jump_ = false;  // 如果超出范围，停止跳跃
    jumpStarted_ = false;
    ROS_WARN("Index out of bounds, using last trajectory point.");
  }

  // 跳跃状态机
  if(jumpState_ == "jumping" && trajectory_[index].F == 0){
    jumpState_ = "flight";
  }

  return trajectory_[index];
}

bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  std::cout << "[LeggedController]: Init." << std::endl;

  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  mappingPtr_ = std::make_shared<CentroidalModelPinocchioMapping>(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), *mappingPtr_,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), *mappingPtr_, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  leggedState_.init(leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
  leggedState_.createCustomState("q_pin", {"base_pos", "base_eulerZYX", "joint_pos"});
  leggedState_.createCustomState("v_pin", {"base_lin_vel_W", "base_eulerZYX_dot", "joint_vel"});

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>();
  std::string wbcConfigFile;
  controller_nh.getParam("/wbcConfigFile", wbcConfigFile);
  std::cout << "[LeggedController]: " << wbcConfigFile << std::endl;
  wbc_->loadTasksSetting(wbcConfigFile);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  logger_ = std::make_unique<CsvLogger>("/tmp/legged_control_log");

  robotFreeFlyer_ = std::make_shared<RobotModel>(urdfFile, "quaternion");
  robotComposite_ = std::make_shared<RobotModel>(urdfFile, "eulerZYX");

  robotFreeFlyer_->test();
  robotComposite_->test();

  // 订阅 /jump 话题
  jump_sub_ = nh.subscribe<std_msgs::Bool>("/jump", 10, boost::bind(&LeggedController::jumpCallback, this, _1));

  std::string traj_path;
  if (!controller_nh.getParam("/trajectory_path", traj_path)) {
    ROS_ERROR("trajectory_path not found in param server!");
    return false;
  }

  ROS_INFO_STREAM("Trajectory path received: " << traj_path);
  if (!loadTrajectory(traj_path)) {
    ROS_ERROR("Failed to load trajectory file.");
    return false;
  }

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.

  vector_t torque(12), posDes(12), velDes(12);

  // 跳跃控制开始判断
  if (should_jump_) {
    if (!jumpStarted_) {
      jumpStartTime_ = time;  // 记录跳跃起始时间
      jumpStarted_ = true;
      jumpState_ = "jumping";  // 设置跳跃状态
      initial_com_ = measuredRbdState_.head(6);  // 记录初始质心位置
      initial_q_ = measuredRbdState_.segment(6, 12);
      initial_footPositions_.clear();
      initial_footPositions_ = robotFreeFlyer_->computeFootPositionsRbd(measuredRbdState_);
      ROS_INFO_STREAM("[LeggedController] Jump started at: " << jumpStartTime_);
      ROS_INFO_STREAM("initial_q_: " << initial_q_.transpose());
      robotFreeFlyer_->printFootPositions(initial_footPositions_);
    }

    if (jumpState_ == "jumping") {
      std::string method = "WBC"; // 控制方法
      if (method == "PD"){ // pd + feedforward 
        // 计算轨迹时间（单位秒）
        double trajTime = (time - jumpStartTime_).toSec();
        // 插值轨迹控制：该函数负责根据轨迹插值得到优化状态和输入
        const auto& point =  interpolateTrajectory(trajTime);
        // 根据z=point.x计算关节位置
        Eigen::VectorXd q(robotFreeFlyer_->nq());
        q.head<3>() << 0, 0, point.x; // 设置基座位置
        robotFreeFlyer_->setBaseOrientationZero(q); // 设置基座方向为零
        auto footPositions = robotFreeFlyer_->getInitfeetPositions();
        q = robotFreeFlyer_->inverseKinematicsAllFeet(footPositions, q.head(robotFreeFlyer_->nqBase()), "DLS"); // 逆运动学求解
        posDes = q.tail(12);

        torque.setZero(12); // 设置力矩为零
        velDes.setZero(12); // 设置速度为零
        auto jacobians = robotFreeFlyer_->computeFootJacobians(q); // 计算脚端雅可比矩阵
        for (size_t i = 0; i < jacobians.size(); ++i) {
          Eigen::Vector3d force;
          force << 0, 0, point.F / 4.0; // 每个脚端施加的力
          torque.segment(3 * i, 3) = - jacobians[i].transpose() * force; // PD控制

          Eigen::Vector3d footVel;
          footVel << 0, 0, -point.dx; // 脚端速度
          velDes.segment(3 * i, 3) = jacobians[i].ldlt().solve(footVel); // 设置脚端速度
        }

        for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
          hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
        }
      } else if (method == "WBC") { // WBC 跳跃
        { // 静止站立测试
          // optimizedState = Eigen::VectorXd::Zero(24);
          // optimizedInput = Eigen::VectorXd::Zero(24);

          // // 设置 z, vz
          // optimizedState(8) = 0.15;
          // optimizedState.tail(12) = initial_q_; // 使用初始关节位置
          // // optimizedState.tail(12) << -0.5621, 1.3056, -2.5357,
          // //                              -0.5621, 1.3056, -2.5357,
          // //                                0.5621,  1.3056, -2.5357,
          // //                                0.5621,  1.3056, -2.5357;

          // double force_per_leg = pinocchio::computeTotalMass(robotFreeFlyer_->getModel()) * 9.81 / 4.0;
          // optimizedInput(2) = force_per_leg;
          // optimizedInput(5) = force_per_leg;
          // optimizedInput(8) = force_per_leg;
          // optimizedInput(11) = force_per_leg;
        }

        { // sin wave
          // optimizedState = Eigen::VectorXd::Zero(24);
          // optimizedInput = Eigen::VectorXd::Zero(24);

          // // 设置 z, vz
          // optimizedState(8) = 0.3 - 0.1 * std::cos( M_PI * (time - jumpStartTime_).toSec());
          // optimizedState(2) = - 0.2 * std::sin(M_PI * (time - jumpStartTime_).toSec());

          // Eigen::VectorXd q_base(robotFreeFlyer_->nqBase());
          // q_base.head<3>() = optimizedState.segment(6, 3); // 设置基座位置
          // robotFreeFlyer_->setBaseOrientationZero(q_base); // 设置基座方向为零
          // optimizedState.tail(12) = robotFreeFlyer_->inverseKinematicsAllFeet(initial_footPositions_, q_base, "DLS").tail(12); // 逆运动学求解

          // double force_per_leg = pinocchio::computeTotalMass(robotFreeFlyer_->getModel()) * 9.81 / 4.0;
          // optimizedInput(2) = force_per_leg;
          // optimizedInput(5) = force_per_leg;
          // optimizedInput(8) = force_per_leg;
          // optimizedInput(11) = force_per_leg;
        }

        { // jump
          // 计算轨迹时间（单位秒）
          double trajTime = (time - jumpStartTime_).toSec();
          // 插值轨迹控制：该函数负责根据轨迹插值得到优化状态和输入
          const auto& point =  interpolateTrajectory(trajTime);

          optimizedState = Eigen::VectorXd::Zero(24);
          optimizedInput = Eigen::VectorXd::Zero(24);

          // 设置 z, vz
          optimizedState(8) = point.x;
          optimizedState(2) = point.dx;

          Eigen::VectorXd q_base(robotFreeFlyer_->nqBase());
          q_base.head<3>() = optimizedState.segment(6, 3); // 设置基座位置
          robotFreeFlyer_->setBaseOrientationZero(q_base); // 设置基座方向为零
          optimizedState.tail(12) = robotFreeFlyer_->inverseKinematicsAllFeet(robotFreeFlyer_->getInitfeetPositions(), q_base, "DLS").tail(12); // 逆运动学求解

          // 设置 接触力
          double force_per_leg = point.F / 4.0;
          optimizedInput(2) = force_per_leg;
          optimizedInput(5) = force_per_leg;
          optimizedInput(8) = force_per_leg;
          optimizedInput(11) = force_per_leg;
        }

        currentObservation_.input = optimizedInput;

        plannedMode = 15; // STANCE

        auto rbdState = measuredRbdState_;
        rbdState[0] -= initial_com_[0]; // 调整质心位置
        rbdState.segment(3, 2) -= initial_com_.segment(3, 2); // 调整质心位置

        wbcTimer_.startTimer();
        mappingPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface()); 
        const auto qDesired = mappingPtr_->getPinocchioJointPosition(optimizedState);

        const auto& model = leggedInterface_->getPinocchioInterface().getModel();
        auto& data = leggedInterface_->getPinocchioInterface().getData();
        pinocchio::computeCentroidalMap(model, data, qDesired);
        const auto vDesired = mappingPtr_->getPinocchioJointVelocity(optimizedState, optimizedInput);

        leggedState_.setFromRbdState(measuredRbdState_);
        vector_t x = wbc_->update(qDesired, vDesired, optimizedInput.head(12), 
                                  leggedState_.custom_state("q_pin"),
                                  leggedState_.custom_state("v_pin"),
                                 modeNumber2StanceLeg(plannedMode), period.toSec(), "pd");
        wbcTimer_.endTimer();

        torque = x.tail(12);

        posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
        velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

        for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
          hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 50, 3, torque(j));
        }

        // logger_->log("dotu", x.head(18));
        // logger_->log("F", x.segment(18, 12));
        // logger_->log("tail", x.tail( 12));
        // logger_->log("F_des", optimizedInput.head(12));
      }
    } else if (jumpState_ == "flight") {
      Eigen::VectorXd q(robotFreeFlyer_->nq());
      q.head<3>() << 0, 0, 0.15; // 设置基座位置
      robotFreeFlyer_->setBaseOrientationZero(q); // 设置基座方向为零
      auto footPositions = robotFreeFlyer_->getInitfeetPositions();
      q = robotFreeFlyer_->inverseKinematicsAllFeet(footPositions, q.head(robotFreeFlyer_->nqBase()), "DLS"); // 逆运动学求解
      posDes = q.tail(12);
      velDes.setZero(12);
      torque.setZero(12);
      for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
        hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 50, 3, torque(j));
      }

      if ((time - jumpStartTime_).toSec() > 2) { // 假设飞行时间为0.5秒
        jumpState_ = "stand"; // 跳跃结束，进入站立状态
        jumpStarted_ = false;
        should_jump_ = false;
        ROS_INFO_STREAM("[LeggedController] Jump ended at: " << time);
      }
    }

  } else {
    // 默认的基于 MPC 的控制逻辑
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  mappingPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface()); 
  const auto qDesired = mappingPtr_->getPinocchioJointPosition(optimizedState);

  const auto& model = leggedInterface_->getPinocchioInterface().getModel();
  auto& data = leggedInterface_->getPinocchioInterface().getData();
  pinocchio::computeCentroidalMap(model, data, qDesired);
  const auto vDesired = mappingPtr_->getPinocchioJointVelocity(optimizedState, optimizedInput);

  leggedState_.setFromRbdState(measuredRbdState_);
  vector_t x = wbc_->update(qDesired, vDesired, optimizedInput.head(12), 
                          leggedState_.custom_state("q_pin"),
                          leggedState_.custom_state("v_pin"),
                          modeNumber2StanceLeg(plannedMode), period.toSec(), "pd");wbcTimer_.endTimer();

  torque = x.tail(12);

  posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
  }
  }

  Eigen::VectorXd q_pinocchio, v_pinocchio;
  robotFreeFlyer_->convertRbdStateToPinocchio(measuredRbdState_, q_pinocchio, v_pinocchio);
  pinocchio::centerOfMass(robotFreeFlyer_->getModel(), robotFreeFlyer_->getData(), q_pinocchio, v_pinocchio);
  auto footPositions = robotFreeFlyer_->computeFootPositions(q_pinocchio);

  vector_t jointEffort(hybridJointHandles_.size());
  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointEffort(i) = hybridJointHandles_[i].getEffort();
  }

  logger_->log("time", ros::Time::now().toSec());
  logger_->log("state", optimizedState);
  logger_->log("input", optimizedInput);

  logger_->log("rbdst_state", measuredRbdState_);
  logger_->log("torque", jointEffort);

  // logger_->log("z", point.x);
  // logger_->log("dz", point.dx);
  logger_->log("CoM", robotFreeFlyer_->getData().com[0]);
  logger_->log("CoM_vel", robotFreeFlyer_->getData().vcom[0]);
  logger_->log("foot_positions(LF)", footPositions[0]);
  logger_->log("foot_positions(LH)", footPositions[1]);
  logger_->log("foot_positions(RF)", footPositions[2]);
  logger_->log("foot_positions(RH)", footPositions[3]);

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

  logger_->flush();
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
