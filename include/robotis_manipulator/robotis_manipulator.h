/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

/**
 * @file robotis_manipulator.h
 * @brief
 * @details
 */

#ifndef ROBOTIS_MANIPULATOR_H_
#define ROBOTIS_MANIPULATOR_H_

#include "robotis_manipulator_common.h"
#include "robotis_manipulator_manager.h"
#include "robotis_manipulator_trajectory_generator.h"
#include "robotis_manipulator_math.h"
#include "robotis_manipulator_log.h"

#include <algorithm>
/**
 * @namespace robotis_manipulator
 * @brief main namespace
 */
namespace robotis_manipulator
{
/**
 * @brief The RobotisManipulator class
 */
class RobotisManipulator
{
private:
  Manipulator manipulator_;
  Trajectory trajectory_;
  Kinematics *kinematics_;
  std::map<Name, JointActuator *> joint_actuator_;
  std::map<Name, ToolActuator *> tool_actuator_;

  bool trajectory_initialized_state_;
  bool joint_actuator_added_stete_;
  bool tool_actuator_added_stete_;
  bool moving_state_;
  bool step_moving_state_;

private:
  /**
   * @brief startMoving
   */
  void startMoving();
  /**
   * @brief getTrajectoryJointValue
   * @param tick_time
   * @return
   */
  JointWaypoint getTrajectoryJointValue(double tick_time);

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();


  /*****************************************************************************
  ** Initialize Function
  *****************************************************************************/
  /**
   * @brief addWorld
   * @param world_name
   * @param child_name
   * @param world_position
   * @param world_orientation
   */
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());
  /**
   * @brief addJoint
   * @param my_name
   * @param parent_name
   * @param child_name
   * @param relative_position
   * @param relative_orientation
   * @param axis_of_rotation
   * @param joint_actuator_id
   * @param max_position_limit
   * @param min_position_limit
   * @param coefficient
   * @param mass
   * @param inertia_tensor
   * @param center_of_mass
   */
  void addJoint(Name my_name,
                Name parent_name,
                Name child_name,
                Eigen::Vector3d relative_position,
                Eigen::Matrix3d relative_orientation,
                Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                int8_t joint_actuator_id = -1, 
                double max_position_limit = M_PI, 
                double min_position_limit = -M_PI,
                double coefficient = 1.0,
                double mass = 0.0,
                Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());
  /**
   * @brief addTool
   * @param my_name
   * @param parent_name
   * @param relative_position
   * @param relative_orientation
   * @param tool_id
   * @param max_position_limit
   * @param min_position_limit
   * @param coefficient
   * @param mass
   * @param inertia_tensor
   * @param center_of_mass
   */
  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1, 
               double max_position_limit =M_PI, 
               double min_position_limit = -M_PI,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());
  /**
   * @brief addComponentChild
   * @param my_name
   * @param child_name
   */
  void addComponentChild(Name my_name, Name child_name);
  /**
   * @brief printManipulatorSetting
   */
  void printManipulatorSetting();
  /**
   * @brief addKinematics
   * @param kinematics
   */
  void addKinematics(Kinematics *kinematics);
  /**
   * @brief addJointActuator
   * @param actuator_name
   * @param joint_actuator
   * @param id_array
   * @param arg
   */
  void addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg);
  /**
   * @brief addToolActuator
   * @param tool_name
   * @param tool_actuator
   * @param id
   * @param arg
   */
  void addToolActuator(Name tool_name, ToolActuator *tool_actuator, uint8_t id, const void *arg);
  /**
   * @brief addCustomTrajectory
   * @param trajectory_name
   * @param custom_trajectory
   */
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  /**
   * @brief addCustomTrajectory
   * @param trajectory_name
   * @param custom_trajectory
   */
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);


  /*****************************************************************************
  ** Manipulator Function
  *****************************************************************************/
  /**
   * @brief getManipulator
   * @return
   */
  Manipulator *getManipulator();
  /**
   * @brief getJointValue
   * @param joint_name
   * @return
   */
  JointValue getJointValue(Name joint_name);
  /**
   * @brief getToolValue
   * @param tool_name
   * @return
   */
  JointValue getToolValue(Name tool_name);
  /**
   * @brief getAllActiveJointValue
   * @return
   */
  std::vector<JointValue> getAllActiveJointValue();
  /**
   * @brief getAllJointValue
   * @return
   */
  std::vector<JointValue> getAllJointValue();
  /**
   * @brief getAllToolPosition
   * @return
   */
  std::vector<double> getAllToolPosition();
  /**
   * @brief getAllToolValue
   * @return
   */
  std::vector<JointValue> getAllToolValue();
  /**
   * @brief getKinematicPose
   * @param component_name
   * @return
   */
  KinematicPose getKinematicPose(Name component_name);
  /**
   * @brief getDynamicPose
   * @param component_name
   * @return
   */
  DynamicPose getDynamicPose(Name component_name);
  /**
   * @brief getPose
   * @param component_name
   * @return
   */
  Pose getPose(Name component_name);


  /*****************************************************************************
  ** Kinematics Function (Including Virtual Function)
  *****************************************************************************/
  /**
   * @brief jacobian
   * @param tool_name
   * @return
   */
  Eigen::MatrixXd jacobian(Name tool_name);
  /**
   * @brief solveForwardKinematics
   */
  void solveForwardKinematics();
  /**
   * @brief solveInverseKinematics
   * @param tool_name
   * @param goal_pose
   * @param goal_joint_value
   * @return
   */
  bool solveInverseKinematics(Name tool_name, Pose goal_pose, std::vector<JointValue> *goal_joint_value);
  /**
   * @brief setKinematicsOption
   * @param arg
   */
  void setKinematicsOption(const void* arg);


  /*****************************************************************************
  ** Actuator Function (Including Virtual Function)
  *****************************************************************************/
  /**
   * @brief setJointActuatorMode
   * @param actuator_name
   * @param id_array
   * @param arg
   */
  void setJointActuatorMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  /**
   * @brief setToolActuatorMode
   * @param actuator_name
   * @param arg
   */
  void setToolActuatorMode(Name actuator_name, const void *arg);
  /**
   * @brief getJointActuatorId
   * @param actuator_name
   * @return
   */
  std::vector<uint8_t> getJointActuatorId(Name actuator_name);
  /**
   * @brief getToolActuatorId
   * @param actuator_name
   * @return
   */
  uint8_t getToolActuatorId(Name actuator_name);
  /**
   * @brief enableActuator
   * @param actuator_name
   */
  void enableActuator(Name actuator_name);
  /**
   * @brief disableActuator
   * @param actuator_name
   */
  void disableActuator(Name actuator_name);
  /**
   * @brief enableAllJointActuator
   */
  void enableAllJointActuator();
  /**
   * @brief disableAllJointActuator
   */
  void disableAllJointActuator();
  /**
   * @brief enableAllToolActuator
   */
  void enableAllToolActuator();
  /**
   * @brief disableAllToolActuator
   */
  void disableAllToolActuator();
  /**
   * @brief enableAllActuator
   */
  void enableAllActuator();
  /**
   * @brief disableAllActuator
   */
  void disableAllActuator();
  /**
   * @brief getActuatorEnabledState
   * @param actuator_name
   * @return
   */
  bool getActuatorEnabledState(Name actuator_name);
  /**
   * @brief sendJointActuatorValue
   * @param joint_component_name
   * @param value
   * @return
   */
  bool sendJointActuatorValue(Name joint_component_name, JointValue value);
  /**
   * @brief sendMultipleJointActuatorValue
   * @param joint_component_name
   * @param value_vector
   * @return
   */
  bool sendMultipleJointActuatorValue(std::vector<Name> joint_component_name, std::vector<JointValue> value_vector);
  /**
   * @brief sendAllJointActuatorValue
   * @param value_vector
   * @return
   */
  bool sendAllJointActuatorValue(std::vector<JointValue> value_vector);
  /**
   * @brief receiveJointActuatorValue
   * @param joint_component_name
   * @return
   */
  JointValue receiveJointActuatorValue(Name joint_component_name);
  /**
   * @brief receiveMultipleJointActuatorValue
   * @param joint_component_name
   * @return
   */
  std::vector<JointValue> receiveMultipleJointActuatorValue(std::vector<Name> joint_component_name);
  /**
   * @brief receiveAllJointActuatorValue
   * @return
   */
  std::vector<JointValue> receiveAllJointActuatorValue();
  /**
   * @brief sendToolActuatorValue
   * @param tool_component_name
   * @param value
   * @return
   */
  bool sendToolActuatorValue(Name tool_component_name, JointValue value);
  /**
   * @brief sendMultipleToolActuatorValue
   * @param tool_component_name
   * @param value_vector
   * @return
   */
  bool sendMultipleToolActuatorValue(std::vector<Name> tool_component_name, std::vector<JointValue> value_vector);
  /**
   * @brief sendAllToolActuatorValue
   * @param value_vector
   * @return
   */
  bool sendAllToolActuatorValue(std::vector<JointValue> value_vector);
  /**
   * @brief receiveToolActuatorValue
   * @param tool_component_name
   * @return
   */
  JointValue receiveToolActuatorValue(Name tool_component_name);
  /**
   * @brief receiveMultipleToolActuatorValue
   * @param tool_component_name
   * @return
   */
  std::vector<JointValue> receiveMultipleToolActuatorValue(std::vector<Name> tool_component_name);
  /**
   * @brief receiveAllToolActuatorValue
   * @return
   */
  std::vector<JointValue> receiveAllToolActuatorValue();


  /*****************************************************************************
  ** Time Function
  *****************************************************************************/
  /**
   * @brief getTrajectoryMoveTime
   * @return
   */
  double getTrajectoryMoveTime();
  /**
   * @brief getMovingState
   * @return
   */
  bool getMovingState();


  /*****************************************************************************
  ** Check Joint Limit Function
  *****************************************************************************/
  /**
   * @brief checkJointLimit
   * @param component_name
   * @param position
   * @return
   */
  bool checkJointLimit(Name component_name, double position);
  /**
   * @brief checkJointLimit
   * @param component_name
   * @param value
   * @return
   */
  bool checkJointLimit(Name component_name, JointValue value);
  /**
   * @brief checkJointLimit
   * @param component_name
   * @param position_vector
   * @return
   */
  bool checkJointLimit(std::vector<Name> component_name, std::vector<double> position_vector);
  /**
   * @brief checkJointLimit
   * @param component_name
   * @param value_vector
   * @return
   */
  bool checkJointLimit(std::vector<Name> component_name, std::vector<JointValue> value_vector);


  /*****************************************************************************
  ** Trajectory Control Fuction
  *****************************************************************************/
  /**
   * @brief getTrajectory
   * @return
   */
  Trajectory *getTrajectory();
  /**
   * @brief makeJointTrajectoryFromPresentPosition
   * @param delta_goal_joint_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectoryFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param goal_joint_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(std::vector<double> goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param goal_joint_value
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(std::vector<JointValue> goal_joint_value, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param tool_name
   * @param goal_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param tool_name
   * @param goal_orientation
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param tool_name
   * @param goal_pose
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectoryFromPresentPose
   * @param tool_name
   * @param position_meter
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Vector3d position_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectoryFromPresentPose
   * @param tool_name
   * @param orientation_meter
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Matrix3d orientation_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectoryFromPresentPose
   * @param tool_name
   * @param goal_pose_delta
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectoryFromPresentPose(Name tool_name, KinematicPose goal_pose_delta, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectory
   * @param tool_name
   * @param goal_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectory
   * @param tool_name
   * @param goal_orientation
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectory
   * @param tool_name
   * @param goal_pose
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});

  /**
   * @brief setCustomTrajectoryOption
   * @param trajectory_name
   * @param arg
   */
  void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
  /**
   * @brief makeCustomTrajectory
   * @param trajectory_name
   * @param tool_name
   * @param arg
   * @param move_time
   * @param present_joint_value
   */
  bool makeCustomTrajectory(Name trajectory_name, Name tool_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeCustomTrajectory
   * @param trajectory_name
   * @param arg
   * @param move_time
   * @param present_joint_value
   */
  bool makeCustomTrajectory(Name trajectory_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief sleepTrajectory
   * @param wait_time
   * @param present_joint_value
   */
  bool sleepTrajectory(double wait_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeToolTrajectory
   * @param tool_name
   * @param tool_goal_position
   */
  bool makeToolTrajectory(Name tool_name, double tool_goal_position);
  /**
   * @brief getJointGoalValueFromTrajectory
   * @param present_time
   * @return
   */
  std::vector<JointValue> getJointGoalValueFromTrajectory(double present_time);
  /**
   * @brief getToolGoalValue
   * @return
   */
  std::vector<JointValue> getToolGoalValue();
  /**
   * @brief getJointGoalValueFromTrajectoryTickTime
   * @param tick_time
   * @return
   */
  std::vector<JointValue> getJointGoalValueFromTrajectoryTickTime(double tick_time);
};
} // namespace ROBOTIS_MANIPULATOR

#endif
