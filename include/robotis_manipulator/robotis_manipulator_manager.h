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
 * @file robotis_manipulator_manager.h
 * @brief
 * @details
 */

#ifndef ROBOTIS_MANIPULATOR_MANAGER_H_
#define ROBOTIS_MANIPULATOR_MANAGER_H_

#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
#else
  #include <eigen3/Eigen/Eigen>
#endif

#include "robotis_manipulator_common.h"

namespace robotis_manipulator
{
/**
 * @brief The Kinematics class
 */
class Kinematics
{
public:
  Kinematics() {}
  virtual ~Kinematics() {}

  /**
   * @brief setOption
   * @param arg
   */
  virtual void setOption(const void *arg) = 0;
  /**
   * @brief jacobian
   * @param manipulator
   * @param tool_name
   * @return
   */
  virtual Eigen::MatrixXd jacobian(Manipulator *manipulator, Name tool_name) = 0;
  /**
   * @brief solveForwardKinematics
   * @param manipulator
   */
  virtual void solveForwardKinematics(Manipulator *manipulator) = 0;
  /**
   * @brief solveInverseKinematics
   * @param manipulator
   * @param tool_name
   * @param target_pose
   * @param goal_joint_position
   * @return
   */
  virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_position) = 0;
};

/**
 * @brief The JointActuator class
 */
class JointActuator
{
public:
  bool enabled_state_;

  JointActuator() : enabled_state_(false) {}
  virtual ~JointActuator() {}

  /**
   * @brief init
   * @param actuator_id
   * @param arg
   */
  virtual void init(std::vector<uint8_t> actuator_id, const void *arg) = 0;
  /**
   * @brief setMode
   * @param actuator_id
   * @param arg
   */
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg) = 0;
  /**
   * @brief getId
   * @return
   */
  virtual std::vector<uint8_t> getId() = 0;

  /**
   * @brief enable
   */
  virtual void enable() = 0;
  /**
   * @brief disable
   */
  virtual void disable() = 0;

  /**
   * @brief sendJointActuatorValue
   * @param actuator_id
   * @param value_vector
   * @return
   */
  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ActuatorValue> value_vector) = 0;
  /**
   * @brief receiveJointActuatorValue
   * @param actuator_id
   * @return
   */
  virtual std::vector<ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id) = 0;

  /**
   * @brief findId
   * @param actuator_id
   * @return
   */
  bool findId(uint8_t actuator_id);
  /**
   * @brief getEnabledState
   * @return
   */
  bool getEnabledState();
};

/**
 * @brief The ToolActuator class
 */
class ToolActuator
{
public:
  /**
   * @brief enabled_state_
   */
  bool enabled_state_;

  /**
   * @brief ToolActuator
   */
  ToolActuator():enabled_state_(false){}
  /**
   * @brief ~ToolActuator
   */
  virtual ~ToolActuator() {}

  /**
   * @brief init
   * @param actuator_id
   * @param arg
   */
  virtual void init(uint8_t actuator_id, const void *arg) = 0;
  /**
   * @brief setMode
   * @param arg
   */
  virtual void setMode(const void *arg) = 0;
  /**
   * @brief getId
   * @return
   */
  virtual uint8_t getId() = 0;

  /**
   * @brief enable
   */
  virtual void enable() = 0;
  /**
   * @brief disable
   */
  virtual void disable() = 0;

  /**
   * @brief sendToolActuatorValue
   * @param value
   * @return
   */
  virtual bool sendToolActuatorValue(ActuatorValue value) = 0;
  /**
   * @brief receiveToolActuatorValue
   * @return
   */
  virtual ActuatorValue receiveToolActuatorValue() = 0;

  /**
   * @brief findId
   * @param actuator_id
   * @return
   */
  bool findId(uint8_t actuator_id);
  /**
   * @brief getEnabledState
   * @return
   */
  bool getEnabledState();
};

/**
 * @brief The CustomJointTrajectory class
 */
class CustomJointTrajectory
{
public:
  CustomJointTrajectory() {}
  virtual ~CustomJointTrajectory() {}

  /**
   * @brief makeJointTrajectory
   * @param move_time
   * @param start
   * @param arg
   */
  virtual void makeJointTrajectory(double move_time, JointWaypoint start, const void *arg) = 0;
  /**
   * @brief setOption
   * @param arg
   */
  virtual void setOption(const void *arg) = 0;
  /**
   * @brief getJointWaypoint
   * @param tick
   * @return
   */
  virtual JointWaypoint getJointWaypoint(double tick) = 0;
};

/**
 * @brief The CustomTaskTrajectory class
 */
class CustomTaskTrajectory
{
public:
  CustomTaskTrajectory() {}
  virtual ~CustomTaskTrajectory() {}

  /**
   * @brief makeTaskTrajectory
   * @param move_time
   * @param start
   * @param arg
   */
  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg) = 0; 
  /**
   * @brief setOption
   * @param arg
   */
  virtual void setOption(const void *arg) = 0;
  /**
   * @brief getTaskWaypoint
   * @param tick
   * @return
   */
  virtual TaskWaypoint getTaskWaypoint(double tick) = 0;
};

} // namespace ROBOTIS_MANIPULATOR
#endif
