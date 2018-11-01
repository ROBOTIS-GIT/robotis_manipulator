/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef ROBOTIS_MANAGER_H_
#define ROBOTIS_MANAGER_H_

#include <eigen3/Eigen/Eigen>

#include "robotis_manipulator_common.h"


namespace ROBOTIS_MANIPULATOR
{

class Kinematics : public Manipulator
{
public:
  Kinematics(){};
  virtual ~Kinematics(){};

  virtual void updatePassiveJointValue(Manipulator *manipulator) = 0;
  virtual Eigen::MatrixXd jacobian(Manipulator *manipulator, Name tool_name) = 0;
  virtual void forward(Manipulator *manipulator) = 0;
  virtual void forward(Manipulator *manipulator, Name component_name) = 0;
  virtual std::vector<double> inverse(Manipulator *manipulator, Name tool_name, Pose target_pose) = 0;
};

class JointActuator
{
public:
  JointActuator(){};
  virtual ~JointActuator(){};

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg) = 0;
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg) = 0;
  virtual std::vector<uint8_t> getId() = 0;

  virtual void enable() = 0;
  virtual void disable() = 0;

  virtual bool sendJointActuatorValue(uint8_t actuator_id, Actuator value) = 0;
  virtual bool sendMultipleJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<Actuator> value_vector) = 0;
  virtual Actuator receiveJointActuatorValue(uint8_t actuator_id) = 0;
  virtual std::vector<Actuator> receiveMultipleJointActuatorValue(std::vector<uint8_t> actuator_id) = 0;
};

class ToolActuator
{
public:
  ToolActuator(){};
  virtual ~ToolActuator(){};

  virtual void init(uint8_t actuator_id, const void *arg) = 0;
  virtual void setMode(const void *arg) = 0;
  virtual uint8_t getId() = 0;

  virtual void enable() = 0;
  virtual void disable() = 0;

  virtual bool sendToolActuatorValue(uint8_t actuator_id, double value) = 0;
  virtual double receiveToolActuatorValue(uint8_t actuator_id) = 0;
};


class DrawingTrajectory
{
public:
  WayPointType output_way_point_type_;
  DrawingTrajectory(){};
  virtual ~DrawingTrajectory(){};

  virtual void init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg) = 0; //arg -> ex) radius, goal_pose, meter
  virtual std::vector<WayPoint> getJointWayPoint(double tick) = 0;
  virtual std::vector<WayPoint> getTaskWayPoint(double tick) = 0;

};

} // namespace OPEN_MANIPULATOR
#endif
