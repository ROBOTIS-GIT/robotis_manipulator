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

#ifndef RMAPI_H_
#define RMAPI_H_


#include <eigen3/Eigen/Eigen>

#include <vector>
#include <map>

#include "RMManager.h"
#include "RMMath.h"

namespace OPEN_MANIPULATOR
{
class Manager 
{
public:
  Manager(){};
  virtual ~Manager(){};

  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(RM_MANAGER::Manipulator *manipulator,
                Name world_name,
                Name child_name,
                Vector3f world_position = Vector3f::Zero(),
                Matrix3f world_orientation = Matrix3f::Identity(3, 3));

  void addComponent(RM_MANAGER::Manipulator *manipulator,
                    Name me_name,
                    Name parent_name,
                    Name child_name,
                    Vector3f relative_position,
                    Matrix3f relative_orientation,
                    Vector3f axis_of_rotation = Vector3f::Zero(),
                    int8_t actuator_id = -1,
                    double coefficient = 1,
                    double mass = 0.0,
                    Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
                    Vector3f center_of_mass = Vector3f::Zero());

  void addTool(RM_MANAGER::Manipulator *manipulator,
               Name me_name,
               Name parent_name,
               Vector3f relative_position,
               Matrix3f relative_orientation,
               int8_t tool_id = -1,
               double coefficient = 1,
               double mass = 0.0,
               Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
               Vector3f center_of_mass = Vector3f::Zero());

  void addComponentChild(RM_MANAGER::Manipulator *manipulator, Name me_name, Name child_name);
  void checkManipulatorSetting(RM_MANAGER::Manipulator *manipulator);

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(RM_MANAGER::Manipulator *manipulator, Pose world_pose);
  void setWorldPosition(RM_MANAGER::Manipulator *manipulator, Vector3f world_position);
  void setWorldOrientation(RM_MANAGER::Manipulator *manipulator, Matrix3f world_orientation);
  void setWorldState(RM_MANAGER::Manipulator *manipulator, State world_state);
  void setWorldVelocity(RM_MANAGER::Manipulator *manipulator, VectorXf world_velocity);
  void setWorldAcceleration(RM_MANAGER::Manipulator *manipulator, VectorXf world_acceleration);

  void setComponent(RM_MANAGER::Manipulator *manipulator, Name name, Component component);
  void setComponentPoseToWorld(RM_MANAGER::Manipulator *manipulator, Name name, Pose pose_to_world);
  void setComponentPositionToWorld(RM_MANAGER::Manipulator *manipulator, Name name, Vector3f position_to_world);
  void setComponentOrientationToWorld(RM_MANAGER::Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd);
  void setComponentStateToWorld(RM_MANAGER::Manipulator *manipulator, Name name, State state_to_world);
  void setComponentVelocityToWorld(RM_MANAGER::Manipulator *manipulator, Name name, VectorXf velocity);
  void setComponentAccelerationToWorld(RM_MANAGER::Manipulator *manipulator, Name name, VectorXf accelaration);
  void setComponentJointAngle(RM_MANAGER::Manipulator *manipulator, Name name, double angle);
  void setComponentJointVelocity(RM_MANAGER::Manipulator *manipulator, Name name, double angular_velocity);
  void setComponentJointAcceleration(RM_MANAGER::Manipulator *manipulator, Name name, double angular_acceleration);
  void setComponentToolOnOff(RM_MANAGER::Manipulator *manipulator, Name name, bool on_off);
  void setComponentToolValue(RM_MANAGER::Manipulator *manipulator, Name name, double actuator_value);

  void setAllActiveJointAngle(RM_MANAGER::Manipulator *manipulator, std::vector<double> angle_vector);

  ///////////////////////////////Get function//////////////////////////////////

  int8_t getDOF(RM_MANAGER::Manipulator *manipulator);

  Name getWorldName(RM_MANAGER::Manipulator *manipulator);
  Name getWorldChildName(RM_MANAGER::Manipulator *manipulator);
  Pose getWorldPose(RM_MANAGER::Manipulator *manipulator);
  Vector3f getWorldPosition(RM_MANAGER::Manipulator *manipulator);
  Matrix3f getWorldOrientation(RM_MANAGER::Manipulator *manipulator);
  State getWorldState(RM_MANAGER::Manipulator *manipulator);
  VectorXf getWorldVelocity(RM_MANAGER::Manipulator *manipulator);
  VectorXf getWorldAcceleration(RM_MANAGER::Manipulator *manipulator);

  int8_t getComponentSize(RM_MANAGER::Manipulator *manipulator);
  std::map<Name, Component> getAllComponent(RM_MANAGER::Manipulator *manipulator);
  std::map<Name, Component>::iterator getIteratorBegin(RM_MANAGER::Manipulator *manipulator);
  std::map<Name, Component>::iterator getIteratorEnd(RM_MANAGER::Manipulator *manipulator);
  Component getComponent(RM_MANAGER::Manipulator *manipulator, Name name);
  Name getComponentParentName(RM_MANAGER::Manipulator *manipulator, Name name);
  std::vector<Name> getComponentChildName(RM_MANAGER::Manipulator *manipulator, Name name);
  Pose getComponentPoseToWorld(RM_MANAGER::Manipulator *manipulator, Name name);
  Vector3f getComponentPositionToWorld(RM_MANAGER::Manipulator *manipulator, Name name);
  Matrix3f getComponentOrientationToWorld(RM_MANAGER::Manipulator *manipulator, Name name);
  State getComponentStateToWorld(RM_MANAGER::Manipulator *manipulator, Name name);
  VectorXf getComponentVelocityToWorld(RM_MANAGER::Manipulator *manipulator, Name name);
  VectorXf getComponentAccelerationToWorld(RM_MANAGER::Manipulator *manipulator, Name name);
  Pose getComponentRelativePoseToParent(RM_MANAGER::Manipulator *manipulator, Name name);
  Vector3f getComponentRelativePositionToParent(RM_MANAGER::Manipulator *manipulator, Name name);
  Matrix3f getComponentRelativeOrientationToParent(RM_MANAGER::Manipulator *manipulator, Name name);
  Joint getComponentJoint(RM_MANAGER::Manipulator *manipulator, Name name);
  int8_t getComponentJointId(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentJointCoefficient(RM_MANAGER::Manipulator *manipulator, Name name);
  Vector3f getComponentJointAxis(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentJointAngle(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentJointVelocity(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentJointAcceleration(RM_MANAGER::Manipulator *manipulator, Name name);
  Tool getComponentTool(RM_MANAGER::Manipulator *manipulator, Name name);
  int8_t getComponentToolId(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentToolCoefficient(RM_MANAGER::Manipulator *manipulator, Name name);
  bool getComponentToolOnOff(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentToolValue(RM_MANAGER::Manipulator *manipulator, Name name);
  double getComponentMass(RM_MANAGER::Manipulator *manipulator, Name name);
  Matrix3f getComponentInertiaTensor(RM_MANAGER::Manipulator *manipulator, Name name);
  Vector3f getComponentCenterOfMass(RM_MANAGER::Manipulator *manipulator, Name name);

  std::vector<double> getAllJointAngle(RM_MANAGER::Manipulator *manipulator);
  std::vector<double> getAllActiveJointAngle(RM_MANAGER::Manipulator *manipulator);
  std::vector<uint8_t> getAllActiveJointID(RM_MANAGER::Manipulator *manipulator);
};

class Kinematics : public Manager
{
public:
  Kinematics(){};
  virtual ~Kinematics(){};

  virtual MatrixXf jacobian(RM_MANAGER::Manipulator *manipulator, Name tool_name) = 0;
  virtual void forward(RM_MANAGER::Manipulator *manipulator) = 0;
  virtual void forward(RM_MANAGER::Manipulator *manipulator, Name component_name) = 0;
  virtual std::vector<double> inverse(RM_MANAGER::Manipulator *manipulator, Name tool_name, Pose target_pose) = 0;
};

class Actuator
{
public:
  Actuator(){};
  virtual ~Actuator(){};

  virtual void initActuator(const void *arg) = 0;
  virtual void setActuatorControlMode() = 0;

  virtual void Enable() = 0;
  virtual void Disable() = 0;

  virtual bool sendAllActuatorAngle(std::vector<double> radian_vector) = 0;
  virtual bool sendMultipleActuatorAngle(std::vector<uint8_t> id, std::vector<double> radian_vector) = 0;
  virtual bool sendActuatorAngle(uint8_t actuator_id, double radian) = 0;
  virtual bool sendActuatorSignal(uint8_t actuator_id, bool onoff) = 0;
  virtual std::vector<double> receiveAllActuatorAngle(void) = 0;
};

class Draw
{
public:
  Draw(){};
  virtual ~Draw(){};

  virtual void initDraw(const void *arg) = 0;
  virtual void setRadius(double radius) = 0;
  virtual void setStartPose(Pose start_pose) = 0;
  virtual void setEndPose(Pose end_pose) = 0;
  virtual void setAngularStartPosition(double start_position) = 0;
  virtual Pose getPose(double tick) = 0;
};

} // namespace OPEN_MANIPULATOR
#endif
